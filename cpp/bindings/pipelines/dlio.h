#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "evalio/pipeline.h"
#include "evalio/types.h"

// DLIO headers
#include "dlio/dlio.h"
#include "nano_gicp/nano_gicp.h"

// PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

inline evalio::Point to_evalio_point(const PointType& pt) {
  return {
    .x = pt.x,
    .y = pt.y,
    .z = pt.z,
    .intensity = pt.intensity,
    .t = evalio::Duration::from_sec(pt.time),
    .range = 0u,
    .row = 0,
    .col = 0
  };
}

inline PointType to_dlio_point(const evalio::Point& pt) {
  PointType dlio_pt;
  dlio_pt.x = pt.x;
  dlio_pt.y = pt.y;
  dlio_pt.z = pt.z;
  dlio_pt.intensity = pt.intensity;
  dlio_pt.time = pt.t.to_sec();
  return dlio_pt;
}

class DLIO: public evalio::Pipeline {
public:
  DLIO() : lidar_T_imu_(evalio::SE3::identity()) {}

  // Info
  static std::string version() {
    return XSTR(EVALIO_DLIO);
  }

  static std::string name() {
    return "dlio";
  }

  static std::string url() {
    return "https://github.com/vectr-ucla/direct_lidar_inertial_odometry";
  }

  // clang-format off
  EVALIO_SETUP_PARAMS(
    (bool, deskew, true, deskew_),
    (bool, gravity_align, true, gravity_align_),
    (int, icp_max_iter, 32, gicp_max_iter_),
    (double, icp_tolerance, 0.005, gicp_epsilon_),
    (double, keyframe_thresh_dist, 1.0, keyframe_thresh_dist_),
    (double, keyframe_thresh_rot, 15.0, keyframe_thresh_rot_),
    (int, submap_knn, 10, submap_knn_),
    (int, submap_kcv, 10, submap_kcv_),
    (int, submap_kcc, 10, submap_kcc_),
    (bool, initial_pose_estimation, true, initial_pose_estimation_),
    (double, voxel_size, 0.5, voxel_size_),
    (double, scan_context_max_radius, 80.0, scan_context_max_radius_),
    (double, scan_context_resolution, 0.5, scan_context_resolution_)
  );
  // clang-format on

  // Getters
  const evalio::SE3 pose() override {
    Eigen::Quaterniond q(state_.q);
    evalio::SO3 so3 = {.qx = q.x(), .qy = q.y(), .qz = q.z(), .qw = q.w()};
    return evalio::SE3(so3, state_.p) * lidar_T_imu_;
  }

  const std::map<std::string, std::vector<evalio::Point>> map() override {
    std::vector<evalio::Point> evalio_map;
    if (submap_cloud_) {
      evalio_map.reserve(submap_cloud_->points.size());
      for (const auto& pt : submap_cloud_->points) {
        evalio_map.push_back(to_evalio_point(pt));
      }
    }
    return {{"point", evalio_map}};
  }

  // Setters
  void set_imu_params(evalio::ImuParams params) override {
    imu_accel_noise_ = params.accel;
    imu_gyro_noise_ = params.gyro;
    imu_accel_bias_noise_ = params.accel_bias;
    imu_gyro_bias_noise_ = params.gyro_bias;
    gravity_ = params.gravity;
  }

  void set_lidar_params(evalio::LidarParams params) override {
    lidar_max_range_ = params.max_range;
    lidar_min_range_ = params.min_range;
  }

  void set_imu_T_lidar(evalio::SE3 T) override {
    lidar_T_imu_ = T.inverse();
    Eigen::Quaterniond q = lidar_T_imu_.rot.toEigen();
    extrinsics_.baselink2imu.block<3,3>(0,0) = q.toRotationMatrix();
    extrinsics_.baselink2imu.block<3,1>(0,3) = lidar_T_imu_.trans;
    extrinsics_.baselink2imu(3,3) = 1.0;
    extrinsics_.baselink2lidar = Eigen::Matrix4d::Identity();
  }

  // Doers
  void initialize() override {
    // Initialize Nano-GICP
    gicp_ = std::make_shared<nano_gicp::NanoGICP<PointType, PointType>>();
    gicp_->setNumThreads(4);
    gicp_->setMaxCorrespondenceDistance(1.0);
    gicp_->setMaximumIterations(gicp_max_iter_);
    gicp_->setTransformationEpsilon(gicp_epsilon_);
    gicp_->setCorrespondenceRandomness(gicp_k_correspondences_);
    
    // Initialize voxel grid filter
    voxel_grid_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    
    // Initialize state
    state_.p = Eigen::Vector3d::Zero();
    state_.v = Eigen::Vector3d::Zero();
    state_.q = Eigen::Quaterniond::Identity();
    state_.bg = Eigen::Vector3d::Zero();
    state_.ba = Eigen::Vector3d::Zero();
    
    // Initialize clouds
    current_scan_.reset(new pcl::PointCloud<PointType>);
    submap_cloud_.reset(new pcl::PointCloud<PointType>);
    
    first_scan_ = true;
  }

  void add_imu(evalio::ImuMeasurement mm) override {
    ImuMeas imu_meas;
    imu_meas.stamp = mm.stamp.to_sec();
    imu_meas.ang_vel = mm.gyro.cast<double>();
    imu_meas.lin_accel = mm.accel.cast<double>();
    if (imu_buffer_.size() > 0) {
      imu_meas.dt = imu_meas.stamp - imu_buffer_.back().stamp;
    } else {
      imu_meas.dt = 0.0;
    }
    imu_buffer_.push_back(imu_meas);
    if (imu_buffer_.size() > 10000) {
      imu_buffer_.pop_front();
    }
  }

  std::map<std::string, std::vector<evalio::Point>>
  add_lidar(evalio::LidarMeasurement mm) override {
    pcl::PointCloud<PointType>::Ptr original_scan(new pcl::PointCloud<PointType>);
    original_scan->points.reserve(mm.points.size());
    
    for (const auto& pt : mm.points) {
      double range = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
      if (range >= lidar_min_range_ && range <= lidar_max_range_) {
        original_scan->points.push_back(to_dlio_point(pt));
      }
    }
    original_scan->width = original_scan->points.size();
    original_scan->height = 1;
    original_scan->is_dense = false;

    double scan_stamp = mm.stamp.to_sec();
    
    if (first_scan_) {
      first_scan_ = false;
      prev_scan_stamp_ = scan_stamp;
      lidar_pose_.p = Eigen::Vector3d::Zero();
      lidar_pose_.q = Eigen::Quaterniond::Identity();
      T_prior_ = Eigen::Matrix4d::Identity();
      T_corr_ = Eigen::Matrix4d::Identity();
      T_ = Eigen::Matrix4d::Identity();
      
      if (deskew_) {
        deskewPointcloud(original_scan, scan_stamp);
      } else {
        current_scan_ = original_scan;
      }
      
      voxel_grid_.setInputCloud(current_scan_);
      pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>);
      voxel_grid_.filter(*filtered);
      current_scan_ = filtered;
      
      gicp_->setInputSource(current_scan_);
      gicp_->calculateSourceCovariances();
      
      submap_cloud_ = current_scan_;
      gicp_->setInputTarget(submap_cloud_);
      
      keyframes_.push_back({lidar_pose_, current_scan_});
      
      std::vector<evalio::Point> result;
      result.reserve(current_scan_->points.size());
      for (const auto& pt : current_scan_->points) {
        result.push_back(to_evalio_point(pt));
      }
      return {{"point", result}};
    }

    if (initial_pose_estimation_ && imu_buffer_.size() >= 2) {
      Eigen::Quaternionf q_init = state_.q.cast<float>();
      Eigen::Vector3f p_init = state_.p.cast<float>();
      Eigen::Vector3f v_init = state_.v.cast<float>();
      std::vector<double> timestamps = {scan_stamp};
      auto frames = integrateImu(prev_scan_stamp_, q_init, p_init, v_init, timestamps);
      if (frames.size() > 0) {
        T_prior_ = frames.back().cast<double>();
        propagateState();
      }
    } else {
      T_prior_ = T_;
    }

    if (deskew_) {
      deskewPointcloud(original_scan, scan_stamp);
    } else {
      current_scan_ = original_scan;
    }

    voxel_grid_.setInputCloud(current_scan_);
    pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>);
    voxel_grid_.filter(*filtered);
    current_scan_ = filtered;

    if (current_scan_->points.size() < 10) {
      std::vector<evalio::Point> result;
      return {{"point", result}};
    }

    gicp_->setInputSource(current_scan_);
    gicp_->calculateSourceCovariances();
    gicp_->setInputTarget(submap_cloud_);
    
    pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>);
    Eigen::Matrix4f guess = T_prior_.cast<float>();
    gicp_->align(*aligned, guess);
    
    if (gicp_->hasConverged()) {
      T_corr_ = gicp_->getFinalTransformation().cast<double>();
      T_ = T_corr_ * T_prior_;
      
      propagateGICP();
      
      updateState(scan_stamp);
      
      bool new_keyframe = updateKeyframes();
      if (new_keyframe) {
        buildSubmap();
      }
    }
    
    prev_scan_stamp_ = scan_stamp;
    
    std::vector<evalio::Point> result;
    result.reserve(aligned->points.size());
    for (const auto& pt : aligned->points) {
      result.push_back(to_evalio_point(pt));
    }
    return {{"point", result}};
  }

private:
  struct ImuMeas {
    double stamp;
    double dt;
    Eigen::Vector3d ang_vel;
    Eigen::Vector3d lin_accel;
  };
  
  struct LidarPose {
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
  };
  
  struct State {
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Quaterniond q;
    Eigen::Vector3d bg;
    Eigen::Vector3d ba;
  };
  
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
  integrateImu(double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init,
               Eigen::Vector3f v_init, const std::vector<double>& sorted_timestamps) {
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> imu_se3;
    
    if (sorted_timestamps.empty() || imu_buffer_.size() < 2) {
      return imu_se3;
    }
    
    auto imu_it = imu_buffer_.begin();
    while (imu_it != imu_buffer_.end() && imu_it->stamp < start_time) {
      imu_it++;
    }
    if (imu_it == imu_buffer_.end() || imu_it == imu_buffer_.begin()) {
      return imu_se3;
    }
    
    auto begin_imu_it = imu_it - 1;
    auto end_imu_it = imu_buffer_.end() - 1;
    
    if (begin_imu_it->stamp > sorted_timestamps.back()) {
      return imu_se3;
    }
    
    Eigen::Quaternionf q = q_init;
    Eigen::Vector3f p = p_init;
    Eigen::Vector3f v = v_init;
    
    auto prev_imu_it = begin_imu_it;
    auto stamp_it = sorted_timestamps.begin();
    
    for (auto imu_it = begin_imu_it + 1; imu_it != end_imu_it && imu_it != imu_buffer_.end(); imu_it++) {
      const ImuMeas& f0 = *prev_imu_it;
      const ImuMeas& f = *imu_it;
      
      double dt = f.dt;
      if (dt <= 0) continue;
      
      Eigen::Vector3f alpha_dt = (f.ang_vel - f0.ang_vel).cast<float>();
      Eigen::Vector3f alpha = alpha_dt / dt;
      Eigen::Vector3f omega = f0.ang_vel.cast<float>() + 0.5f * alpha_dt;
      
      q = Eigen::Quaternionf(
        q.w() - 0.5f * (q.x()*omega[0] + q.y()*omega[1] + q.z()*omega[2]) * dt,
        q.x() + 0.5f * (q.w()*omega[0] - q.z()*omega[1] + q.y()*omega[2]) * dt,
        q.y() + 0.5f * (q.z()*omega[0] + q.w()*omega[1] - q.x()*omega[2]) * dt,
        q.z() + 0.5f * (q.x()*omega[1] - q.y()*omega[0] + q.w()*omega[2]) * dt
      );
      q.normalize();
      
      Eigen::Vector3f a0 = q._transformVector(f0.lin_accel.cast<float>());
      a0[2] -= static_cast<float>(gravity_[2]);
      Eigen::Vector3f a = q._transformVector(f.lin_accel.cast<float>());
      a[2] -= static_cast<float>(gravity_[2]);
      
      Eigen::Vector3f j_dt = a - a0;
      Eigen::Vector3f j = j_dt / dt;
      
      while (stamp_it != sorted_timestamps.end() && *stamp_it <= f.stamp) {
        double idt = *stamp_it - f0.stamp;
        Eigen::Vector3f omega_i = f0.ang_vel.cast<float>() + 0.5f * alpha * idt;
        
        Eigen::Quaternionf q_i(
          q.w() - 0.5f * (q.x()*omega_i[0] + q.y()*omega_i[1] + q.z()*omega_i[2]) * idt,
          q.x() + 0.5f * (q.w()*omega_i[0] - q.z()*omega_i[1] + q.y()*omega_i[2]) * idt,
          q.y() + 0.5f * (q.z()*omega_i[0] + q.w()*omega_i[1] - q.x()*omega_i[2]) * idt,
          q.z() + 0.5f * (q.x()*omega_i[1] - q.y()*omega_i[0] + q.w()*omega_i[2]) * idt
        );
        q_i.normalize();
        
        Eigen::Vector3f p_i = p + v*idt + 0.5f*a0*idt*idt + (1.0f/6.0f)*j*idt*idt*idt;
        
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3,3>(0,0) = q_i.toRotationMatrix();
        T.block<3,1>(0,3) = p_i;
        
        imu_se3.push_back(T);
        stamp_it++;
      }
      
      p += v*dt + 0.5f*a0*dt*dt + (1.0f/6.0f)*j_dt*dt*dt;
      v += a0*dt + 0.5f*j_dt*dt;
      
      prev_imu_it = imu_it;
    }
    
    return imu_se3;
  }
  
  void deskewPointcloud(pcl::PointCloud<PointType>::Ptr& scan, double scan_stamp) {
    if (imu_buffer_.size() < 2 || !deskew_) {
      current_scan_ = scan;
      return;
    }
    
    std::vector<double> timestamps;
    timestamps.reserve(scan->points.size());
    for (const auto& pt : scan->points) {
      timestamps.push_back(scan_stamp + pt.time);
    }
    std::sort(timestamps.begin(), timestamps.end());
    
    if (timestamps.empty()) {
      current_scan_ = scan;
      return;
    }
    
    double median_time = timestamps[timestamps.size() / 2];
    std::vector<double> sorted_timestamps = {median_time};
    
    auto frames = integrateImu(prev_scan_stamp_, lidar_pose_.q.cast<float>(),
                                lidar_pose_.p.cast<float>(),
                                Eigen::Vector3f::Zero(), sorted_timestamps);
    
    if (frames.empty()) {
      T_prior_ = Eigen::Matrix4d::Identity();
      current_scan_ = scan;
      return;
    }
    
    T_prior_ = frames[0].cast<double>();
    
    pcl::PointCloud<PointType>::Ptr deskewed(new pcl::PointCloud<PointType>);
    deskewed->points.reserve(scan->points.size());
    
    for (size_t i = 0; i < scan->points.size(); i++) {
      double pt_time = scan_stamp + scan->points[i].time;
      double dt = pt_time - median_time;
      
      Eigen::Vector3f omega = Eigen::Vector3f::Zero();
      if (imu_buffer_.size() > 1) {
        auto it = imu_buffer_.begin();
        while (it != imu_buffer_.end() && it->stamp < pt_time) it++;
        if (it != imu_buffer_.begin() && it != imu_buffer_.end()) {
          omega = ((it->ang_vel + (it-1)->ang_vel) * 0.5).cast<float>();
        }
      }
      
      Eigen::Quaternionf dq(
        1.0f, 0.5f*omega[0]*dt, 0.5f*omega[1]*dt, 0.5f*omega[2]*dt
      );
      dq.normalize();
      
      Eigen::Matrix4f T_pt = Eigen::Matrix4f::Identity();
      T_pt.block<3,3>(0,0) = dq.toRotationMatrix();
      
      Eigen::Vector4f pt_vec(scan->points[i].x, scan->points[i].y, scan->points[i].z, 1.0f);
      pt_vec = T_prior_.cast<float>() * T_pt * pt_vec;
      
      PointType pt;
      pt.x = pt_vec[0];
      pt.y = pt_vec[1];
      pt.z = pt_vec[2];
      pt.intensity = scan->points[i].intensity;
      pt.time = scan->points[i].time;
      deskewed->points.push_back(pt);
    }
    
    deskewed->width = deskewed->points.size();
    deskewed->height = 1;
    deskewed->is_dense = false;
    current_scan_ = deskewed;
  }
  
  void propagateState() {
    if (imu_buffer_.size() < 2) return;
    
    ImuMeas latest_imu = imu_buffer_.back();
    double dt = latest_imu.dt;
    if (dt <= 0) return;
    
    Eigen::Quaterniond qhat = state_.q;
    
    Eigen::Vector3d world_accel = qhat.toRotationMatrix() * (latest_imu.lin_accel - state_.ba);
    
    state_.p += state_.v * dt + 0.5 * dt * dt * (world_accel - gravity_);
    state_.v += (world_accel - gravity_) * dt;
    
    Eigen::Vector3d omega = latest_imu.ang_vel - state_.bg;
    Eigen::Quaterniond dq;
    dq.w() = 0.5 * dt * (-qhat.x() * omega[0] - qhat.y() * omega[1] - qhat.z() * omega[2]);
    dq.x() = 0.5 * dt * (qhat.w() * omega[0] + qhat.y() * omega[2] - qhat.z() * omega[1]);
    dq.y() = 0.5 * dt * (qhat.w() * omega[1] - qhat.x() * omega[2] + qhat.z() * omega[0]);
    dq.z() = 0.5 * dt * (qhat.w() * omega[2] + qhat.x() * omega[1] - qhat.y() * omega[0]);
    
    state_.q.w() += dq.w();
    state_.q.x() += dq.x();
    state_.q.y() += dq.y();
    state_.q.z() += dq.z();
    state_.q.normalize();
  }
  
  void propagateGICP() {
    lidar_pose_.p = T_.block<3,1>(0,3);
    
    Eigen::Matrix3d rotSO3 = T_.block<3,3>(0,0);
    Eigen::Quaterniond q(rotSO3);
    
    double norm = std::sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
    if (norm > 1e-10) {
      q.w() /= norm;
      q.x() /= norm;
      q.y() /= norm;
      q.z() /= norm;
    }
    lidar_pose_.q = q;
  }
  
  void updateState(double scan_stamp) {
    double dt = scan_stamp - prev_scan_stamp_;
    if (dt <= 0) dt = 0.1;
    
    Eigen::Vector3d pin = lidar_pose_.p;
    Eigen::Quaterniond qin = lidar_pose_.q;
    Eigen::Quaterniond qhat = state_.q;
    
    Eigen::Quaterniond qe = qhat.conjugate() * qin;
    
    double sgn = 1.0;
    if (qe.w() < 0) {
      sgn = -1.0;
    }
    
    Eigen::Quaterniond qcorr;
    qcorr.w() = 1.0 - std::abs(qe.w());
    qcorr.vec() = sgn * qe.vec();
    qcorr = qhat * qcorr;
    qcorr.normalize();
    
    Eigen::Vector3d err = pin - state_.p;
    Eigen::Vector3d err_body = qhat.conjugate().toRotationMatrix() * err;
    
    double geo_Kp = 1.0;
    double geo_Kv = 1.0;
    double geo_Kq = 1.0;
    double geo_Kab = 0.1;
    double geo_Kgb = 0.1;
    double abias_max = 0.1;
    double gbias_max = 0.01;
    
    state_.ba -= dt * geo_Kab * err_body;
    state_.ba = state_.ba.array().min(abias_max).max(-abias_max);
    
    state_.bg[0] -= dt * geo_Kgb * qe.w() * qe.x();
    state_.bg[1] -= dt * geo_Kgb * qe.w() * qe.y();
    state_.bg[2] -= dt * geo_Kgb * qe.w() * qe.z();
    state_.bg = state_.bg.array().min(gbias_max).max(-gbias_max);
    
    state_.p += dt * geo_Kp * err;
    state_.v += dt * geo_Kv * err;
    
    state_.q.w() += dt * geo_Kq * (qcorr.w() - state_.q.w());
    state_.q.x() += dt * geo_Kq * (qcorr.x() - state_.q.x());
    state_.q.y() += dt * geo_Kq * (qcorr.y() - state_.q.y());
    state_.q.z() += dt * geo_Kq * (qcorr.z() - state_.q.z());
    state_.q.normalize();
  }
  
  bool updateKeyframes() {
    if (keyframes_.empty()) return true;
    
    double closest_d = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    
    for (size_t i = 0; i < keyframes_.size(); i++) {
      double d = (state_.p - keyframes_[i].first.p).norm();
      if (d < closest_d) {
        closest_d = d;
        closest_idx = i;
      }
    }
    
    Eigen::Quaterniond dq = state_.q * keyframes_[closest_idx].first.q.conjugate();
    if (dq.w() < 0) {
      dq.w() = -dq.w();
      dq.vec() = -dq.vec();
    }
    double theta_deg = 2.0 * std::atan2(dq.vec().norm(), std::abs(dq.w())) * 180.0 / 3.14159265358979323846;
    
    if (closest_d > keyframe_thresh_dist_ || std::abs(theta_deg) > keyframe_thresh_rot_) {
      keyframes_.push_back({lidar_pose_, current_scan_});
      if (keyframes_.size() > 1000) {
        keyframes_.erase(keyframes_.begin());
      }
      return true;
    }
    
    return false;
  }
  
  void buildSubmap() {
    if (keyframes_.empty()) return;
    
    pcl::PointCloud<PointType>::Ptr submap(new pcl::PointCloud<PointType>);
    
    size_t num_kf = std::min(static_cast<size_t>(submap_knn_), keyframes_.size());
    size_t start_idx = keyframes_.size() > num_kf ? keyframes_.size() - num_kf : 0;
    
    for (size_t i = start_idx; i < keyframes_.size(); i++) {
      *submap += *keyframes_[i].second;
    }
    
    voxel_grid_.setInputCloud(submap);
    pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>);
    voxel_grid_.filter(*filtered);
    
    submap_cloud_ = filtered;
    gicp_->setInputTarget(submap_cloud_);
    gicp_->calculateTargetCovariances();
  }
  
  State state_;
  LidarPose lidar_pose_;
  std::deque<ImuMeas> imu_buffer_;
  std::vector<std::pair<LidarPose, pcl::PointCloud<PointType>::Ptr>> keyframes_;
  double prev_scan_stamp_ = 0.0;
  Eigen::Matrix4d T_prior_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_corr_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_ = Eigen::Matrix4d::Identity();
  
  std::shared_ptr<nano_gicp::NanoGICP<PointType, PointType>> gicp_;
  pcl::PointCloud<PointType>::Ptr current_scan_;
  pcl::PointCloud<PointType>::Ptr submap_cloud_;
  pcl::VoxelGrid<PointType> voxel_grid_;
  
  struct Extrinsics {
    Eigen::Matrix4d baselink2imu = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d baselink2lidar = Eigen::Matrix4d::Identity();
  };
  Extrinsics extrinsics_;
  
  // Parameters
  bool deskew_ = true;
  bool gravity_align_ = true;
  int gicp_max_iter_ = 32;
  double gicp_epsilon_ = 0.005;
  int gicp_k_correspondences_ = 20;
  double keyframe_thresh_dist_ = 1.0;
  double keyframe_thresh_rot_ = 15.0;
  int submap_knn_ = 10;
  int submap_kcv_ = 10;
  int submap_kcc_ = 10;
  bool initial_pose_estimation_ = true;
  double voxel_size_ = 0.5;
  double scan_context_max_radius_ = 80.0;
  double scan_context_resolution_ = 0.5;
  double lidar_max_range_ = 100.0;
  double lidar_min_range_ = 0.5;
  
  // IMU parameters
  double imu_accel_noise_ = 0.1;
  double imu_gyro_noise_ = 0.01;
  double imu_accel_bias_noise_ = 0.001;
  double imu_gyro_bias_noise_ = 0.0001;
  Eigen::Vector3d gravity_ = Eigen::Vector3d(0, 0, -9.81);
  
  evalio::SE3 lidar_T_imu_;
  bool first_scan_ = true;
};