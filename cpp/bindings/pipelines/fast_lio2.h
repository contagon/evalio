#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <vector>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "evalio/pipeline.h"
#include "evalio/types.h"

typedef pcl::PointXYZINormal FastLIO2PointType;
typedef pcl::PointCloud<FastLIO2PointType> FastLIO2PointCloud;

inline evalio::Point to_evalio_point(const FastLIO2PointType& pt) {
  return {
    .x = pt.x,
    .y = pt.y,
    .z = pt.z,
    .intensity = pt.intensity,
    .t = evalio::Duration::from_sec(0),
    .range = 0u,
    .row = 0,
    .col = 0
  };
}

inline FastLIO2PointType to_pcl_point(const evalio::Point& pt) {
  FastLIO2PointType pcl_pt;
  pcl_pt.x = pt.x;
  pcl_pt.y = pt.y;
  pcl_pt.z = pt.z;
  pcl_pt.intensity = pt.intensity;
  return pcl_pt;
}

class FastLIO2: public evalio::Pipeline {
public:
  FastLIO2() : lidar_T_imu_(evalio::SE3::identity()) {}

  static std::string version() {
    return XSTR(EVALIO_FAST_LIO2);
  }

  static std::string name() {
    return "fast_lio2";
  }

  static std::string url() {
    return "https://github.com/hku-mars/FAST_LIO";
  }

  EVALIO_SETUP_PARAMS(
    (double, filter_size_surf, 0.5, filter_size_surf_),
    (double, filter_size_map, 0.5, filter_size_map_),
    (double, cube_side_length, 200.0, cube_len_),
    (double, det_range, 300.0, det_range_),
    (int, max_iteration, 4, max_iteration_),
    (double, epsi, 0.001, epsi_),
    (bool, pcd_save_en, false, pcd_save_en_),
    (double, lidar_max_range, 100.0, lidar_max_range_),
    (double, lidar_min_range, 0.5, lidar_min_range_)
  );

  const evalio::SE3 pose() override {
    if (!initialized_) {
      return evalio::SE3::identity();
    }
    Eigen::Quaterniond q(state_q_);
    evalio::SO3 so3 = {.qx = q.x(), .qy = q.y(), .qz = q.z(), .qw = q.w()};
    return evalio::SE3(so3, state_p_) * lidar_T_imu_;
  }

  const std::map<std::string, std::vector<evalio::Point>> map() override {
    std::vector<evalio::Point> evalio_map;
    evalio_map.reserve(map_cloud_->points.size());
    for (const auto& pt : map_cloud_->points) {
      evalio_map.push_back(to_evalio_point(pt));
    }
    return {{"point", evalio_map}};
  }

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
    num_scan_lines_ = params.num_rows;
  }

  void set_imu_T_lidar(evalio::SE3 T) override {
    lidar_T_imu_ = T.inverse();
  }

  void initialize() override {
    state_p_ = Eigen::Vector3d::Zero();
    state_v_ = Eigen::Vector3d::Zero();
    state_q_ = Eigen::Quaterniond::Identity();
    state_bg_ = Eigen::Vector3d::Zero();
    state_ba_ = Eigen::Vector3d::Zero();
    
    map_cloud_.reset(new FastLIO2PointCloud);
    feats_undistort_.reset(new FastLIO2PointCloud);
    feats_down_.reset(new FastLIO2PointCloud);
    
    downSizeFilterSurf_.setLeafSize(filter_size_surf_, filter_size_surf_, filter_size_surf_);
    downSizeFilterMap_.setLeafSize(filter_size_map_, filter_size_map_, filter_size_map_);
    
    first_scan_ = true;
    initialized_ = false;
    prev_lidar_time_ = 0.0;
  }

  void add_imu(evalio::ImuMeasurement mm) override {
    ImuMeas imu_meas;
    imu_meas.timestamp = mm.stamp.to_sec();
    imu_meas.accel = mm.accel;
    imu_meas.gyro = mm.gyro;
    if (imu_buffer_.size() > 0) {
      imu_meas.dt = imu_meas.timestamp - imu_buffer_.back().timestamp;
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
    double lidar_time = mm.stamp.to_sec();
    
    FastLIO2PointCloud::Ptr cloud(new FastLIO2PointCloud);
    cloud->points.reserve(mm.points.size());
    for (const auto& pt : mm.points) {
      double range = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
      if (range >= lidar_min_range_ && range <= lidar_max_range_) {
        cloud->points.push_back(to_pcl_point(pt));
      }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    if (first_scan_) {
      first_scan_ = false;
      prev_lidar_time_ = lidar_time;
      *feats_undistort_ = *cloud;
      downSizeFilterSurf_.setInputCloud(feats_undistort_);
      downSizeFilterSurf_.filter(*feats_down_);
      
      if (feats_down_->points.size() > 5) {
        *map_cloud_ = *feats_down_;
        downSizeFilterMap_.setInputCloud(map_cloud_);
        downSizeFilterMap_.filter(*map_cloud_);
        initialized_ = true;
      }
      
      std::vector<evalio::Point> result;
      result.reserve(feats_down_->points.size());
      for (const auto& pt : feats_down_->points) {
        result.push_back(to_evalio_point(pt));
      }
      return {{"point", result}};
    }

    if (imu_buffer_.size() < 2) {
      std::vector<evalio::Point> result;
      return {{"point", result}};
    }

    propagateState(prev_lidar_time_, lidar_time);
    
    *feats_undistort_ = *cloud;
    downSizeFilterSurf_.setInputCloud(feats_undistort_);
    downSizeFilterSurf_.filter(*feats_down_);

    if (feats_down_->points.size() < 5 || !initialized_) {
      prev_lidar_time_ = lidar_time;
      std::vector<evalio::Point> result;
      return {{"point", result}};
    }

    updateState(*feats_down_);
    
    FastLIO2PointVector points_to_add;
    points_to_add.reserve(feats_down_->points.size());
    for (const auto& pt : feats_down_->points) {
      Eigen::Vector3d p_body(pt.x, pt.y, pt.z);
      Eigen::Vector3d p_world = state_q_.toRotationMatrix() * p_body + state_p_;
      FastLIO2PointType pt_world;
      pt_world.x = p_world.x();
      pt_world.y = p_world.y();
      pt_world.z = p_world.z();
      pt_world.intensity = pt.intensity;
      points_to_add.push_back(pt_world);
    }
    
    FastLIO2PointCloud::Ptr map_update(new FastLIO2PointCloud);
    map_update->points = points_to_add;
    *map_cloud_ += *map_update;
    downSizeFilterMap_.setInputCloud(map_cloud_);
    downSizeFilterMap_.filter(*map_cloud_);

    prev_lidar_time_ = lidar_time;
    
    std::vector<evalio::Point> result;
    result.reserve(feats_down_->points.size());
    for (const auto& pt : feats_down_->points) {
      result.push_back(to_evalio_point(pt));
    }
    return {{"point", result}};
  }

private:
  struct ImuMeas {
    double timestamp;
    double dt;
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;
  };
  
  typedef std::vector<FastLIO2PointType, Eigen::aligned_allocator<FastLIO2PointType>> FastLIO2PointVector;

  void propagateState(double start_time, double end_time) {
    if (imu_buffer_.empty()) return;
    
    Eigen::Quaterniond q = state_q_;
    Eigen::Vector3d p = state_p_;
    Eigen::Vector3d v = state_v_;
    
    auto it = imu_buffer_.begin();
    while (it != imu_buffer_.end() && it->timestamp < start_time) {
      it++;
    }
    if (it == imu_buffer_.end() || it == imu_buffer_.begin()) {
      return;
    }
    
    auto prev_it = it - 1;
    double current_time = std::max(start_time, prev_it->timestamp);
    
    while (it != imu_buffer_.end() && it->timestamp <= end_time) {
      double dt = it->timestamp - current_time;
      if (dt <= 0) {
        it++;
        continue;
      }
      
      Eigen::Vector3d accel = it->accel - state_ba_;
      Eigen::Vector3d gyro = it->gyro - state_bg_;
      
      Eigen::Vector3d accel_world = q.toRotationMatrix() * accel;
      
      p += v * dt + 0.5 * (accel_world + gravity_) * dt * dt;
      v += (accel_world + gravity_) * dt;
      
      Eigen::Quaterniond dq(1.0, 0.5*gyro.x()*dt, 0.5*gyro.y()*dt, 0.5*gyro.z()*dt);
      dq.normalize();
      q = q * dq;
      q.normalize();
      
      current_time = it->timestamp;
      prev_it = it;
      it++;
    }
    
    if (prev_it != imu_buffer_.end() && end_time > current_time) {
      double dt = end_time - current_time;
      Eigen::Vector3d accel = prev_it->accel - state_ba_;
      Eigen::Vector3d gyro = prev_it->gyro - state_bg_;
      
      Eigen::Vector3d accel_world = q.toRotationMatrix() * accel;
      
      p += v * dt + 0.5 * (accel_world + gravity_) * dt * dt;
      v += (accel_world + gravity_) * dt;
      
      Eigen::Quaterniond dq(1.0, 0.5*gyro.x()*dt, 0.5*gyro.y()*dt, 0.5*gyro.z()*dt);
      dq.normalize();
      q = q * dq;
      q.normalize();
    }
    
    state_q_ = q;
    state_p_ = p;
    state_v_ = v;
  }
  
  void updateState(const FastLIO2PointCloud& feats) {
    if (map_cloud_->points.size() < 5) return;
    
    const int NUM_MATCH_POINTS = 5;
    FastLIO2PointVector nearest_points;
    std::vector<float> point_search_sq_dis;
    
    for (int iter = 0; iter < max_iteration_; iter++) {
      Eigen::MatrixXd H = Eigen::MatrixXd::Zero(feats.points.size() * 3, 6);
      Eigen::VectorXd b = Eigen::VectorXd::Zero(feats.points.size() * 3);
      
      int valid_count = 0;
      for (size_t i = 0; i < feats.points.size(); i++) {
        Eigen::Vector3d p_body(feats.points[i].x, feats.points[i].y, feats.points[i].z);
        Eigen::Vector3d p_world = state_q_.toRotationMatrix() * p_body + state_p_;
        
        std::vector<int> point_search_idx;
        std::vector<float> point_search_dis;
        
        pcl::KdTreeFLANN<FastLIO2PointType> kdtree;
        kdtree.setInputCloud(map_cloud_);
        FastLIO2PointType search_point;
        search_point.x = p_world.x();
        search_point.y = p_world.y();
        search_point.z = p_world.z();
        
        if (kdtree.nearestKSearch(search_point, NUM_MATCH_POINTS, point_search_idx, point_search_dis) < NUM_MATCH_POINTS) {
          continue;
        }
        
        if (point_search_dis[NUM_MATCH_POINTS - 1] > 5.0) {
          continue;
        }
        
        Eigen::Matrix<double, NUM_MATCH_POINTS, 3> A;
        Eigen::Matrix<double, NUM_MATCH_POINTS, 1> b_plane;
        b_plane.setOnes();
        b_plane *= -1.0;
        
        for (int j = 0; j < NUM_MATCH_POINTS; j++) {
          const auto& pt = map_cloud_->points[point_search_idx[j]];
          A(j, 0) = pt.x;
          A(j, 1) = pt.y;
          A(j, 2) = pt.z;
        }
        
        Eigen::Vector3d normvec = A.colPivHouseholderQr().solve(b_plane);
        double n = normvec.norm();
        if (n < 1e-6) continue;
        normvec /= n;
        
        double pd2 = normvec.dot(p_world) + 1.0 / n;
        
        if (std::abs(pd2) > 0.1) continue;
        
        Eigen::Matrix<double, 1, 6> J;
        Eigen::Matrix3d R = state_q_.toRotationMatrix();
        Eigen::Vector3d p_skew(-p_body.y(), p_body.x(), 0.0);
        J.block<1, 3>(0, 0) = normvec.transpose();
        J.block<1, 3>(0, 3) = normvec.transpose() * R * skewSymmetric(p_body);
        
        H.block<1, 6>(valid_count * 3, 0) = J;
        b(valid_count * 3) = -pd2;
        valid_count++;
      }
      
      if (valid_count < 3) break;
      
      H.conservativeResize(valid_count * 3, 6);
      b.conservativeResize(valid_count * 3);
      
      Eigen::Matrix<double, 6, 6> HtH = H.transpose() * H;
      Eigen::Matrix<double, 6, 1> Htb = H.transpose() * b;
      
      Eigen::Matrix<double, 6, 1> delta = HtH.ldlt().solve(-Htb);
      
      if (delta.norm() < epsi_) {
        break;
      }
      
      state_p_ += delta.head<3>();
      Eigen::Quaterniond dq(1.0, 0.5*delta(3), 0.5*delta(4), 0.5*delta(5));
      dq.normalize();
      state_q_ = state_q_ * dq;
      state_q_.normalize();
    }
  }
  
  Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
         -v.y(), v.x(), 0;
    return m;
  }

  Eigen::Vector3d state_p_;
  Eigen::Vector3d state_v_;
  Eigen::Quaterniond state_q_;
  Eigen::Vector3d state_bg_;
  Eigen::Vector3d state_ba_;
  
  std::deque<ImuMeas> imu_buffer_;
  
  FastLIO2PointCloud::Ptr map_cloud_;
  FastLIO2PointCloud::Ptr feats_undistort_;
  FastLIO2PointCloud::Ptr feats_down_;
  
  pcl::VoxelGrid<FastLIO2PointType> downSizeFilterSurf_;
  pcl::VoxelGrid<FastLIO2PointType> downSizeFilterMap_;
  
  double filter_size_surf_ = 0.5;
  double filter_size_map_ = 0.5;
  double cube_len_ = 200.0;
  double det_range_ = 300.0;
  int max_iteration_ = 4;
  double epsi_ = 0.001;
  bool pcd_save_en_ = false;
  double lidar_max_range_ = 100.0;
  double lidar_min_range_ = 0.5;
  int num_scan_lines_ = 16;
  
  double imu_accel_noise_ = 0.1;
  double imu_gyro_noise_ = 0.01;
  double imu_accel_bias_noise_ = 0.001;
  double imu_gyro_bias_noise_ = 0.0001;
  Eigen::Vector3d gravity_ = Eigen::Vector3d(0, 0, -9.81);
  
  evalio::SE3 lidar_T_imu_;
  bool first_scan_ = true;
  bool initialized_ = false;
  double prev_lidar_time_ = 0.0;
};
