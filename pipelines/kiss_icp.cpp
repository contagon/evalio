
#include <memory>
#include <stdexcept>

#include "kiss_icp/pipeline/KissICP.hpp"
#include "pipeline.h"

class KissICP : Pipeline {
    public:
        KissICP() : config_() {};

        void add_imu(Eigen::Vector3d gyro, Eigen::Vector3d acc, uint64_t timestamp) { }

        // TODO: Array of timestamps???
        // TODO: Point object to hold x, y, z, intensity, stamp??
        void add_lidar(std::vector<Eigen::Vector4d> points, uint64_t timestamp) {
            kiss_icp_->RegisterFrame(points);
        }

        void set_param(std::string key, std::string value) {
            throw std::invalid_argument("Invalid parameter, KissICP doesn't have string param " + key);
        }

        void set_param(std::string key, double value) {
            if (key == "voxel_size") {
                config_.voxel_size = value;
            } else if (key == "max_range") {
                config_.max_range = value;
            } else if (key == "min_range") {
                config_.min_range = value;
            } else if (key == "min_motion_th") {
                config_.min_motion_th = value;
            } else if (key == "initial_threshold") {
                config_.initial_threshold = value;
            } else if (key == "max_num_iterations") {
                config_.max_num_iterations = value;
            } else if (key == "convergence_criterion") {
                config_.convergence_criterion = value;
            } else if (key == "max_num_threads") {
                config_.max_num_threads = value;
            } else {
                throw std::invalid_argument("Invalid parameter, KissICP doesn't have double param " + key);
            }

            kiss_icp_ = std::make_unique<kiss_icp::pipeline::KissICP>(config_);
        }

        void set_param(std::string key, int value) {
            if (key == "max_points_per_voxel") {
                config_.max_points_per_voxel = value;
            } else {
                throw std::invalid_argument("Invalid parameter, KissICP doesn't have int param " + key);
            }

            kiss_icp_ = std::make_unique<kiss_icp::pipeline::KissICP>(config_);
        }

        void set_param(std::string key, bool value) {
            if (key == "deskew") {
                config_.deskew = value;
            } else {
                throw std::invalid_argument("Invalid parameter, KissICP doesn't have bool param " + key);
            }

            kiss_icp_ = std::make_unique<kiss_icp::pipeline::KissICP>(config_);
        }

        const gtsam::Pose3 pose() {
            const Sophus::SE3d pose = kiss_icp_->pose();
            const auto t = pose.translation();
            const auto q = pose.unit_quaternion();
            return gtsam::Pose3(gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z()), t);
        }

        const std::vector<Eigen::Vector4d> map() {
            return kiss_icp_->LocalMap();
        }


    private:
        std::unique_ptr<kiss_icp::pipeline::KissICP> kiss_icp_;
        kiss_icp::pipeline::KISSConfig config_;

};