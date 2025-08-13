#include "system.hh"
#include "system_config.hh"

namespace slam {
System::System(const std::string& config_path) : config_path_(config_path) {
    InitConfigParams();
    lidar_process_ptr_ = std::make_shared<LidarProcess>(
        system_config_ptr_->lidar_config_.lidar_type, system_config_ptr_->lidar_config_.use_livox_driver,
        system_config_ptr_->lidar_config_.lidar_min_range, system_config_ptr_->lidar_config_.lidar_max_range,
        system_config_ptr_->lidar_config_.point_filter_num, system_config_ptr_->frontend_config_.keep_angle_ranges,
        system_config_ptr_->frontend_config_.remove_ranges);
}

void System::InitConfigParams() {
    LOG_INFO("config_path: {}", config_path_);
    system_config_ptr_ = std::make_shared<SystemConfig>();
    if (!system_config_ptr_->LoadAndPrintConfig(config_path_)) {
        LOG_ERROR("Load config file failed, Program will exit!");
        std::exit(1);
    }
    LOG_INFO("System init done!");
}

void System::AddIMU(const IMU& imu) {
    std::lock_guard<std::mutex> lock(m_buf_mutex_);
    imu_queue_.push_back(imu);
}
void System::AddEncoder(const Encoder& encoder) {
    std::lock_guard<std::mutex> lock(m_buf_mutex_);
    encoder_queue_.push_back(encoder);
}
void System::AddLidar(const PointCloudPtr& cloud, const double cloud_time) {
    std::lock_guard<std::mutex> lock(m_buf_mutex_);
    lidar_queue_.push_back(cloud);
    lidar_time_queue_.push_back(cloud_time);
}
void System::AddGNSS(const GNSS& gnss) {
    std::lock_guard<std::mutex> lock(m_buf_mutex_);
    gnss_queue_.push_back(gnss);
}

System::~System() {
}
}  // namespace slam