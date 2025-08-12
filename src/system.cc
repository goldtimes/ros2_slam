#include "system.hh"
#include "system_config.hh"

namespace slam {
System::System(ros::NodeHandle nh) : nh_(nh) {
    InitConfigParams();
    InitPub();
    InitSub();
    InitService();
}

void System::InitConfigParams() {
    std::string config_path;
    nh_.param<std::string>("config_path", config_path, "");
    LOG_INFO("config_path: {}", config_path);
    system_config_ptr_ = std::make_shared<SystemConfig>();
    if (!system_config_ptr_->LoadAndPrintConfig(config_path)) {
        LOG_ERROR("Load config file failed, Program will exit!");
        std::exit(1);
    }
}

void System::InitPub() {
}
void System::InitSub() {
    imu_sub_ = nh_.subscribe(system_config_ptr_->imu_config_.imu_topic, 100, &System::ImuCallback, this,
                             ros::TransportHints().tcpNoDelay());
    if (system_config_ptr_->lidar_config_.use_livox_driver == 0) {
        lidar_sub_ = nh_.subscribe(system_config_ptr_->lidar_config_.lidar_topic, 100, &System::StandarCloudCallback,
                                   this, ros::TransportHints().tcpNoDelay());
        LOG_INFO("use standard lidar driver");
    } else if (system_config_ptr_->lidar_config_.use_livox_driver == 1) {
        lidar_sub_ = nh_.subscribe(system_config_ptr_->lidar_config_.lidar_topic, 10, &System::LivoxCloudCallback, this,
                                   ros::TransportHints().tcpNoDelay());
        LOG_INFO("use livox driver 1");

    } else if (system_config_ptr_->lidar_config_.use_livox_driver == 2) {
        lidar_sub_ = nh_.subscribe(system_config_ptr_->lidar_config_.lidar_topic, 10, &System::Livox2CloudCallback,
                                   this, ros::TransportHints().tcpNoDelay());
        LOG_INFO("use livox driver 2");
    } else {
        LOG_ERROR("use_livox_driver must be 0, 1 or 2!");
        std::exit(1);
    }
    encoder_sub_ = nh_.subscribe(system_config_ptr_->encoder_config_.encoder_topic, 100, &System::EncoderCallback, this,
                                 ros::TransportHints().tcpNoDelay());
    gnss_sub_ = nh_.subscribe(system_config_ptr_->gnss_config_.gnss_topic, 100, &System::GNSSCallback, this,
                              ros::TransportHints().tcpNoDelay());
}
void System::InitService() {
}

void System::StandarCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
}
void System::Livox2CloudCallback(const livox_ros_driver2::CustomMsg::ConstPtr& cloud_livox) {
}
void System::LivoxCloudCallback(const livox_ros_driver::CustomMsg::ConstPtr& cloud_livox) {
}
void System::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
}
void System::EncoderCallback(const nav_msgs::Odometry::ConstPtr& encoder_msg) {
}
void System::GNSSCallback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msg) {
}

System::~System() {
}
}  // namespace slam