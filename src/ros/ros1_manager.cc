#include "ros/ros1_manager.hh"
#include "system_config.hh"

namespace slam {

ROS1Manager::ROS1Manager(const ros::NodeHandle& nh, std::shared_ptr<SystemConfig> system_config_ptr)
    : nh_(nh), system_config_ptr_(system_config_ptr) {
    LOG_INFO("ROS1Manager init");
    InitPub();
    InitSub();
    InitService();
}
ROS1Manager::~ROS1Manager() {
}

void ROS1Manager::InitPub() {
}
void ROS1Manager::InitSub() {
    imu_sub_ = nh_.subscribe(system_config_ptr_->imu_config_.imu_topic, 100, &ROS1Manager::ImuCallback, this,
                             ros::TransportHints().tcpNoDelay());
    if (system_config_ptr_->lidar_config_.use_livox_driver == 0) {
        lidar_sub_ = nh_.subscribe(system_config_ptr_->lidar_config_.lidar_topic, 100,
                                   &ROS1Manager::StandarCloudCallback, this, ros::TransportHints().tcpNoDelay());
        LOG_INFO("use standard lidar driver");
    } else if (system_config_ptr_->lidar_config_.use_livox_driver == 1) {
        lidar_sub_ = nh_.subscribe(system_config_ptr_->lidar_config_.lidar_topic, 10, &ROS1Manager::LivoxCloudCallback,
                                   this, ros::TransportHints().tcpNoDelay());
        LOG_INFO("use livox driver 1");

    } else if (system_config_ptr_->lidar_config_.use_livox_driver == 2) {
        lidar_sub_ = nh_.subscribe(system_config_ptr_->lidar_config_.lidar_topic, 10, &ROS1Manager::Livox2CloudCallback,
                                   this, ros::TransportHints().tcpNoDelay());
        LOG_INFO("use livox driver 2");
    } else {
        LOG_ERROR("use_livox_driver must be 0, 1 or 2!");
        std::exit(1);
    }
    encoder_sub_ = nh_.subscribe(system_config_ptr_->encoder_config_.encoder_topic, 100, &ROS1Manager::EncoderCallback,
                                 this, ros::TransportHints().tcpNoDelay());
    gnss_sub_ = nh_.subscribe(system_config_ptr_->gnss_config_.gnss_topic, 100, &ROS1Manager::GNSSCallback, this,
                              ros::TransportHints().tcpNoDelay());
}
void ROS1Manager::InitService() {
}

void ROS1Manager::StandarCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
}
void ROS1Manager::Livox2CloudCallback(const livox_ros_driver2::CustomMsg::ConstPtr& cloud_livox) {
}
void ROS1Manager::LivoxCloudCallback(const livox_ros_driver::CustomMsg::ConstPtr& cloud_livox) {
}
void ROS1Manager::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
}
void ROS1Manager::EncoderCallback(const nav_msgs::Odometry::ConstPtr& encoder_msg) {
}
void ROS1Manager::GNSSCallback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msg) {
}
}  // namespace slam