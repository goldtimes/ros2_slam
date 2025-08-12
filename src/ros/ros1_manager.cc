#include "ros/ros1_manager.hh"
#include "system.hh"
#include "system_config.hh"
namespace slam {

ROS1Manager::ROS1Manager(const ros::NodeHandle& nh, std::shared_ptr<System> system_ptr)
    : nh_(nh), system_ptr_(system_ptr) {
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
    imu_sub_ = nh_.subscribe(system_ptr_->GetSystemConfig()->imu_config_.imu_topic, 100, &ROS1Manager::ImuCallback,
                             this, ros::TransportHints().tcpNoDelay());
    if (system_ptr_->GetSystemConfig()->lidar_config_.use_livox_driver == 0) {
        lidar_sub_ = nh_.subscribe(system_ptr_->GetSystemConfig()->lidar_config_.lidar_topic, 100,
                                   &ROS1Manager::StandarCloudCallback, this, ros::TransportHints().tcpNoDelay());
        LOG_INFO("use standard lidar driver");
    } else if (system_ptr_->GetSystemConfig()->lidar_config_.use_livox_driver == 1) {
        lidar_sub_ = nh_.subscribe(system_ptr_->GetSystemConfig()->lidar_config_.lidar_topic, 10,
                                   &ROS1Manager::LivoxCloudCallback, this, ros::TransportHints().tcpNoDelay());
        LOG_INFO("use livox driver 1");

    } else if (system_ptr_->GetSystemConfig()->lidar_config_.use_livox_driver == 2) {
        lidar_sub_ = nh_.subscribe(system_ptr_->GetSystemConfig()->lidar_config_.lidar_topic, 10,
                                   &ROS1Manager::Livox2CloudCallback, this, ros::TransportHints().tcpNoDelay());
        LOG_INFO("use livox driver 2");
    } else {
        LOG_ERROR("use_livox_driver must be 0, 1 or 2!");
        std::exit(1);
    }
    if (system_ptr_->GetSystemConfig()->has_encoder_) {
        encoder_sub_ = nh_.subscribe(system_ptr_->GetSystemConfig()->encoder_config_.encoder_topic, 100,
                                     &ROS1Manager::EncoderCallback, this, ros::TransportHints().tcpNoDelay());
    }
    if (system_ptr_->GetSystemConfig()->has_gnss_) {
        gnss_sub_ = nh_.subscribe(system_ptr_->GetSystemConfig()->gnss_config_.gnss_topic, 100,
                                  &ROS1Manager::GNSSCallback, this, ros::TransportHints().tcpNoDelay());
    }
}
void ROS1Manager::InitService() {
}

void ROS1Manager::StandarCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    static double last_record_lidar_time = cloud_msg->header.stamp.toSec();
    double curr_lidar_time = cloud_msg->header.stamp.toSec();
    // 统计fps
    if (curr_lidar_time - last_record_lidar_time >= 5.0) {
        lidar_fps_ = static_cast<int>(lidar_frame_count_ / 5);
        LOG_INFO(GREEN "receive lidar fps:{}" RESET, lidar_fps_);
        lidar_frame_count_ = 0;
        last_record_lidar_time = curr_lidar_time;
    }
    // 检测是否有数据丢失
    if (last_lidar_time_ != -1 && curr_lidar_time - last_lidar_time_ > 0.2) {
        LOG_ERROR("lidar data lost");
        LOG_ERROR("current_lidar_time:{}, last_lidar_time:{}", curr_lidar_time, last_lidar_time_);
    }
    last_lidar_time_ = curr_lidar_time;
    lidar_frame_count_++;
    // 需要在这里处理lidar数据
}
void ROS1Manager::Livox2CloudCallback(const livox_ros_driver2::CustomMsg::ConstPtr& cloud_livox) {
    static double last_record_lidar_time = cloud_livox->header.stamp.toSec();
    double curr_lidar_time = cloud_livox->header.stamp.toSec();
    if (curr_lidar_time - last_record_lidar_time >= 5.0) {
        lidar_fps_ = static_cast<int>(lidar_frame_count_ / 5);
        LOG_INFO(GREEN "receive lidar fps:{}" RESET, lidar_fps_);
        lidar_frame_count_ = 0;
        last_record_lidar_time = curr_lidar_time;
    }
    if (last_lidar_time_ != -1 && curr_lidar_time - last_lidar_time_ > 0.2) {
        LOG_ERROR("lidar data lost");
        LOG_ERROR("current_lidar_time:{}, last_lidar_time:{}", curr_lidar_time, last_lidar_time_);
    }
    last_lidar_time_ = curr_lidar_time;
    lidar_frame_count_++;
}
void ROS1Manager::LivoxCloudCallback(const livox_ros_driver::CustomMsg::ConstPtr& cloud_livox) {
    static double last_record_lidar_time = cloud_livox->header.stamp.toSec();
    double curr_lidar_time = cloud_livox->header.stamp.toSec();
    if (curr_lidar_time - last_record_lidar_time >= 5.0) {
        lidar_fps_ = static_cast<int>(lidar_frame_count_ / 5);
        LOG_INFO(GREEN "receive lidar fps:{}" RESET, lidar_fps_);
        lidar_frame_count_ = 0;
        last_record_lidar_time = curr_lidar_time;
    }
    if (last_lidar_time_ != -1 && curr_lidar_time - last_lidar_time_ > 0.2) {
        LOG_ERROR("lidar data lost");
        LOG_ERROR("current_lidar_time:{}, last_lidar_time:{}", curr_lidar_time, last_lidar_time_);
    }
    last_lidar_time_ = curr_lidar_time;
    lidar_frame_count_++;
}
void ROS1Manager::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    static double last_record_imu_time = imu_msg->header.stamp.toSec();
    double curr_imu_time = imu_msg->header.stamp.toSec();
    if (curr_imu_time - last_record_imu_time >= 5.0) {
        imu_fps_ = static_cast<int>(imu_frame_count_ / 5);
        LOG_INFO(GREEN "receive imu fps:{}" RESET, imu_fps_);
        imu_frame_count_ = 0;
        last_record_imu_time = curr_imu_time;
    }
    if (last_imu_time_ != -1 && curr_imu_time - last_imu_time_ > 0.2) {
        LOG_ERROR("imu data lost");
        LOG_ERROR("current_imu_time:{}, last_imu_time:{}", curr_imu_time, last_imu_time_);
    }
    last_imu_time_ = curr_imu_time;
    imu_frame_count_++;
    V3D acc, gyro;
    acc << imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z;
    gyro << imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;
    IMU imu(curr_imu_time, acc, gyro);
    // push to system
}
void ROS1Manager::EncoderCallback(const nav_msgs::Odometry::ConstPtr& encoder_msg) {
    static double last_record_encoder_time = encoder_msg->header.stamp.toSec();
    double curr_encoder_time = encoder_msg->header.stamp.toSec();
    if (curr_encoder_time - last_record_encoder_time >= 5.0) {
        encoder_fps_ = static_cast<int>(encoder_frame_count_ / 5);
        LOG_INFO(GREEN "receive encoder fps:{}" RESET, encoder_fps_);
        encoder_frame_count_ = 0;
        last_record_encoder_time = curr_encoder_time;
    }
    if (last_encoder_time_ != -1 && curr_encoder_time - last_encoder_time_ > 0.2) {
        LOG_ERROR("encoder data lost");
        LOG_ERROR("current_encoder_time:{}, last_encoder_time:{}", curr_encoder_time, last_encoder_time_);
    }
    last_encoder_time_ = curr_encoder_time;
    encoder_frame_count_++;
    V3D linear_vel, angular_vel;
    linear_vel << encoder_msg->twist.twist.linear.x, encoder_msg->twist.twist.linear.y,
        encoder_msg->twist.twist.linear.z;
    angular_vel << encoder_msg->twist.twist.angular.x, encoder_msg->twist.twist.angular.y,
        encoder_msg->twist.twist.angular.z;
    Encoder encoder(curr_encoder_time, linear_vel, angular_vel);
    // push to system
}
void ROS1Manager::GNSSCallback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msg) {
    static double last_record_gnss_time = gnss_msg->header.stamp.toSec();
    double curr_gnss_time = gnss_msg->header.stamp.toSec();
    if (curr_gnss_time - last_record_gnss_time >= 5.0) {
        gnss_fps_ = static_cast<int>(gnss_frame_count_ / 5);
        LOG_INFO(GREEN "receive gnss fps:{}" RESET, gnss_fps_);
        gnss_frame_count_ = 0;
        last_record_gnss_time = curr_gnss_time;
    }
    if (last_gnss_time_ != -1 && curr_gnss_time - last_gnss_time_ > 0.2) {
        LOG_ERROR("gnss data lost");
        LOG_ERROR("current_gnss_time:{}, last_gnss_time:{}", curr_gnss_time, last_gnss_time_);
    }
    last_gnss_time_ = curr_gnss_time;
    gnss_frame_count_++;
}
}  // namespace slam