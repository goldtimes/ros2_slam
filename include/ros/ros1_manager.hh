#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include "logger.hh"

namespace slam {

class SystemConfig;

class ROS1Manager {
   public:
    ROS1Manager(const ros::NodeHandle& nh, std::shared_ptr<SystemConfig> system_config_ptr);
    ~ROS1Manager();

    void InitPub();
    void InitSub();
    void InitService();
    // 标准雷达消息回调
    void StandarCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    // livox驱动2回调
    void Livox2CloudCallback(const livox_ros_driver2::CustomMsg::ConstPtr& cloud_livox);
    // livox驱动1回调
    void LivoxCloudCallback(const livox_ros_driver::CustomMsg::ConstPtr& cloud_livox);
    // imu回调
    void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    // 编码器回调
    void EncoderCallback(const nav_msgs::Odometry::ConstPtr& encoder_msg);
    // gnss回调
    void GNSSCallback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msg);

   private:
    ros::NodeHandle nh_;

    ros::Subscriber imu_sub_;
    ros::Subscriber gnss_sub_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber encoder_sub_;
    std::shared_ptr<SystemConfig> system_config_ptr_;
};
}  // namespace slam