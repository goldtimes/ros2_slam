#include <geometry_msgs/TransformStamped.h>
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include "commons.hh"
#include "logger.hh"

namespace slam {

class SystemConfig;
class System;

class ROS1Manager {
   public:
    ROS1Manager(const ros::NodeHandle& nh, std::shared_ptr<System> system_ptr);
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

    void Visualize();

    void PublishTF(const double& sensor_time);

    void PublishState(const double& sensor_time);

    void PublishLidar(const double& sensor_time);

   private:
    geometry_msgs::TransformStamped GetTransformStamped(const double timestamp,
                                                        const PoseTrans& transform = PoseTrans(),
                                                        bool flip_trans = false);

    sensor_msgs::PointCloud2 ToPointCloud2(const PointCloudPtr& cloud, const std::string& frame_id,
                                           double timestamp = -1);
    void voxelTimerCB(const ros::TimerEvent& event);

   private:
    ros::NodeHandle nh_;

    ros::Subscriber imu_sub_;
    ros::Subscriber gnss_sub_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber encoder_sub_;
    std::shared_ptr<System> system_ptr_;

    ros::Publisher cloud_lidar_pub_;
    ros::Publisher cloud_robot_pub_;
    ros::Publisher cloud_odom_pub_;

    ros::Timer voxel_map_timer_;

    // tf2
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // fps的统计
    double last_imu_time_ = -1;
    int imu_frame_count_ = 0;
    int imu_fps_ = 0;
    double last_encoder_time_ = -1;
    int encoder_frame_count_ = 0;
    int encoder_fps_ = 0;
    double last_lidar_time_ = -1;
    int lidar_frame_count_ = 0;
    int lidar_fps_ = 0;
    double last_gnss_time_ = -1;
    int gnss_frame_count_ = 0;
    int gnss_fps_ = 0;

    double last_visualize_time_ = -1;
    std::thread visualize_thread_;
};
}  // namespace slam