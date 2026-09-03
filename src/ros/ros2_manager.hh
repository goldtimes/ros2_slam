#pragma once
// ============================================================
//  ROS2 版 SLAM 管理器(与 src/ros/ros1_manager.* 对应)
//  仅用于 ROS2(rclcpp),通过顶层 CMakeLists 的 cmake/ROS2.cmake 编译
// ============================================================
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include "common/commons.hh"
#include "common/logger.hh"
#include "gnss/gnss_process.hh"
#include "ros/ros_common.hh"

namespace slam {

class SystemConfig;
class System;

class ROS2Manager {
   public:
    ROS2Manager(const rclcpp::Node::SharedPtr &node, std::shared_ptr<System> system_ptr);
    ~ROS2Manager();

    void InitPub();
    void InitSub();

    // 标准雷达消息回调
    void StandarCloudCallback(const PointCloud2Msg::SharedPtr cloud_msg);
    // livox driver2 回调
    void Livox2CloudCallback(const LivoxMsg2::SharedPtr cloud_livox);
    // imu回调
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    // 编码器回调
    void EncoderCallback(const nav_msgs::msg::Odometry::SharedPtr encoder_msg);
    // gnss回调
    void GNSSCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gnss_msg);
    // rviz /initialpose 回调
    void RosInitPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg);

    // 定时器触发的可视化(约10Hz)
    void Visualize();
    void PublishTF(const double &sensor_time);
    void PublishState(const double &sensor_time);
    void PublishLidar(const double &sensor_time);

   private:
    geometry_msgs::msg::TransformStamped GetTransformStamped(const double timestamp,
                                                             const PoseTrans &transform = PoseTrans(),
                                                             bool flip_trans = false);

    sensor_msgs::msg::PointCloud2 ToPointCloud2(const PointCloudXYZIPtr &cloud, const std::string &frame_id,
                                                double timestamp = -1);

    void PoseTransToPoseStampedMsg(const PoseTrans &pose_trans, geometry_msgs::msg::PoseStamped &pose_msg);
    void PoseTransToOdomMsg(const PoseTrans &pose_trans, nav_msgs::msg::Odometry &odom_msg);
    void RosPoseToPoseTrans(const geometry_msgs::msg::Pose &pose_msg, PoseTrans &pose_trans);

    void PublishPath(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub, nav_msgs::msg::Path &path,
                     const std::string &frame_id, double sensor_time, const PoseTrans &pose_trans);

    void voxelTimerCB();

   private:
    rclcpp::Node::SharedPtr nh_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<LivoxMsg2>::SharedPtr lidar_livox_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr encoder_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ros_init_pose_sub_;

    std::shared_ptr<System> system_ptr_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_lidar_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_robot_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_body_pub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gnss_odom_pub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lio_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr encoder_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr lio_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gnss_path_pub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_pub_;

    nav_msgs::msg::Path lio_path_;
    nav_msgs::msg::Path encoder_path_;
    nav_msgs::msg::Path gnss_path_;

    rclcpp::TimerBase::SharedPtr voxel_map_timer_;
    rclcpp::TimerBase::SharedPtr visualize_timer_;

    bool has_encoder_ = false;
    bool has_gnss_ = false;
    SLAM_MODE slam_mode_ = SLAM_MODE::ODOMETER;

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
    std::shared_ptr<GnssProcess> gnss_process_;
    bool gnss_init_ = false;
};
}  // namespace slam
