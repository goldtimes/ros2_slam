#pragma once
// ============================================================
//  lio_slam ROS 版本公共头文件
//  -------------------------------------------
//  提供 ROS1 / ROS2 双版本下的:
//    1) 编译期版本判定 ROS_AVAILABLE(1=ROS1, 2=ROS2)
//    2) 消息类型别名(PointCloud2 / livox 等)
//    3) 时间戳 / 频率控制等跨版本工具
//
//  使用方式:编译时通过 -DROS_AVAILABLE=1(ROS1) 或 =2(ROS2) 指定。
// ============================================================

#ifndef ROS_AVAILABLE
#error "ROS_AVAILABLE must be defined to 1 (ROS1) or 2 (ROS2) via -DROS_AVAILABLE=..."
#endif

#include <chrono>
#include <cstdint>
#include <thread>

#if ROS_AVAILABLE == 1
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#elif ROS_AVAILABLE == 2
#include <builtin_interfaces/msg/time.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#else
#error "ROS_AVAILABLE must be 1 (ROS1) or 2 (ROS2) !"
#endif

namespace slam {

// ==================== 消息类型别名 ====================
#if ROS_AVAILABLE == 1
// 标准点云消息(ROS1)
using PointCloud2Msg = sensor_msgs::PointCloud2;
using PointCloud2MsgConstPtr = sensor_msgs::PointCloud2::ConstPtr;
// livox driver1(仅 ROS1 存在)
using LivoxMsg1 = livox_ros_driver::CustomMsg;
using LivoxMsg1ConstPtr = livox_ros_driver::CustomMsg::ConstPtr;
// livox driver2(ROS1 包名无 ::msg,ROS2 包名有 ::msg)
using LivoxMsg2 = livox_ros_driver2::CustomMsg;
using LivoxMsg2ConstPtr = livox_ros_driver2::CustomMsg::ConstPtr;
#elif ROS_AVAILABLE == 2
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using PointCloud2MsgConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
// livox driver2(ROS2)
using LivoxMsg2 = livox_ros_driver2::msg::CustomMsg;
using LivoxMsg2ConstPtr = livox_ros_driver2::msg::CustomMsg::ConstSharedPtr;
#endif

// ==================== 时间工具 ====================
// ROS1: ros::Time, ROS2: builtin_interfaces::msg::Time
#if ROS_AVAILABLE == 1
inline double StampToSec(const ros::Time &t) {
    return t.toSec();
}
#elif ROS_AVAILABLE == 2
inline double StampToSec(const builtin_interfaces::msg::Time &t) {
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}
#endif

// ==================== 运行标志 / 频率控制(算法线程使用) ====================
inline bool RosOk() {
#if ROS_AVAILABLE == 1
    return ros::ok();
#elif ROS_AVAILABLE == 2
    return rclcpp::ok();
#else
    return true;
#endif
}

// 行为近似 ros::Rate / rclcpp::Rate 的跨版本频率控制
class Rate {
   public:
    explicit Rate(double frequency) : period_ns_(static_cast<int64_t>(1e9 / frequency)) {
        last_ns_ = NowNs();
    }

    void sleep() {
        auto now_ns = NowNs();
        auto next_ns = last_ns_ + period_ns_;
        if (next_ns > now_ns) {
            std::this_thread::sleep_for(std::chrono::nanoseconds(next_ns - now_ns));
        }
        // 落后超过一个周期时重新对齐,避免连续空转追赶
        last_ns_ = (next_ns > now_ns) ? next_ns : now_ns;
    }

   private:
    static int64_t NowNs() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }
    int64_t period_ns_;
    int64_t last_ns_;
};

}  // namespace slam
