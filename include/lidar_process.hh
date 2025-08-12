#pragma once
// 告诉编译器，当前头文件在一次编译过程中只被包含一次
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include "logger.hh"
#include "pointcloud_utils.hh"

namespace slam {
enum class LIDAR_MODE {
    MID360,      // mid360
    AVIA,        // avia
    LSLIDAR,     // lslidar
    RS16,        // rs16
    AIRY,        // airylidar
    VANJEE,      // vanjee
    VELODYNE32,  // velodyne32
    OUSTER64,    // ouster64
};

class LidarProcess {
   public:
    LidarProcess(const std::string& lidar_type, int use_livox_driver, double min_range, double max_range,
                 int point_filter_num);
    ~LidarProcess() = default;

    bool Process(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool Process(const livox_ros_driver::CustomMsg::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool Process(const livox_ros_driver2::CustomMsg::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);

   private:
    LIDAR_MODE lidar_mode_;
    std::string lidar_type_;
    int use_livox_driver_;
    double min_range_;
    double max_range_;
    int point_filter_num_;
};
}  // namespace slam