#pragma once
// 告诉编译器，当前头文件在一次编译过程中只被包含一次
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include "common/logger.hh"
#include "utils/pointcloud_utils.hh"

namespace slam {
enum class LIDAR_MODE {
    MID360,      // mid360
    AVIA,        // avia
    LSLIDAR,     // lslidar
    RS16,        // rs16
    AIRY,        // airylidar
    VANJEE,      // vanjee
    VELODYNE16,  // velodyne16
    VELODYNE32,  // velodyne32
    OUSTER64,    // ouster64
};

class LidarProcess {
   public:
    LidarProcess(const std::string& lidar_type, int use_livox_driver, double min_range, double max_range,
                 int point_filter_num, std::vector<double> keep_angle_ranges = {-180.0, 180},
                 std::vector<double> remove_ranges = {0.0, 0.0, 0.0, 0.0});
    ~LidarProcess() = default;

    bool Process(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool Process(const livox_ros_driver::CustomMsg::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool Process(const livox_ros_driver2::CustomMsg::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);

    bool mid360_process(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool avia_process(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool ls16_process(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool rs16_process(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool airy_process(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool vanjee_process(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool velodyne16_process(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool velodyne32_process(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool ouster64_process(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    // mid360的livox消息处理
    bool mid360_process(const livox_ros_driver::CustomMsg::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool mid360_process(const livox_ros_driver2::CustomMsg::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    // avia的livox消息处理
    bool avia_process(const livox_ros_driver::CustomMsg::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool avia_process(const livox_ros_driver2::CustomMsg::ConstPtr& cloud_msg, PointCloudPtr& out_cloud);

   private:
    double normalizedAngle(double angle_deg) {
        while (angle_deg > 180.0) {
            angle_deg -= 360.0;
        }
        while (angle_deg < -180) {
            angle_deg += 360.0;
        }
        return angle_deg;
    }

   private:
    LIDAR_MODE lidar_mode_;
    std::string lidar_type_;
    int use_livox_driver_;
    double min_range_;
    double max_range_;
    int point_filter_num_;
    std::vector<double> keep_angle_ranges_;
    std::vector<double> remove_ranges_;
    std::vector<std::pair<double, double>> keep_angles;
    double remove_lidar_front_;
    double remove_lidar_back_;
    double remove_lidar_left_;
    double remove_lidar_right_;
};
}  // namespace slam