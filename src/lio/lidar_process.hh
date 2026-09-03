#pragma once
// 告诉编译器，当前头文件在一次编译过程中只被包含一次
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include "common/logger.hh"
#include "ros/ros_common.hh"
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

    // 标准点云消息处理
    bool Process(const PointCloud2MsgConstPtr& cloud_msg, PointCloudPtr& out_cloud);
#if ROS_AVAILABLE == 1
    // livox driver1(仅 ROS1)
    bool Process(const LivoxMsg1ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
#endif
    // livox driver2
    bool Process(const LivoxMsg2ConstPtr& cloud_msg, PointCloudPtr& out_cloud);

    bool mid360_process(const PointCloud2MsgConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool avia_process(const PointCloud2MsgConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool ls16_process(const PointCloud2MsgConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool rs16_process(const PointCloud2MsgConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool airy_process(const PointCloud2MsgConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool vanjee_process(const PointCloud2MsgConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool velodyne16_process(const PointCloud2MsgConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool velodyne32_process(const PointCloud2MsgConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    bool ouster64_process(const PointCloud2MsgConstPtr& cloud_msg, PointCloudPtr& out_cloud);
#if ROS_AVAILABLE == 1
    // mid360的livox driver1消息处理(仅 ROS1)
    bool mid360_process(const LivoxMsg1ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    // avia的livox driver1消息处理(仅 ROS1)
    bool avia_process(const LivoxMsg1ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
#endif
    // mid360的livox driver2消息处理
    bool mid360_process(const LivoxMsg2ConstPtr& cloud_msg, PointCloudPtr& out_cloud);
    // avia的livox driver2消息处理
    bool avia_process(const LivoxMsg2ConstPtr& cloud_msg, PointCloudPtr& out_cloud);

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