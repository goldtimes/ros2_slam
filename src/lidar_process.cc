#include "lidar_process.hh"

namespace slam {
LidarProcess::LidarProcess(const std::string& lidar_type, int use_livox_driver, double min_range, double max_range,
                           int point_filter_num)
    : lidar_type_(lidar_type),
      use_livox_driver_(use_livox_driver),
      min_range_(min_range),
      max_range_(max_range),
      point_filter_num_(point_filter_num) {
    if (lidar_type_ == "mid360") {
        lidar_mode_ = LIDAR_MODE::MID360;
    } else if (lidar_type_ == "avia") {
        lidar_mode_ = LIDAR_MODE::AVIA;
    } else if (lidar_type_ == "lslidar") {
        lidar_mode_ = LIDAR_MODE::LSLIDAR;
    } else if (lidar_type_ == "rs16") {
        lidar_mode_ = LIDAR_MODE::RS16;
    } else if (lidar_type_ == "airy") {
        lidar_mode_ = LIDAR_MODE::AIRY;
    } else if (lidar_type_ == "vanjee") {
        lidar_mode_ = LIDAR_MODE::VANJEE;
    } else if (lidar_type_ == "velodyne32") {
        lidar_mode_ = LIDAR_MODE::VELODYNE32;
    } else if (lidar_type_ == "ouster64") {
        lidar_mode_ = LIDAR_MODE::OUSTER64;
    } else {
        lidar_mode_ = LIDAR_MODE::MID360;
        LOG_WARN("lidar type {} not support, use mid360 instead", lidar_type_);
    }
}

// 处理ros标准的雷达消息
bool LidarProcess::Process(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, PointCloudPtr& out_cloud) {
    switch (lidar_mode_) {
        case LIDAR_MODE::MID360:
            break;
        case LIDAR_MODE::LSLIDAR:
            break;
        case LIDAR_MODE::RS16:
            break;
        case LIDAR_MODE::AIRY:
            break;
        case LIDAR_MODE::VANJEE:
            break;
        case LIDAR_MODE::VELODYNE32:
            break;
        case LIDAR_MODE::OUSTER64:
            break;
        default:
            break;
    }
    return true;
}

// 处理livox driver1雷达消息
bool LidarProcess::Process(const livox_ros_driver::CustomMsg::ConstPtr& cloud_msg, PointCloudPtr& out_cloud) {
    return true;
}

// 处理livox driver2雷达消息
bool LidarProcess::Process(const livox_ros_driver2::CustomMsg::ConstPtr& cloud_msg, PointCloudPtr& out_cloud) {
    return true;
}

}  // namespace slam