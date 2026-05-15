/*
 * @Author: lihang lihang@kilox.cn
 * @Date: 2025-09-29 16:47:24
 * @LastEditors: lihang lihang@kilox.cn
 * @LastEditTime: 2025-09-29 17:52:28
 * @FilePath: /fast_lvio_ws/src/lio_slam/test/dynamic.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using PointType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointType>;

std::vector<PointCloud::ConstPtr> submaps;
pcl::search::KdTree<PointType> submap_tree;
bool build_submap = false;

ros::Publisher dynamic_cloud_pub_;
ros::Publisher merged_cloud_pub_;
PointCloud::Ptr merged_cloud;
std::string frame_id = "map";

PointCloud::Ptr extract_dynamic_points(const PointCloud::ConstPtr& cloud, const std::vector<float>& residuals,
                                       float thresh) {
    PointCloud::Ptr dynamic_cloud(new PointCloud);
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (residuals[i] > thresh) {  // 残差超过阈值，判定为动态点
            dynamic_cloud->push_back(cloud->points[i]);
        }
    }
    std::cout << "动态点数量: " << dynamic_cloud->size() << std::endl;
    return dynamic_cloud;
}

std::vector<float> compute_residuals(const PointCloud::Ptr& target_cloud,
                                     const pcl::search::KdTree<PointType>& target_tree, const PointCloud::Ptr& source) {
    std::vector<float> residuals;
    residuals.reserve(target_cloud->size());

    // 对目标点云中的每个点，搜索最近邻
    for (const auto& point : source->points) {
        std::vector<int> idx(1);     // 最近邻索引
        std::vector<float> dist(1);  // 最近邻距离
        if (target_tree.nearestKSearch(point, 1, idx, dist) > 0) {
            residuals.push_back(dist[0]);  // 存储距离（残差）
            // std::cout << "最近邻距离: " << dist[0] << std::endl;
        } else {
            residuals.push_back(1e6);  // 无匹配点，残差设为极大值
        }
    }

    return residuals;
}

sensor_msgs::PointCloud2 ToPointCloud2(const PointCloud::Ptr& cloud, const std::string& frame_id, double timestamp) {
    sensor_msgs::PointCloud2 cloud_msg;
    if (!cloud->empty()) {
        pcl::toROSMsg(*cloud, cloud_msg);
    }
    cloud_msg.header.frame_id = frame_id;
    if (timestamp <= 0) {
        cloud_msg.header.stamp = ros::Time::now();
    } else {
        cloud_msg.header.stamp = ros::Time(timestamp);
    }
    return cloud_msg;
}

void LidarCallback(const sensor_msgs::PointCloud2::Ptr& cloud_ptr) {
    if (submaps.size() <= 10) {
        // 转换为PCL点云
        PointCloud::Ptr cloud(new PointCloud);
        pcl::fromROSMsg(*cloud_ptr, *cloud);
        submaps.push_back(cloud);
        return;
    }
    if (!build_submap) {
        for (const auto& cloud : submaps) {
            *merged_cloud += *cloud;
        }
        // 发布合并后的点云
        auto merged_cloud_msg = ToPointCloud2(merged_cloud, frame_id, ros::Time::now().toSec());

        // 降采样
        pcl::VoxelGrid<PointType> voxel_grid;
        voxel_grid.setInputCloud(merged_cloud);
        voxel_grid.setLeafSize(0.3f, 0.3f, 0.3f);
        voxel_grid.filter(*merged_cloud);
        merged_cloud_pub_.publish(merged_cloud_msg);

        submap_tree.setInputCloud(merged_cloud);
        build_submap = true;
    }
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*cloud_ptr, *cloud);
    // 配准并识别动态物体
    auto residuals = compute_residuals(merged_cloud, submap_tree, cloud);
    auto dynamic_cloud = extract_dynamic_points(cloud, residuals, 0.3);
    if (dynamic_cloud->empty()) {
        return;
    }
    // to ros cloud msg
    auto cloud_msg = ToPointCloud2(dynamic_cloud, frame_id, ros::Time::now().toSec());
    dynamic_cloud_pub_.publish(cloud_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_test");
    ros::NodeHandle nh;
    merged_cloud.reset(new PointCloud);

    auto lidar_sub = nh.subscribe("/livox/lidar", 10, LidarCallback);
    dynamic_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("dynamic_cloud", 10, true);
    merged_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("merged_cloud", 10, true);
    ros::spin();

    return 0;
}
