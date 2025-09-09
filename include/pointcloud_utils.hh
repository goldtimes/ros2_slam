#pragma once
#include <omp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include "eigen_type.hh"
#include "lidar_point_type.hh"
#include "logger.hh"

namespace slam {
using PointType = pcl::PointXYZINormal;
using PointCloudType = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloudType::Ptr;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using PointTree = pcl::search::KdTree<PointType>;

using PointXYZI = pcl::PointXYZI;
using PointCloudXYZI = pcl::PointCloud<PointXYZI>;
using PointXYZITree = pcl::search::KdTree<PointXYZI>;

template <typename T>
inline V3D ToV3D(const T& point) {
    return V3D(point.x, point.y, point.z);
}

template <typename T>
inline T ToPoint(const V3D& pt) {
    T point;
    point.x = pt.x();
    point.y = pt.y();
    point.z = pt.z();
    return point;
}

PointCloudPtr TransformLidarOMP(const PointCloudPtr& cloud, const SE3& transform);
PointCloudPtr TransformLidarOMP(const PointCloudPtr& cloud, const M3D& R, const V3D& t);
PointCloudPtr TransformLidar(const PointCloudPtr& cloud, const M3D& R, const V3D& t);

template <typename T>
void TransformCloud(const T& cloud, T& out_cloud, const M3D& R, const V3D& t) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = R.cast<float>();
    transform.block<3, 1>(0, 3) = t.cast<float>();
    pcl::transformPointCloud(*cloud, *out_cloud, transform);
}
}  // namespace slam