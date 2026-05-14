
#pragma once
#include <omp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include "common/eigen_type.hh"
#include "common/lidar_point_type.hh"
#include "utils/logger.hh"

namespace slam {

using PointType = slam::PointXYZIRT;
using PointCloudType = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloudType::Ptr;

using PointXYZI = pcl::PointXYZI;
using PointCloudXYZI = pcl::PointCloud<PointXYZI>;
using PointVec = std::vector<PointXYZI, Eigen::aligned_allocator<PointXYZI>>;
using PointCloudXYZIPtr = pcl::PointCloud<PointXYZI>::Ptr;
using PointXYZITree = pcl::search::KdTree<PointXYZI>;
using PointTree = pcl::search::KdTree<PointXYZI>;
using PointFilter = pcl::VoxelGrid<PointXYZI>;

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

template <typename T>
inline T dis_a_b(const T& xa, const T& ya, const T& xb, const T& yb) {
    return std::sqrt(std::pow(xa - xb, 2) + std::pow(ya - yb, 2));
}

PointCloudXYZIPtr TransformLidarOMP(const PointCloudXYZIPtr& cloud, const SE3& transform);
PointCloudXYZIPtr TransformLidarOMP(const PointCloudXYZIPtr& cloud, const M3D& R, const V3D& t);
PointCloudXYZIPtr TransformLidar(const PointCloudXYZIPtr& cloud, const M3D& R, const V3D& t);

template <typename T>
void TransformCloud(const T& cloud, T& out_cloud, const M3D& R, const V3D& t) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = R.cast<float>();
    transform.block<3, 1>(0, 3) = t.cast<float>();
    pcl::transformPointCloud(*cloud, *out_cloud, transform);
}
}  // namespace slam