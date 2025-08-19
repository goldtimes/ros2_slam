#pragma once
#include <omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include "eigen_type.hh"
#include "lidar_point_type.hh"
#include "logger.hh"

namespace slam {
using PointType = PointXYZIRT;
using PointCloudType = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloudType::Ptr;

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

}  // namespace slam