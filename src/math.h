#pragma once

#include "eigen_types.hh"
#include "lidar_point_type.hh"
namespace math {
template <typename PointT>
inline double distance2(const PointT& pt1, const PointT& pt2) {
    Eigen::Vector3f d = pt1.getVector3fMap() - pt2.getVector3fMap();
    return d.squaredNorm();
}

// convert from pcl point to eigen
template <typename T, int dim, typename PointType>
inline Eigen::Matrix<T, dim, 1> ToEigen(const PointType& pt) {
    return Eigen::Matrix<T, dim, 1>(pt.x, pt.y, pt.z);
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, pcl::PointXYZ>(const pcl::PointXYZ& pt) {
    return pt.getVector3fMap();
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, pcl::PointXYZI>(const pcl::PointXYZI& pt) {
    return pt.getVector3fMap();
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, pcl::PointXYZINormal>(const pcl::PointXYZINormal& pt) {
    return pt.getVector3fMap();
}

}  // namespace math