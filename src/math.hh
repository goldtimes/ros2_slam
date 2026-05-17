#pragma once

#include "eigen_type.hh"
#include "lidar_point_type.hh"
#include "pointcloud_utils.hh"

namespace math {
template <typename PointT>
inline double distance2(const PointT &pt1, const PointT &pt2) {
  Eigen::Vector3f d = pt1.getVector3fMap() - pt2.getVector3fMap();
  return d.squaredNorm();
}

// convert from pcl point to eigen
template <typename T, int dim, typename PointType>
inline Eigen::Matrix<T, dim, 1> ToEigen(const PointType &pt) {
  return Eigen::Matrix<T, dim, 1>(pt.x, pt.y, pt.z);
}

template <>
inline Eigen::Matrix<float, 3, 1>
ToEigen<float, 3, pcl::PointXYZ>(const pcl::PointXYZ &pt) {
  return pt.getVector3fMap();
}

template <>
inline Eigen::Matrix<float, 3, 1>
ToEigen<float, 3, pcl::PointXYZI>(const pcl::PointXYZI &pt) {
  return pt.getVector3fMap();
}

template <>
inline Eigen::Matrix<float, 3, 1>
ToEigen<float, 3, pcl::PointXYZINormal>(const pcl::PointXYZINormal &pt) {
  return pt.getVector3fMap();
}
/**
 * estimate a plane
 * @tparam T
 * @param pca_result
 * @param point
 * @param threshold
 * @return
 */
template <typename T>
inline bool
esti_plane(Eigen::Matrix<T, 4, 1> &pca_result,
           const std::vector<slam::PointXYZI,
                             Eigen::aligned_allocator<slam::PointXYZI>> &point,
           const T &threshold = 0.1f) {
  if (point.size() < fasterlio::MIN_NUM_MATCH_POINTS) {
    return false;
  }

  Eigen::Matrix<T, 3, 1> normvec;

  if (point.size() == fasterlio::NUM_MATCH_POINTS) {
    Eigen::Matrix<T, fasterlio::NUM_MATCH_POINTS, 3> A;
    Eigen::Matrix<T, fasterlio::NUM_MATCH_POINTS, 1> b;

    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < fasterlio::NUM_MATCH_POINTS; j++) {
      A(j, 0) = point[j].x;
      A(j, 1) = point[j].y;
      A(j, 2) = point[j].z;
    }

    normvec = A.colPivHouseholderQr().solve(b);
  } else {
    Eigen::MatrixXd A(point.size(), 3);
    Eigen::VectorXd b(point.size(), 1);

    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < point.size(); j++) {
      A(j, 0) = point[j].x;
      A(j, 1) = point[j].y;
      A(j, 2) = point[j].z;
    }

    Eigen::MatrixXd n = A.colPivHouseholderQr().solve(b);
    normvec(0, 0) = n(0, 0);
    normvec(1, 0) = n(1, 0);
    normvec(2, 0) = n(2, 0);
  }

  T n = normvec.norm();
  pca_result(0) = normvec(0) / n;
  pca_result(1) = normvec(1) / n;
  pca_result(2) = normvec(2) / n;
  pca_result(3) = 1.0 / n;

  for (const auto &p : point) {
    Eigen::Matrix<T, 4, 1> temp = p.getVector4fMap();
    temp[3] = 1.0;
    if (fabs(pca_result.dot(temp)) > threshold) {
      return false;
    }
  }
  return true;
}

} // namespace math