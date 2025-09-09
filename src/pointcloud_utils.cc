/*
 * @Author: lihang lihang@kilox.cn
 * @Date: 2025-08-29 14:15:00
 * @LastEditors: lihang lihang@kilox.cn
 * @LastEditTime: 2025-09-09 13:49:32
 * @FilePath: /fast_lvio_ws/src/open_slam/src/pointcloud_utils.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "pointcloud_utils.hh"

namespace slam {

PointCloudPtr TransformLidarOMP(const PointCloudPtr& cloud, const SE3& transform) {
    PointCloudPtr transformed_cloud(new PointCloudType);
    transformed_cloud->resize(cloud->size());
#ifdef MP_EN
    omp_set_num_threads(4);
#pragma omp parallel for
#endif
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto pt_eigen = ToV3D(cloud->points[i]);
        const auto pt_transforemd = transform * pt_eigen;
        //  这样计算会有问题
        // const auto pt_transforemd = transform.so3().matrix() * pt_eigen + transform.translation();
        PointType pt = ToPoint<PointType>(pt_transforemd);
        pt.intensity = cloud->points[i].intensity;
        transformed_cloud->points[i] = pt;
    }
    return transformed_cloud;
}

PointCloudPtr TransformLidarOMP(const PointCloudPtr& cloud, const M3D& R, const V3D& t) {
    PointCloudPtr transformed_cloud(new PointCloudType);
    transformed_cloud->resize(cloud->size());
#ifdef MP_EN
    omp_set_num_threads(4);
#pragma omp parallel for
#endif
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto pt_eigen = ToV3D(cloud->points[i]);
        const auto pt_transforemd = R * pt_eigen + t;
        PointType pt = ToPoint<PointType>(pt_transforemd);
        pt.intensity = cloud->points[i].intensity;
        transformed_cloud->points[i] = pt;
    }
    return transformed_cloud;
}

PointCloudPtr TransformLidar(const PointCloudPtr& cloud, const M3D& r, const V3D& t) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = r.cast<float>();
    transform.block<3, 1>(0, 3) = t.cast<float>();
    PointCloudPtr ret(new PointCloudType);
    pcl::transformPointCloud(*cloud, *ret, transform);
    return ret;
}

}  // namespace slam