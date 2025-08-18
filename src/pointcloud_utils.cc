#include "pointcloud_utils.hh"

namespace slam {

PointCloudPtr TransformLidarOMP(const PointCloudPtr& cloud, const SE3& transform) {
    PointCloudPtr transformed_cloud(new PointCloudPtr);
    transformed_cloud->resize(cloud->size());
#ifdef MP_EN
    omp_set_num_threads(6);
#pragma omp parallel for
#endif
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto pt_eigen = ToV3D(cloud->points[i]);
        const auto pt_transforemd = transform.so3() * pt_eigen + transform.translation();
        // transformed_cloud->points[i] = transform * pt_eigen;
        transformed_cloud->points[i] = ToPoint<PointType>(pt_transforemd);
    }
    return transformed_cloud;
}

}  // namespace slam