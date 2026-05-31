#include "pointcloud_utils.hh"

namespace slam {

PointCloudXYZIPtr TransformLidarOMP(const PointCloudXYZIPtr& cloud, const SE3& transform) {
    PointCloudXYZIPtr transformed_cloud(new PointCloudXYZI);
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
        PointXYZI pt = ToPoint<PointXYZI>(pt_transforemd);
        pt.intensity = cloud->points[i].intensity;
        transformed_cloud->points[i] = pt;
    }
    return transformed_cloud;
}

PointCloudXYZIPtr TransformLidarOMP(const PointCloudXYZIPtr& cloud, const M3D& R, const V3D& t) {
    PointCloudXYZIPtr transformed_cloud(new PointCloudXYZI);
    transformed_cloud->resize(cloud->size());
#ifdef MP_EN
    omp_set_num_threads(4);
#pragma omp parallel for
#endif
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto pt_eigen = ToV3D(cloud->points[i]);
        const auto pt_transforemd = R * pt_eigen + t;
        PointXYZI pt = ToPoint<PointXYZI>(pt_transforemd);
        pt.intensity = cloud->points[i].intensity;
        transformed_cloud->points[i] = pt;
    }
    return transformed_cloud;
}

PointCloudXYZIPtr TransformLidar(const PointCloudXYZIPtr& cloud, const M3D& r, const V3D& t) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = r.cast<float>();
    transform.block<3, 1>(0, 3) = t.cast<float>();
    PointCloudXYZIPtr ret(new PointCloudXYZI);
    pcl::transformPointCloud(*cloud, *ret, transform);
    return ret;
}

PointCloudXYZIPtr VoxelFilter(const PointCloudXYZIPtr& cloud, float leaf_size) {
    PointCloudXYZIPtr ret(new PointCloudXYZI);
    pcl::VoxelGrid<PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*ret);
    return ret;
}

}  // namespace slam