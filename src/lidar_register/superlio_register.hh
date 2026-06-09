/*
 * @Author: lihang lihang@kilox.cn
 * @Date: 2025-09-04 17:38:22
 * @LastEditors: lihang lihang@kilox.cn
 * @LastEditTime: 2025-10-10 13:41:41
 * @FilePath:
 * /fast_lvio_ws/src/lio_slam/include/lidar_register/p2plane_register.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%A
 */
#pragma once
#include "OctVoxMap/OctVoxMap.hpp"
#include "OctVoxMap/VoxeGridlFilter.h"
#include "common/eigen_type.hh"
#include "common/lidar_point_type.hh"
#include "lidar_register.hh"
#include "utils/pointcloud_utils.hh"
namespace slam {

class SuperLIORegister : public LidarRegister {
   public:
    struct ThreadACC {
        M12D HTVH = M12D::Zero();
        V12D HTVr = V12D::Zero();
        ThreadACC() : HTVH(M12D::Zero()), HTVr(V12D::Zero()) {
        }
    };
    SuperLIORegister(const std::shared_ptr<SystemConfig> &system_config, std::shared_ptr<IESKF> kf_ptr);

    ~SuperLIORegister();

    virtual bool InitMap(PointCloudXYZIPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) override;
    virtual bool Align(PointCloudXYZIPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) override;
    virtual void UpdateLidarFunc(State &nav_state, ESKFShareState &shared_data) override;
    virtual void UpdateMap() override;
    virtual PointCloudXYZIPtr GetSubmap() override;
    virtual void CacheData() override;
    virtual void SaveMap() override;

   private:
    bool calc_plane_coeff(const int N, const std::array<V3D, 5> &points, std::array<double, 4> &abcd);
    bool compute_error(const std::array<double, 4> &abcd, const V3D &point, const float length, double &error);

   private:
    using OctVoxMapType = OctVoxMap<Eigen::Vector3d, double>;
    using KNNHeapType = KNNHeap<5, Eigen::Vector3d>;

    OctVoxMapType::Ptr ivox_;
    VoxelGridClosest<PointXYZI> voxel_grid_fliter_;
    PointCloudXYZIPtr cloud_world;
    std::size_t effect_knn_num_ = 0;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points_world;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points_body;

    alignas(64) bool effect_mask_[20000] = {false};
    alignas(64) bool effect_knn_mask_[20000] = {false};
    std::vector<int> effect_knn_idxs_;

    std::vector<std::pair<Eigen::Matrix<double, 6, 6>, Eigen::Matrix<double, 6, 1>>> H_R_;
    std::vector<std::array<double, 4>> abcd_vec_;
    int pcd_index_ = -1;
    double lidar_noise_std_;
    double lidar_info_matrix_;
    bool updated_success = true;

    PointCloudXYZIPtr mapCloud;
};
}  // namespace slam