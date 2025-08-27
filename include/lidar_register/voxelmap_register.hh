#pragma once
#include "lidar_register.hh"
#include "voxel_map.hh"

namespace slam {
class VoxelMapRegister : public LidarRegister {
   public:
    VoxelMapRegister(const std::shared_ptr<SystemConfig> &system_config, std::shared_ptr<IESKF> kf_ptr);

    ~VoxelMapRegister();

    virtual bool InitMap(PointCloudPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) override;
    virtual bool Align(PointCloudPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) override;
    virtual void UpdateLidarFunc(NavState &nav_state, ESKFShareState &shared_data) override;
    virtual void UpdateMap() override;

   private:
    M3D transformLiDARCovToWorld(const Eigen::Vector3d &point_lidar, const std::shared_ptr<IESKF> kf_ptr,
                                 const PoseTrans &T_IL, const Eigen::Matrix3d &cov_lidar);

    void calcBodyCov(Eigen::Vector3d &pb, const double &range_inc, const double &degree_inc, Eigen::Matrix3d &cov);

   private:
    double voxel_size_;
    int max_layer_;
    std::vector<int> layer_point_size_;
    int max_points_size_;
    int max_cov_points_size_;
    float planer_threshold_;
    bool updatemap_omp_;
    int sigma_num_;
    int max_capacity_;

    double range_cov;
    double angle_cov;

    std::shared_ptr<VoxelMap> voxel_map_;

    // 先计算lidar的cov
    std::vector<M3D> lidar_covs_;
    std::vector<ResidualData> residual_infos_;
};
}  // namespace slam