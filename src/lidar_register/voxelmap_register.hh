
#pragma once
#include <pcl/filters/voxel_grid.h>
#include "lidar_register.hh"
#include "voxel_map.hh"

namespace slam {
class VoxelMapRegister : public LidarRegister {
   public:
    VoxelMapRegister(const std::shared_ptr<SystemConfig> &system_config, std::shared_ptr<IESKF> kf_ptr);

    ~VoxelMapRegister();

    virtual bool InitMap(PointCloudXYZIPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) override;
    virtual bool Align(PointCloudXYZIPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) override;
    virtual void UpdateLidarFunc(State &nav_state, ESKFShareState &shared_data) override;
    virtual void UpdateMap() override;
    virtual PointCloudXYZIPtr GetSubmap() override;

   private:
    // M3D transformLiDARCovToWorld(const Eigen::Vector3d &point_lidar, const std::shared_ptr<IESKF> kf_ptr,
    //                              const PoseTrans &T_IL, const Eigen::Matrix3d &cov_lidar);

    M3D calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc);

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
    std::vector<ResidualData> residual_infos_;
};
}  // namespace slam