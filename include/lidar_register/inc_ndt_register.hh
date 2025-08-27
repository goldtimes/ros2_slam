#pragma once
#include "inc_ndt.hh"
#include "lidar_register.hh"

namespace slam {
class IncNdtRegister : public LidarRegister {
   public:
    IncNdtRegister(const std::shared_ptr<SystemConfig> &system_config, std::shared_ptr<IESKF> kf_ptr);

    ~IncNdtRegister();

    virtual bool InitMap(PointCloudPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) override;
    virtual bool Align(PointCloudPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) override;
    virtual void UpdateLidarFunc(NavState &nav_state, ESKFShareState &shared_data) override;
    virtual void UpdateMap() override;

   private:
    double voxel_size_;
    bool near_search_;
    int max_capacity_;
    int min_effective_pts_;
    int min_pts_in_voxel_;
    int max_pts_in_voxel_;
    double res_outlier_thresh_;
    double eps_;
    bool calib_lidar2imu_;

    std::shared_ptr<IncNdt> ndt_ptr_;
    PoseTrans last_pose_;
};
}  // namespace slam