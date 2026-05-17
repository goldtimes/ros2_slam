#pragma once
#include "lidar_register.hh"
#include "lio/ivox3d/ivox3d.h"
#include <pcl/filters/voxel_grid.h>

namespace slam {
class FasterlioRegister : public LidarRegister {
public:
  using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointXYZI>;
  using PointVector =
      std::vector<PointXYZI, Eigen::aligned_allocator<PointXYZI>>;
  FasterlioRegister(const std::shared_ptr<SystemConfig> &system_config,
                    std::shared_ptr<IESKF> kf_ptr);

  ~FasterlioRegister();

  virtual bool InitMap(PointCloudXYZIPtr &cloud_lidar,
                       std::shared_ptr<IESKF> kf_ptr_) override;
  virtual bool Align(PointCloudXYZIPtr &cloud_lidar,
                     std::shared_ptr<IESKF> kf_ptr_) override;
  virtual void UpdateLidarFunc(State &nav_state,
                               ESKFShareState &shared_data) override;
  virtual void UpdateMap() override;
  virtual PointCloudXYZIPtr GetSubmap() override;

private:
  double ivox_grid_resolution_;
  double planner_threshold_;
  int ivox_nearby_type_; // 6, 18, 26
  IVoxType::Options ivox_options_;
  std::shared_ptr<IVoxType> ivox_ = nullptr;
  std::vector<float> residuals_;            // point-to-plane residuals
  std::vector<bool> point_selected_surf_;   // selected points
  std::vector<Vec4f> plane_coef_;           // plane coeffs
  std::vector<PointVector> nearest_points_; // nearest points of current scan
};
} // namespace slam