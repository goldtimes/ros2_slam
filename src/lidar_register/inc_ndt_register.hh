/*
 * @Author: lihang lihang@kilox.cn
 * @Date: 2025-09-04 17:38:22
 * @LastEditors: lihang lihang@kilox.cn
 * @LastEditTime: 2025-09-09 09:22:53
 * @FilePath:
 * /fast_lvio_ws/src/lio_slam/include/lidar_register/inc_ndt_register.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include "inc_ndt.hh"
#include "lidar_register.hh"

namespace slam {
class IncNdtRegister : public LidarRegister {
public:
  IncNdtRegister(const std::shared_ptr<SystemConfig> &system_config,
                 std::shared_ptr<IESKF> kf_ptr);

  ~IncNdtRegister();

  virtual bool InitMap(PointCloudXYZIPtr &cloud_lidar,
                       std::shared_ptr<IESKF> kf_ptr_) override;
  virtual bool Align(PointCloudXYZIPtr &cloud_lidar,
                     std::shared_ptr<IESKF> kf_ptr_) override;
  virtual void UpdateLidarFunc(State &nav_state,
                               ESKFShareState &shared_data) override;
  virtual void UpdateMap() override;
  virtual PointCloudXYZIPtr GetSubmap() override;
  virtual void CacheData() override;
  virtual void SaveMap() override;

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
} // namespace slam