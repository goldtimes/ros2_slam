#pragma once

#include <memory>
#include "commons.hh"
#include "ieskf.hh"
#include "logger.hh"
#include "pointcloud_utils.hh"
#include "state.hh"
#include "system_config.hh"
#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0

namespace slam {

// 抽象的类
class LidarRegister {
   public:
    LidarRegister(const std::shared_ptr<SystemConfig>& system_config, std::shared_ptr<IESKF> kf_ptr)
        : system_config_(system_config), kf_ptr_(kf_ptr) {
    }
    virtual ~LidarRegister() = default;
    // 初始化地图
    virtual bool InitMap(PointCloudPtr& cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) = 0;
    // 配准并更新地图
    virtual bool Align(PointCloudPtr& cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) = 0;

    virtual void UpdateLidarFunc(State& nav_state, ESKFShareState& shared_data) = 0;

    virtual void UpdateMap() = 0;

   protected:
    bool first_frame_ = true;
    std::shared_ptr<IESKF> kf_ptr_;
    std::shared_ptr<SystemConfig> system_config_;
    PointCloudPtr current_lidar_;  // 原始的雷达点云
};

}  // namespace slam