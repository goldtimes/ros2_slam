#pragma once

#include <functional>
#include <memory>
#include "commons.hh"
#include "logger.hh"
#include "state.hh"
#include "system_config.hh"

namespace slam {

// 抽象的类
class LidarRegister {
   public:
    LidarRegister(const std::shared_ptr<SystemConfig>& system_config) : system_config_(system_config) {
    }
    virtual ~LidarRegister() = default;

    virtual bool InitMap(PointCloudPtr& cloud_world) = 0;

    virtual void SetLidarLossFunc(std::function<void(NavState&, ESKFShareState&)> lidar_loss_func) {
        lidar_loss_func_ = lidar_loss_func;
    }

   protected:
    bool first_frame_;
    std::shared_ptr<SystemConfig> system_config_;
    PointCloudPtr cloud_lidar_;  // 原始的雷达点云
    std::function<void(NavState&, ESKFShareState&)> lidar_loss_func_;
};

}  // namespace slam