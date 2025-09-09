/*
 * @Author: lihang lihang@kilox.cn
 * @Date: 2025-09-04 17:38:22
 * @LastEditors: lihang lihang@kilox.cn
 * @LastEditTime: 2025-09-09 11:12:11
 * @FilePath: /fast_lvio_ws/src/open_slam/include/lidar_register/lidar_register.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <memory>
#include "commons.hh"
#include "ieskf.hh"
#include "logger.hh"
#include "pointcloud_utils.hh"
#include "state.hh"
#include "system_config.hh"

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

    virtual PointCloudPtr GetSubmap() = 0;

    bool IsKeyFrame() {
        return is_keyframe_;
    }

   protected:
    bool first_frame_ = true;
    std::shared_ptr<IESKF> kf_ptr_;
    std::shared_ptr<SystemConfig> system_config_;
    PointCloudPtr current_lidar_;  // 原始的雷达点云
    bool is_keyframe_ = false;
    PoseTrans last_keypose_;
};

}  // namespace slam