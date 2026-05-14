
#pragma once

#include <memory>
#include "common/commons.hh"
#include "common/state.hh"
#include "lio/ieskf.hh"
#include "system_config.hh"
#include "utils/logger.hh"
#include "utils/pointcloud_utils.hh"

namespace slam {

// 抽象的类
class LidarRegister {
   public:
    LidarRegister(const std::shared_ptr<SystemConfig>& system_config, std::shared_ptr<IESKF> kf_ptr)
        : system_config_(system_config), kf_ptr_(kf_ptr) {
    }
    virtual ~LidarRegister() = default;
    // 初始化地图
    virtual bool InitMap(PointCloudXYZIPtr& cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) = 0;
    // 配准并更新地图
    virtual bool Align(PointCloudXYZIPtr& cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) = 0;

    virtual void UpdateLidarFunc(State& nav_state, ESKFShareState& shared_data) = 0;

    virtual void UpdateMap() = 0;

    virtual PointCloudXYZIPtr GetSubmap() = 0;

    bool IsKeyFrame() {
        return is_keyframe_;
    }

   protected:
    bool first_frame_ = true;
    std::shared_ptr<IESKF> kf_ptr_;
    std::shared_ptr<SystemConfig> system_config_;
    PointCloudXYZIPtr current_lidar_;  // 原始的雷达点云
    bool is_keyframe_ = false;
    PoseTrans last_keypose_;
    std::mutex local_map_mutex_;
    PointCloudXYZIPtr submap_;
    std::deque<std::pair<PoseTrans, PointCloudXYZIPtr>> keyframes_;

    pcl::VoxelGrid<PointXYZI> voxel_grid_;

    int keyframe_size_ = 0;
    double keyframe_distance_ = 0.0;
    double keyframe_angle_distance_ = 0.0;
    bool use_angle_keyframe_ = false;
    int updated_failed_num_ = 0;
};

}  // namespace slam