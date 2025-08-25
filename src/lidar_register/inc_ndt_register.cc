#include "lidar_register/inc_ndt_register.hh"

namespace slam {
IncNdtRegister::IncNdtRegister(const std::shared_ptr<SystemConfig> &system_config, std::shared_ptr<IESKF> kf_ptr)
    : LidarRegister(system_config, kf_ptr) {
    LOG_INFO("IncNdtRegister");
    voxel_size_ = system_config->frontend_config_.ndt_config.voxel_size;
    near_search_ = system_config->frontend_config_.ndt_config.near_search;
    max_capacity_ = system_config->frontend_config_.ndt_config.max_capacity;
    min_effective_pts_ = system_config->frontend_config_.ndt_config.min_effective_pts;
    min_pts_in_voxel_ = system_config->frontend_config_.ndt_config.min_pts_in_voxel;
    max_pts_in_voxel_ = system_config->frontend_config_.ndt_config.max_pts_in_voxel;
    res_outlier_thresh_ = system_config->frontend_config_.ndt_config.res_outlier_thresh;
    eps_ = system_config->frontend_config_.ndt_config.eps;

    // 设置雷达损失函数
    kf_ptr_->SetLidarLossFunc(
        [this](NavState &state, ESKFShareState &shared_data) { UpdateLidarFunc(state, shared_data); });
    // 设置迭代停止的条件
    kf_ptr_->SetStopFunc([](const V21D &delta) { return delta.norm() < 1e-3; });
}

IncNdtRegister::~IncNdtRegister() {
    LOG_INFO("~IncNdtRegister");
}

bool IncNdtRegister::InitMap(PointCloudPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) {
    return true;
}

bool IncNdtRegister::Align(PointCloudPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) {
    return true;
}
void IncNdtRegister::UpdateLidarFunc(NavState &nav_state, ESKFShareState &shared_data) {
}

void IncNdtRegister::UpdateMap() {
}
}  // namespace slam