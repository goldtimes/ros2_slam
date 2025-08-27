#include "lidar_register/inc_ndt_register.hh"

namespace slam {
IncNdtRegister::IncNdtRegister(const std::shared_ptr<SystemConfig> &system_config, std::shared_ptr<IESKF> kf_ptr)
    : LidarRegister(system_config, kf_ptr) {
    LOG_INFO("IncNdtRegister init");
    voxel_size_ = system_config->frontend_config_.ndt_config.voxel_size;
    near_search_ = system_config->frontend_config_.ndt_config.near_search;
    max_capacity_ = system_config->frontend_config_.ndt_config.max_capacity;
    min_effective_pts_ = system_config->frontend_config_.ndt_config.min_effective_pts;
    min_pts_in_voxel_ = system_config->frontend_config_.ndt_config.min_pts_in_voxel;
    max_pts_in_voxel_ = system_config->frontend_config_.ndt_config.max_pts_in_voxel;
    res_outlier_thresh_ = system_config->frontend_config_.ndt_config.res_outlier_thresh;
    eps_ = system_config->frontend_config_.ndt_config.eps;
    calib_lidar2imu_ = system_config->frontend_config_.calib_lidar2imu;

    ndt_ptr_ = std::make_shared<IncNdt>(voxel_size_, near_search_, max_capacity_, min_effective_pts_, min_pts_in_voxel_,
                                        max_pts_in_voxel_, res_outlier_thresh_, eps_, calib_lidar2imu_);
    // 设置雷达损失函数
    kf_ptr_->SetLidarLossFunc(
        [this](NavState &state, ESKFShareState &shared_data) { UpdateLidarFunc(state, shared_data); });
    // 设置迭代停止的条件
    kf_ptr_->SetStopFunc([](const V21D &delta) { return delta.norm() < 1e-4; });
}

IncNdtRegister::~IncNdtRegister() {
    LOG_INFO("~IncNdtRegister");
}

bool IncNdtRegister::InitMap(PointCloudPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) {
    if (first_frame_) {
        // transform cloud_lidar to world frame
        auto current_pose = PoseTrans(kf_ptr_->GetState().r_wi, kf_ptr_->GetState().t_wi);
        auto T_WL = current_pose * system_config_->lidar2imu_;
        auto cloud_world_tmp = TransformLidarOMP(cloud_lidar, T_WL.R, T_WL.t);
        // pcl::io::savePCDFileBinary("cloud_world_tmp.pcd", *cloud_world_tmp);
        ndt_ptr_->AddCloud(cloud_world_tmp);
        first_frame_ = false;
    }
    return true;
}

bool IncNdtRegister::Align(PointCloudPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) {
    // transform cloud_lidar to body
    auto cloud_body = TransformLidarOMP(cloud_lidar, system_config_->lidar2imu_.R, system_config_->lidar2imu_.t);
    // 降采样
    ndt_ptr_->SetSource(cloud_body);
    kf_ptr_->Update();
    return true;
}
void IncNdtRegister::UpdateLidarFunc(NavState &nav_state, ESKFShareState &shared_data) {
    ndt_ptr_->ComputeResidualAndJacobians(nav_state, shared_data);
}

void IncNdtRegister::UpdateMap() {
    auto current_pose = PoseTrans(kf_ptr_->GetState().r_wi, kf_ptr_->GetState().t_wi);
    auto delta_pose = last_pose_.inverse() * current_pose;

    if (delta_pose.t.norm() > 1.0 || delta_pose.RPY().norm() > (10.0 / 180.0 * M_PI)) {
        auto cloud_world = TransformLidarOMP(ndt_ptr_->source_, current_pose.R, current_pose.t);
        ndt_ptr_->AddCloud(cloud_world);
        last_pose_ = current_pose;
    }
}
}  // namespace slam