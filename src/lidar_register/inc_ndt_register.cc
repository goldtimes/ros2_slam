/*
 * @Author: lihang lihang@kilox.cn
 * @Date: 2025-09-04 17:38:22
 * @LastEditors: lihang lihang@kilox.cn
 * @LastEditTime: 2025-09-09 09:28:31
 * @FilePath: /fast_lvio_ws/src/open_slam/src/lidar_register/inc_ndt_register.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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
        [this](State &state, ESKFShareState &shared_data) { UpdateLidarFunc(state, shared_data); });
    // 设置迭代停止的条件
    kf_ptr_->SetStopFunc([](const V33D &delta) { return delta.norm() < 1e-6; });
}

IncNdtRegister::~IncNdtRegister() {
    LOG_INFO("~IncNdtRegister");
}

bool IncNdtRegister::InitMap(PointCloudPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) {
    if (first_frame_) {
        // transform cloud_lidar to world frame
        auto current_pose = PoseTrans(kf_ptr_->GetState().rot, kf_ptr_->GetState().pos);
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
    kf_ptr_->UpdateLidar();
    return true;
}
void IncNdtRegister::UpdateLidarFunc(State &nav_state, ESKFShareState &shared_data) {
    ndt_ptr_->ComputeResidualAndJacobians(nav_state, shared_data);
}

void IncNdtRegister::UpdateMap() {
    auto current_pose = PoseTrans(kf_ptr_->GetState().rot, kf_ptr_->GetState().pos);
    auto delta_pose = last_pose_.inverse() * current_pose;

    if (delta_pose.t.norm() > 0.5 || delta_pose.RPY().norm() > (10.0 / 180.0 * M_PI)) {
        auto cloud_world = TransformLidarOMP(ndt_ptr_->source_, current_pose.R, current_pose.t);
        ndt_ptr_->AddCloud(cloud_world);
        last_pose_ = current_pose;
    }
}

PointCloudPtr IncNdtRegister::GetSubmap() {
    PointCloudPtr cloud(new PointCloudType);

    return cloud;
}
}  // namespace slam