#include "lidar_register/voxelmap_register.hh"

namespace slam {
VoxelMapRegister::VoxelMapRegister(const std::shared_ptr<SystemConfig> &system_config, std::shared_ptr<IESKF> kf_ptr)
    : LidarRegister(system_config, kf_ptr) {
    LOG_INFO("VoxelMapRegister init done!");
    il_t_var = M3D::Zero();
    il_r_var = M3D::Identity() * std::pow(0.001, 2);
    voxel_size_ = system_config_->frontend_config_.voxel_config.voxle_size;
    max_layer_ = system_config_->frontend_config_.voxel_config.max_layer;
    layer_point_size_ = system_config_->frontend_config_.voxel_config.layer_point_size;
    max_points_size_ = system_config_->frontend_config_.voxel_config.max_points_size;
    max_cov_points_size_ = system_config_->frontend_config_.voxel_config.max_cov_points_size;
    planer_threshold_ = system_config_->frontend_config_.voxel_config.plannar_threshold;
    max_capacity_ = system_config_->frontend_config_.voxel_config.max_capacity;
    range_cov = system_config->frontend_config_.voxel_config.ranging_cov;
    angle_cov = system_config->frontend_config_.voxel_config.angle_cov;
    LOG_INFO(
        "voxel_size_: {}, max_layer_: {}, max_points_size_: {}, max_cov_points_size_: {}, "
        "planer_threshold_: {}",
        voxel_size_, max_layer_, max_points_size_, max_cov_points_size_, planer_threshold_);
    updatemap_omp_ = system_config_->frontend_config_.voxel_config.updatemap_omp;
    sigma_num_ = system_config_->frontend_config_.voxel_config.sigma_num;

    // 设置雷达损失函数
    kf_ptr_->SetLidarLossFunc(
        [this](NavState &state, ESKFShareState &shared_data) { UpdateLidarFunc(state, shared_data); });
    // 设置迭代停止的条件
    kf_ptr_->SetStopFunc([](const V21D &delta) { return delta.norm() < 1e-6; });
}

VoxelMapRegister::~VoxelMapRegister() {
}

bool VoxelMapRegister::InitMap(PointCloudPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) {
    if (first_frame_) {
        // transform cloud_lidar to world frame
        auto current_pose = SE3(kf_ptr_->GetState().r_wi, kf_ptr_->GetState().t_wi);
        auto T_WL = current_pose * system_config_->lidar2robot_;
        auto cloud_world = TransformLidarOMP(cloud_lidar, T_WL);
        std::vector<pointWithCov> pv_list;
        for (size_t i = 0; i < cloud_lidar->size(); ++i) {
            auto pt_lidar = ToV3D(cloud_lidar->points[i]);
            auto pt_world = ToV3D(cloud_world->points[i]);
            pointWithCov pv;
            pv.point = pt_world;
            // 防止为0
            if (pt_lidar[2] == 0) {
                pt_lidar[2] = 0.001;
            }
            // 计算雷达点的测量协方差
            M3D cov_lidar;
            cov_lidar = calcBodyCov(pt_lidar, system_config_->frontend_config_.voxel_config.ranging_cov,
                                    system_config_->frontend_config_.voxel_config.angle_cov);
            // 计算世界坐标下的点的协方差
            M3D cov_world;
            cov_world = transformLiDARCovToWorld(pt_lidar, kf_ptr_, system_config_->lidar2robot_, cov_lidar);
            pv.cov = cov_world;
            pv_list.push_back(pv);
        }
        buildVoxelMap(pv_list, voxel_size_, max_layer_, layer_point_size_, max_points_size_, max_cov_points_size_,
                      planer_threshold_, voxel_map_, data_, grids_);
        LOG_INFO("Build Voxel Map Size: {}", voxel_map_.size());
        first_frame_ = false;
    }
    return true;
}

bool VoxelMapRegister::Align(PointCloudPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) {
    lidar_covs_.clear();
    current_lidar_ = cloud_lidar;
    for (size_t i = 0; i < cloud_lidar->size(); ++i) {
        auto pt_lidar = ToV3D(cloud_lidar->points[i]);
        if (pt_lidar[2] == 0) {
            pt_lidar[2] = 0.001;
        }
        M3D cov_lidar, cov_world;
        cov_lidar = calcBodyCov(pt_lidar, range_cov, angle_cov);
        lidar_covs_.push_back(cov_lidar);
    }
    kf_ptr_->Update();
    return true;
}

void VoxelMapRegister::UpdateMap() {
    // 更新地图
    auto current_pose = SE3(kf_ptr_->GetState().r_wi, kf_ptr_->GetState().t_wi);
    auto T_WL = current_pose * system_config_->lidar2robot_;
    // to world cloud
    auto cloud_world = TransformLidarOMP(current_lidar_, T_WL);
    // 计算point with cov
    std::vector<pointWithCov> pv_list;
    for (size_t i = 0; i < cloud_world->size(); ++i) {
        pointWithCov pv;
        auto pt_lidar = ToV3D(current_lidar_->points[i]);
        auto pt_world = ToV3D(cloud_world->points[i]);
        pv.point = pt_world;
        M3D cov_lidar = lidar_covs_[i];
        M3D cov_world = transformLiDARCovToWorld(pt_lidar, kf_ptr_, system_config_->lidar2robot_, lidar_covs_[i]);
        pv.cov = cov_world;
        pv_list.push_back(pv);
    }
    updateVoxelMap(pv_list, voxel_size_, max_layer_, layer_point_size_, max_points_size_, max_points_size_,
                   planer_threshold_, voxel_map_, max_capacity_, data_, grids_);
}

void VoxelMapRegister::UpdateLidarFunc(NavState &nav_state, ESKFShareState &shared_data) {
    // 这里计算了H,b矩阵，然后通过高斯牛顿求解之后，更新ieskf维护的状态量
    // 当前的状态
    M3D rot_end = nav_state.r_wi;
    V3D trans_end = nav_state.t_wi;
    double total_res = 0.0;
    auto effct_feat_num = 0;
    auto current_pose = SE3(kf_ptr_->GetState().r_wi, kf_ptr_->GetState().t_wi);
    auto T_WL = current_pose * system_config_->lidar2robot_;
    // to world cloud
    auto cloud_world = TransformLidarOMP(current_lidar_, T_WL);
    std::vector<ptpl> ptpl_list;
    std::vector<pointWithCov> pv_list;
    // 计算point with cov
    for (size_t i = 0; i < cloud_world->size(); ++i) {
        pointWithCov pv;
        auto pt_lidar = ToV3D(current_lidar_->points[i]);
        auto pt_world = ToV3D(cloud_world->points[i]);
        pv.point = pt_lidar;
        pv.point_world = pt_world;
        pv.cov_lidar = lidar_covs_[i];
        M3D cov_world = transformLiDARCovToWorld(pt_lidar, kf_ptr_, system_config_->lidar2robot_, lidar_covs_[i]);
        pv.cov = cov_world;
        pv_list.push_back(pv);
    }
    // scan to match
    std::vector<V3D> non_match_list;
    BuildResidualListOMP(voxel_map_, voxel_size_, 3, max_layer_, pv_list, ptpl_list, non_match_list);
    effct_feat_num = ptpl_list.size();
    if (effct_feat_num < 10) {
        LOG_ERROR("NO effective points");
        shared_data.valid = false;
    }
    shared_data.valid = true;

    LOG_INFO("find effective points: {}, match ptpl size: {}", effct_feat_num, ptpl_list.size());
    // 配准的信息存储在ptpl_list中
    assert(ptpl_list.size() == effct_feat_num);
    // 可以用omp加速处理
    shared_data.H_.setZero();
    shared_data.b_.setZero();
    Eigen::Matrix<double, 1, 12> J;
    // 论文中的jv
    Eigen::Matrix<double, 1, 6> J_v;
    // #ifdef MP_EN
    //     omp_set_num_threads(MP_PROC_NUM);
    // #pragma omp parallel for
    // #endif
    for (int i = 0; i < effct_feat_num; ++i) {
        const V3D point_lidar = ptpl_list[i].point;
        M3D point_lidar_crossmat;
        point_lidar_crossmat << SKEW_SYM_MATRX(point_lidar);
        V3D point_body = system_config_->lidar2imu_ * point_lidar;
        M3D point_body_crossmat;
        point_body_crossmat << SKEW_SYM_MATRX(point_body);

        V3D norm_vec(ptpl_list[i].normal);
        // r,p, r_il,t_il
        Eigen::Matrix<double, 1, 3> dres_dr;
        dres_dr = -norm_vec.transpose() * rot_end * point_body_crossmat;
        V3D dres_dt = norm_vec;
        if (system_config_->frontend_config_.calib_lidar2imu) {
        } else {
            J.block<1, 3>(0, 0) = dres_dr;
            J.block<1, 3>(0, 3) = dres_dt.transpose();
        }
        // 计算残差点到平面的距离
        float pd2 = norm_vec.x() * ptpl_list[i].point_world.x() + norm_vec.y() * ptpl_list[i].point_world.y() +
                    norm_vec.z() * ptpl_list[i].point_world.z() + ptpl_list[i].d;
        total_res += std::fabs(pd2);
        V3D point_wolrd = ptpl_list[i].point_world;
        // 这里计算论文中的每个观测噪声Ri
        // J_v * 协方差 * J_v^T 。这里不是构造1x9的矩阵，但是本质上是一样的
        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq << point_wolrd - ptpl_list[i].center;
        J_nq << -ptpl_list[i].normal;
        double sigma_l = J_nq * ptpl_list[i].plane_cov * J_nq.transpose();
        M3D cov_lidar = ptpl_list[i].cov_lidar;
        M3D R_cov_Rt = T_WL.so3().matrix() * cov_lidar * T_WL.so3().matrix().transpose();
        double r_cov = sigma_l + norm_vec.transpose() * R_cov_Rt * norm_vec;
        double r_info = r_cov < 0.0001 ? 1000 : 1.0 / r_cov;
        // 这样会导致更新完全倾斜到雷达侧
        shared_data.H_ += J.transpose() * r_info * J;
        shared_data.b_ -= J.transpose() * r_info * pd2;
    }
    LOG_INFO("total_res:{}", total_res / effct_feat_num);
}

M3D VoxelMapRegister::transformLiDARCovToWorld(const Eigen::Vector3d &point_lidar, const std::shared_ptr<IESKF> kf_ptr,
                                               const SE3 &lidar_to_imu, const Eigen::Matrix3d &cov_lidar) {
    Eigen::Matrix3d point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_lidar);

    // lidar到body的方差传播
    // conjugate() 共轭
    Eigen::Matrix3d cov_body = lidar_to_imu.rotationMatrix() * cov_lidar * lidar_to_imu.rotationMatrix().transpose() +
                               lidar_to_imu.rotationMatrix() * (-point_crossmat) * kf_ptr->GetCov().block<3, 3>(6, 6) *
                                   (-point_crossmat).transpose() * lidar_to_imu.rotationMatrix().transpose() +
                               kf_ptr->GetCov().block<3, 3>(9, 9);
    // P_L =  T_L_to_I * P_L
    Eigen::Vector3d p_body = lidar_to_imu.rotationMatrix() * point_lidar + lidar_to_imu.translation();

    point_crossmat << SKEW_SYM_MATRX(p_body);

    Eigen::Matrix3d rot_var = kf_ptr->GetCov().block<3, 3>(0, 0);
    Eigen::Matrix3d t_var = kf_ptr->GetCov().block<3, 3>(3, 3);

    Eigen::Matrix3d cov_world = kf_ptr->GetState().r_wi * cov_body * kf_ptr->GetState().r_wi.transpose() +
                                kf_ptr->GetState().r_wi * (-point_crossmat) * rot_var * (-point_crossmat).transpose() *
                                    kf_ptr->GetState().r_wi.transpose() +
                                t_var;
    return cov_world;
}
}  // namespace slam