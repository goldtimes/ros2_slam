#include "lidar_register/voxelmap_register.hh"

namespace slam {
VoxelMapRegister::VoxelMapRegister(const std::shared_ptr<SystemConfig> &system_config, std::shared_ptr<IESKF> kf_ptr)
    : LidarRegister(system_config, kf_ptr) {
    LOG_INFO("VoxelMapRegister init done!");
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
    voxel_map_ =
        std::make_shared<VoxelMap>(voxel_size_, max_layer_, max_points_size_, planer_threshold_, max_capacity_);
    // 设置雷达损失函数
    kf_ptr_->SetLidarLossFunc(
        [this](NavState &state, ESKFShareState &shared_data) { UpdateLidarFunc(state, shared_data); });
    // 设置迭代停止的条件
    kf_ptr_->SetStopFunc([](const V21D &delta) { return delta.norm() < 1e-6; });

    residual_infos_.resize(10000);
}

VoxelMapRegister::~VoxelMapRegister() {
}

bool VoxelMapRegister::InitMap(PointCloudPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) {
    if (first_frame_) {
        // transform cloud_lidar to world frame
        auto current_pose = PoseTrans(kf_ptr_->GetState().r_wi, kf_ptr_->GetState().t_wi);
        auto T_WL = current_pose * system_config_->lidar2imu_;
        auto cloud_world_tmp = TransformLidarOMP(cloud_lidar, T_WL.R, T_WL.t);
        std::vector<PointWithCov> pv_list;
        for (size_t i = 0; i < cloud_lidar->size(); ++i) {
            auto pt_lidar = ToV3D(cloud_lidar->points[i]);
            auto pt_world = ToV3D(cloud_world_tmp->points[i]);
            PointWithCov pv;
            pv.point = pt_world;
            // 防止为0
            if (pt_lidar[2] == 0) {
                pt_lidar[2] = 0.001;
            }
            // 计算雷达点的测量协方差
            M3D cov_lidar;
            calcBodyCov(pt_lidar, range_cov, angle_cov, cov_lidar);
            // 计算世界坐标下的点的协方差
            M3D point_body_crossmat = Sophus::SO3d::hat(pt_lidar);
            M3D cov_world;
            cov_world = T_WL.R * cov_lidar * T_WL.R.transpose() +
                        point_body_crossmat * kf_ptr_->GetCov().block<3, 3>(0, 0) * point_body_crossmat.transpose() +
                        kf_ptr_->GetCov().block<3, 3>(3, 3);
            pv.cov = cov_world;
            pv_list.push_back(pv);
        }
        voxel_map_->Insert(pv_list);
        LOG_INFO("Build Voxel Map Size: {}", voxel_map_->cache.size());
        first_frame_ = false;
    }
    return true;
}

bool VoxelMapRegister::Align(PointCloudPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) {
    lidar_covs_.clear();
    current_lidar_ = cloud_lidar;
    // 降采样
    for (size_t i = 0; i < current_lidar_->size(); ++i) {
        auto pt_lidar = ToV3D(current_lidar_->points[i]);
        if (pt_lidar[2] == 0) {
            pt_lidar[2] = 0.001;
        }
        residual_infos_[i].point_lidar = pt_lidar;
        M3D cov_lidar;
        calcBodyCov(pt_lidar, range_cov, angle_cov, cov_lidar);
        residual_infos_[i].pcov = cov_lidar;
    }
    kf_ptr_->Update();
    return true;
}

void VoxelMapRegister::UpdateMap() {
    // 更新地图
    auto current_pose = PoseTrans(kf_ptr_->GetState().r_wi, kf_ptr_->GetState().t_wi);
    auto T_WL = current_pose * system_config_->lidar2imu_;
    // to world cloud
    auto cloud_world = TransformLidarOMP(current_lidar_, T_WL.R, T_WL.t);
    // 计算point with cov
    std::vector<PointWithCov> pv_list;
    for (size_t i = 0; i < cloud_world->size(); ++i) {
        PointWithCov pv;
        auto pt_lidar = ToV3D(current_lidar_->points[i]);
        auto pt_world = ToV3D(cloud_world->points[i]);
        pv.point = pt_world;
        M3D cov_lidar = residual_infos_[i].pcov;
        Eigen::Matrix3d point_body_crossmat = Sophus::SO3d::hat(residual_infos_[i].point_lidar);

        M3D cov_world;
        cov_world = T_WL.R * cov_lidar * T_WL.R.transpose() +
                    point_body_crossmat * kf_ptr_->GetCov().block<3, 3>(0, 0) * point_body_crossmat.transpose() +
                    kf_ptr_->GetCov().block<3, 3>(3, 3);
        pv.cov = cov_world;
        pv_list.push_back(pv);
    }
    voxel_map_->Insert(pv_list);
}

void VoxelMapRegister::UpdateLidarFunc(NavState &nav_state, ESKFShareState &shared_data) {
    // 这里计算了H,b矩阵，然后通过高斯牛顿求解之后，更新ieskf维护的状态量
    // 当前的状态
    M3D rot_end = nav_state.r_wi;
    V3D trans_end = nav_state.t_wi;
    auto current_pose = PoseTrans(rot_end, trans_end);
    auto T_WL = current_pose * system_config_->lidar2imu_;
    // to world cloud
    auto cloud_world = TransformLidarOMP(current_lidar_, T_WL.R, T_WL.t);
    double total_res = 0.0;
    int size = cloud_world->size();
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    // 计算point with cov
    for (size_t i = 0; i < size; ++i) {
        residual_infos_[i].is_valid = false;
        residual_infos_[i].current_layer = 0;
        residual_infos_[i].from_near = false;
        residual_infos_[i].point_world = ToV3D(cloud_world->points[i]);

        Eigen::Matrix3d point_crossmat = Sophus::SO3d::hat(residual_infos_[i].point_lidar);
        residual_infos_[i].cov = T_WL.R * residual_infos_[i].pcov * T_WL.R.transpose() +
                                 point_crossmat * kf_ptr_->GetCov().block<3, 3>(0, 0) * point_crossmat.transpose() +
                                 kf_ptr_->GetCov().block<3, 3>(3, 3);
        VoxelKey position = voxel_map_->Index(residual_infos_[i].point_world);
        auto iter = voxel_map_->feat_map.find(position);
        if (iter != voxel_map_->feat_map.end()) {
            voxel_map_->BuildResidual(residual_infos_[i], iter->second.tree);
        }
    }

    shared_data.H_.setZero();
    shared_data.b_.setZero();
    Eigen::Matrix<double, 1, 12> J;
    Eigen::Matrix<double, 1, 6> J_v;
    int effect_num = 0;

    for (int i = 0; i < size; i++) {
        if (!residual_infos_[i].is_valid) continue;
        effect_num++;
        J.setZero();
        J_v.block<1, 3>(0, 0) = (residual_infos_[i].point_world - residual_infos_[i].plane_center).transpose();
        J_v.block<1, 3>(0, 3) = -residual_infos_[i].plane_norm.transpose();
        double r_cov = J_v * residual_infos_[i].plane_cov * J_v.transpose();
        r_cov += residual_infos_[i].plane_norm.transpose() * T_WL.R * residual_infos_[i].pcov * T_WL.R.transpose() *
                 residual_infos_[i].plane_norm;
        double r_info = r_cov < 0.0001 ? 1000 : 1 / r_cov;
        assert(r_cov > 0.0);
        J.block<1, 3>(0, 0) = residual_infos_[i].plane_norm.transpose();
        J.block<1, 3>(0, 3) = -residual_infos_[i].plane_norm.transpose() * state.rot *
                              Sophus::SO3d::hat(state.rot_ext * residual_infos_[i].point_lidar + state.pos_ext);
        if (params_.estimate_ext) {
            J.block<1, 3>(0, 6) =
                -residual_infos_[i].plane_norm.transpose() * r_wl * Sophus::SO3d::hat(residual_infos_[i].point_lidar);
            J.block<1, 3>(0, 9) = residual_infos_[i].plane_norm.transpose() * state.rot;
        }
        total_res += std::fabs(residual_infos_[i].residual);
        shared_state.H += J.transpose() * r_info * J;
        shared_state.b += J.transpose() * r_info * residual_infos_[i].residual;
    }
    if (effect_num < 1) {
        LOG_ERROR("NO EFFECTIVE POINT");
        shared_data.is_valid = false;
        return;
    }
    LOG_INFO("total_res:{}", total_res / effect_num);
}

void VoxelMapRegister::calcBodyCov(Eigen::Vector3d &pb, const double &range_inc, const double &degree_inc,
                                   Eigen::Matrix3d &cov) {
    if (pb[2] == 0) {
        pb[2] = 0.001;
    }
    double range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
    double range_var = range_inc * range_inc;
    Eigen::Matrix2d direction_var;
    direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);
    Eigen::Vector3d direction(pb);
    direction.normalize();
    Eigen::Matrix3d direction_hat;
    direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;
    Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
    base_vector1.normalize();
    Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
    base_vector2.normalize();
    Eigen::Matrix<double, 3, 2> N;
    N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);
    Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
    cov = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
}

M3D VoxelMapRegister::transformLiDARCovToWorld(const Eigen::Vector3d &point_lidar, const std::shared_ptr<IESKF> kf_ptr,
                                               const PoseTrans &lidar_to_imu, const Eigen::Matrix3d &cov_lidar) {
    Eigen::Matrix3d point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_lidar);

    // lidar到body的方差传播
    // conjugate() 共轭
    Eigen::Matrix3d cov_body = lidar_to_imu.R * cov_lidar * lidar_to_imu.R.transpose() +
                               lidar_to_imu.R * (-point_crossmat) * kf_ptr->GetCov().block<3, 3>(6, 6) *
                                   (-point_crossmat).transpose() * lidar_to_imu.R.transpose() +
                               kf_ptr->GetCov().block<3, 3>(9, 9);
    // P_L =  T_L_to_I * P_L
    // Eigen::Vector3d p_body = lidar_to_imu.rotationMatrix() * point_lidar + lidar_to_imu.translation();
    Eigen::Vector3d p_body = lidar_to_imu * point_lidar;

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