#include "lidar_register/p2plane_register.hh"

namespace slam {
P2PlaneRegister::P2PlaneRegister(const std::shared_ptr<SystemConfig> &system_config, std::shared_ptr<IESKF> kf_ptr)
    : LidarRegister(system_config, kf_ptr) {
    LOG_INFO("P2PlaneRegister constructor");
    first_frame_ = true;
    // ikd_tree树
    m_ikdtree = std::make_shared<KD_TREE<slam::PointXYZIRT>>();
    m_ikdtree->set_downsample_param(map_resolution);
    map_resolution = system_config->frontend_config_.p2plane_config.map_resolution;
    cube_len = system_config->frontend_config_.p2plane_config.cube_len;
    det_range = system_config->frontend_config_.p2plane_config.det_range;
    move_thresh = system_config->frontend_config_.p2plane_config.move_thresh;
    // 协方差
    lidar_noise_std_ = system_config->lidar_config_.lidar_noise_std;
    // 信息矩阵
    lidar_info_matrix_ = lidar_noise_std_ == 0.0 ? 1000 : 1.0 / lidar_noise_std_;
    // 分配空间
    current_lidar_.reset(new PointCloudType);
    cloud_world.reset(new PointCloudType(10000, 1));
    m_norm_vec.reset(new PointCloudType(10000, 1));
    m_effect_cloud_lidar.reset(new PointCloudType(10000, 1));
    m_effect_norm_vec.reset(new PointCloudType(10000, 1));
    m_nearest_points.resize(10000);
    m_point_selected_flag.resize(10000, false);
    LOG_INFO("map resolution: {}, cube_len:{}, det_range:{}, move_thresh:{}", map_resolution, cube_len, det_range,
             move_thresh);
    // 设置雷达损失函数
    kf_ptr_->SetMaxIterNum(system_config_->frontend_config_.max_iteration);
    kf_ptr_->SetLidarLossFunc(
        [this](State &state, ESKFShareState &shared_data) { UpdateLidarFunc(state, shared_data); });
    // 设置迭代停止的条件
    kf_ptr_->SetStopFunc([&](const V33D &delta) -> bool {
        V3D rot_delta = delta.block<3, 1>(0, 0);
        V3D t_delta = delta.block<3, 1>(3, 0);
        return (rot_delta.norm() * 57.3 < 0.01) && (t_delta.norm() * 100 < 0.015);
    });
}
P2PlaneRegister::~P2PlaneRegister() {
}

bool P2PlaneRegister::InitMap(PointCloudPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) {
    // trans to world cloud
    // set to ikdtree
    if (first_frame_) {
        // transform cloud_lidar to world frame
        auto current_pose = PoseTrans(kf_ptr_->GetState().rot, kf_ptr_->GetState().pos);
        auto T_WL = current_pose * system_config_->lidar2imu_;
        auto cloud_world_tmp = TransformLidarOMP(cloud_lidar, T_WL.R, T_WL.t);
        // pcl::io::savePCDFileBinary("cloud_world_tmp.pcd", *cloud_world_tmp);
        m_ikdtree->Build(cloud_world_tmp->points);
        LOG_INFO("Build Map Size:{}, cloud  size:{}", m_ikdtree->size(), cloud_world_tmp->size());
        first_frame_ = false;
    }
    // const auto current_state = kf_ptr_->GetState();
    // SE3 T_WI(current_state.r_wi, current_state.t_wi);
    // V3D pos_lidar = SE3(current_state.r_wi, current_state.t_wi) * system_config_->lidar2imu_.translation();
    // V3D pose_lidar_test = T_WI.so3().matrix() * system_config_->lidar2imu_.translation() + T_WI.translation();
    // LOG_INFO("pos_lidar: {}, pose_lidar_test: {}", pos_lidar.transpose(), pose_lidar_test.transpose());
    return true;
}

void P2PlaneRegister::TrimCloud() {
    // 清空需要裁剪的区域
    m_local_map.cub_to_rm.clear();
    const auto current_state = kf_ptr_->GetState();
    V3D pos_lidar = PoseTrans(current_state.rot, current_state.pos) * system_config_->lidar2imu_.t;
    // 初始化立方体的范围
    if (!m_local_map.initialized) {
        for (int i = 0; i < 3; ++i) {
            m_local_map.local_map_corner.vertex_min[i] = pos_lidar[i] - cube_len / 2.0;
            m_local_map.local_map_corner.vertex_max[i] = pos_lidar[i] + cube_len / 2.0;
        }
        m_local_map.initialized = true;
        return;
    }
    // 判断是否移动
    float dist_to_map_edge[3][2];
    bool need_move = false;
    double det_thresh = det_range * move_thresh;
    for (int i = 0; i < 3; ++i) {
        dist_to_map_edge[i][0] = std::fabs(pos_lidar[i] - m_local_map.local_map_corner.vertex_min[i]);
        dist_to_map_edge[i][1] = std::fabs(pos_lidar[i] - m_local_map.local_map_corner.vertex_max[i]);
        // 小于阈值就需要移动地图
        if (dist_to_map_edge[i][0] <= det_thresh || dist_to_map_edge[i][1] <= det_thresh) {
            need_move = true;
        }
    }
    if (!need_move) {
        return;
    }
    // 裁剪地图
    BoxPointType new_corner, temp_corner;
    // 拷贝之前的距离
    new_corner = m_local_map.local_map_corner;
    // 计算移动距离（确保移动后激光雷达处于地图中间区域）
    float move_dist =
        std::max((cube_len - 2.0 * move_thresh * det_thresh) * 0.5 * 0.9, double(det_range * (move_thresh - 1)));
    for (int i = 0; i < 3; ++i) {
        temp_corner = m_local_map.local_map_corner;
        // 左移动
        if (dist_to_map_edge[i][0] <= det_thresh) {
            new_corner.vertex_min[i] -= move_dist;
            new_corner.vertex_max[i] -= move_dist;
            temp_corner.vertex_min[i] = m_local_map.local_map_corner.vertex_max[i] - move_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
        // 右移动
        if (dist_to_map_edge[i][1] <= det_thresh) {
            new_corner.vertex_min[i] += move_dist;
            new_corner.vertex_max[i] += move_dist;
            temp_corner.vertex_max[i] = m_local_map.local_map_corner.vertex_min[i] + move_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
    }
    m_local_map.local_map_corner = new_corner;
    PointVec points_history;
    m_ikdtree->acquire_removed_points(points_history);
    if (m_local_map.cub_to_rm.size() > 0) {
        m_ikdtree->Delete_Point_Boxes(m_local_map.cub_to_rm);
    }
    return;
}
void P2PlaneRegister::IncreMap() {
    if (current_lidar_->empty()) {
        return;
    }
    const State &current_state = kf_ptr_->GetState();
    PoseTrans T_WL = PoseTrans(current_state.rot, current_state.pos) * system_config_->lidar2imu_;
    int cloud_size = current_lidar_->size();
    PointVec point_to_add;
    PointVec point_no_need_downsample;
    for (int i = 0; i < cloud_size; ++i) {
        const auto point_lidar = ToV3D(current_lidar_->points[i]);
        const auto point_world = T_WL * point_lidar;
        cloud_world->points[i].x = point_world[0];
        cloud_world->points[i].y = point_world[1];
        cloud_world->points[i].z = point_world[2];
        cloud_world->points[i].intensity = current_lidar_->points[i].intensity;
        if (m_nearest_points[i].empty()) {
            point_to_add.push_back(cloud_world->points[i]);
            continue;
        }

        const PointVec &points_near = m_nearest_points[i];
        bool need_add = true;
        PointType downsample_result, mid_point;
        mid_point.x = std::floor(cloud_world->points[i].x / map_resolution) * map_resolution + 0.5 * map_resolution;
        mid_point.y = std::floor(cloud_world->points[i].y / map_resolution) * map_resolution + 0.5 * map_resolution;
        mid_point.z = std::floor(cloud_world->points[i].z / map_resolution) * map_resolution + 0.5 * map_resolution;

        // 如果该点所在的voxel没有点，则直接加入地图，且不需要降采样
        if (fabs(points_near[0].x - mid_point.x) > 0.5 * map_resolution &&
            fabs(points_near[0].y - mid_point.y) > 0.5 * map_resolution &&
            fabs(points_near[0].z - mid_point.z) > 0.5 * map_resolution) {
            point_no_need_downsample.push_back(cloud_world->points[i]);
            continue;
        }
        float dist = sq_dist(cloud_world->points[i], mid_point);

        for (int readd_i = 0; readd_i < 5; readd_i++) {
            // 如果该点的近邻点较少，则需要加入到地图中
            if (points_near.size() < static_cast<size_t>(5)) break;
            // 如果该点的近邻点距离voxel中心点的距离比该点距离voxel中心点更近，则不需要加入该点
            if (sq_dist(points_near[readd_i], mid_point) < dist) {
                need_add = false;
                break;
            }
        }
        if (need_add) point_to_add.push_back(cloud_world->points[i]);
    }
    // LOG_INFO("point_to_add size:{}", point_to_add.size());
    // LOG_INFO("point_no_need_downsample size:{}", point_no_need_downsample.size());
    m_ikdtree->Add_Points(point_to_add, true);
    m_ikdtree->Add_Points(point_no_need_downsample, false);
}

bool P2PlaneRegister::Align(PointCloudPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) {
    // filter cloud
    current_lidar_ = cloud_lidar;
    // auto t1 = std::chrono::high_resolution_clock::now();
    TrimCloud();
    // auto t2 = std::chrono::high_resolution_clock::now();
    // auto trim_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    // LOG_INFO("trim time:{}", trim_time * 1e3);
    kf_ptr_->UpdateLidar();
    if (updated_failed_num_ > 3) {
        return false;
    }
    return true;
}
void P2PlaneRegister::UpdateLidarFunc(State &nav_state, ESKFShareState &shared_data) {
    int size = current_lidar_->size();
    double total_res = 0;
    const State &current_state = kf_ptr_->GetState();
    PoseTrans T_WL = PoseTrans(current_state.rot, current_state.pos) * system_config_->lidar2imu_;
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < size; ++i) {
        PointType point_lidar = current_lidar_->points[i];
        const auto pt_lidar = ToV3D(point_lidar);
        auto &point_world = cloud_world->points[i];
        const auto pt_world = T_WL * pt_lidar;
        // auto pt_world_test =
        //     current_state.r_wi * (current_state.r_il * pt_lidar + current_state.t_il) + current_state.t_wi;
        // LOG_INFO("pt_world:{}, pt_world_test:{}", pt_world.transpose(), pt_world_test.transpose());
        point_world.x = pt_world[0];
        point_world.y = pt_world[1];
        point_world.z = pt_world[2];
        point_world.intensity = point_lidar.intensity;
        std::vector<float> point_sq_dist(5);
        auto &points_near = m_nearest_points[i];
        m_ikdtree->Nearest_Search(point_world, 5, points_near, point_sq_dist);
        if (points_near.size() >= 5 && point_sq_dist[4] <= 5) {
            m_point_selected_flag[i] = true;
        } else {
            m_point_selected_flag[i] = false;
            continue;
        }

        // 估计平面
        Eigen::Vector4d plane_coeff;
        m_point_selected_flag[i] = false;
        if (EstimatePlane(points_near, 0.1, plane_coeff)) {
            double pd2 = plane_coeff(0) * point_world.x + plane_coeff(1) * point_world.y +
                         plane_coeff(2) * point_world.z + plane_coeff(3);
            double s = 1 - 0.9 * std::fabs(pd2) / sqrt(pt_lidar.norm());
            if (s > 0.9) {
                m_point_selected_flag[i] = true;
                m_norm_vec->points[i].x = plane_coeff[0];
                m_norm_vec->points[i].y = plane_coeff[1];
                m_norm_vec->points[i].z = plane_coeff[2];
                m_norm_vec->points[i].intensity = pd2;
            }
        }
    }

    int effect_feat_num = 0;
    // 得到有效的点云和法向量
    for (int i = 0; i < size; i++) {
        if (!m_point_selected_flag[i]) continue;
        m_effect_cloud_lidar->points[effect_feat_num] = current_lidar_->points[i];
        m_effect_norm_vec->points[effect_feat_num] = m_norm_vec->points[i];
        effect_feat_num++;
    }
    // LOG_INFO("effect_feat_num: {}", effect_feat_num);
    if (effect_feat_num < 1) {
        shared_data.valid = false;
        updated_success = false;
        updated_failed_num_++;
        LOG_INFO("NO Effective Points!");
        return;
    }
    shared_data.valid = true;
    shared_data.H_.setZero();
    shared_data.b_.setZero();
    Eigen::Matrix<double, 1, 12> J;
    for (int i = 0; i < effect_feat_num; ++i) {
        J.Zero();
        const auto point_lidar = m_effect_cloud_lidar->points[i];
        const auto norm = m_effect_norm_vec->points[i];
        const V3D pt_lidar = ToV3D(point_lidar);
        const V3D norm_vec = ToV3D(norm);
        // 残差对旋转的雅可比矩阵
        Eigen::Matrix<double, 1, 3> dres_dr =
            -norm_vec.transpose() * current_state.rot *
            Sophus::SO3d::hat(current_state.rot_ext * pt_lidar + current_state.pos_ext);
        // 残差对平移的雅可比矩阵
        V3D dres_dt = norm_vec;
        if (system_config_->frontend_config_.calib_lidar2imu) {
        } else {
            J.block<1, 3>(0, 3) = dres_dr;
            J.block<1, 3>(0, 0) = dres_dt.transpose();
        }
        shared_data.H_ += J.transpose() * lidar_info_matrix_ * J;
        shared_data.b_ += J.transpose() * lidar_info_matrix_ * norm.intensity;
        total_res += std::fabs(norm.intensity);
        // std::cout << "H:" << shared_data.H_ << std::endl;
        // std::cout << "b:" << shared_data.b_ << std::endl;
    }
    updated_success = true;
    updated_failed_num_ = 0;
    // LOG_INFO("iter:{},effect_feat_num:{}, res:{}, aver res:{}", shared_data.iter_num, effect_feat_num, total_res,
    //          total_res / effect_feat_num);
}

bool P2PlaneRegister::EstimatePlane(const PointVec &points, double thresh, Eigen::Vector4d &plane_coeff) {
    Eigen::MatrixXd A(points.size(), 3);
    Eigen::MatrixXd b(points.size(), 1);
    A.setZero();
    b.setOnes();
    b *= -1.0;
    for (size_t i = 0; i < points.size(); i++) {
        A(i, 0) = points[i].x;
        A(i, 1) = points[i].y;
        A(i, 2) = points[i].z;
    }
    V3D normvec = A.colPivHouseholderQr().solve(b);
    double norm = normvec.norm();
    plane_coeff[0] = normvec(0) / norm;
    plane_coeff[1] = normvec(1) / norm;
    plane_coeff[2] = normvec(2) / norm;
    plane_coeff[3] = 1.0 / norm;
    for (size_t j = 0; j < points.size(); j++) {
        if (std::fabs(plane_coeff(0) * points[j].x + plane_coeff(1) * points[j].y + plane_coeff(2) * points[j].z +
                      plane_coeff(3)) > thresh) {
            return false;
        }
    }
    return true;
}
void P2PlaneRegister::UpdateMap() {
    IncreMap();
}
}  // namespace slam