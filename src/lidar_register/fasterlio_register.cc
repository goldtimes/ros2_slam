#include "fasterlio_register.hh"
#include "common/math.hh"
#include "system/system_config.hh"

using namespace slam;

FasterlioRegister::FasterlioRegister(const std::shared_ptr<SystemConfig> &system_config, std::shared_ptr<IESKF> kf_ptr)
    : LidarRegister(system_config, kf_ptr) {
    ivox_grid_resolution_ = system_config->frontend_config_.fasterlio_config.ivox_grid_resolution;
    planner_threshold_ = system_config->frontend_config_.fasterlio_config.planner_threshold;
    ivox_nearby_type_ = system_config->frontend_config_.fasterlio_config.ivox_nearby_type;

    // 关键帧参数
    keyframe_size_ = system_config->frontend_config_.keyframe_size;
    keyframe_distance_ = system_config->frontend_config_.keyframe_distance;
    keyframe_angle_distance_ = system_config->frontend_config_.keyframe_angle_distance;
    use_angle_keyframe_ = system_config->frontend_config_.use_angle_keyframe;

    if (ivox_nearby_type_ == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type_ == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type_ == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG_ERROR("Invalid ivox nearby type: {}", ivox_nearby_type_);
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    }
    cloud_world.reset(new PointCloudXYZI(10000, 1));

    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    submap_.reset(new PointCloudXYZI);
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

FasterlioRegister::~FasterlioRegister() {
}

bool FasterlioRegister::InitMap(PointCloudXYZIPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) {
    if (first_frame_) {
        // transform cloud_lidar to world frame
        auto current_pose = PoseTrans(kf_ptr_->GetState().rot, kf_ptr_->GetState().pos);
        auto T_WL = current_pose * system_config_->lidar2imu_;
        auto cloud_world_tmp = TransformLidarOMP(cloud_lidar, T_WL.R, T_WL.t);
        // pcl::io::savePCDFileBinary("cloud_world_tmp.pcd", *cloud_world_tmp);
        ivox_->AddPoints(cloud_world_tmp->points);
        LOG_INFO("Build Map Size:{}, cloud  size:{}", ivox_->NumValidGrids(), cloud_world_tmp->size());
        first_frame_ = false;
        keyframes_.emplace_back(T_WL, cloud_world_tmp);
        {
            std::lock_guard<std::mutex> lock(local_map_mutex_);
            submap_ = cloud_world_tmp;
        }
    }
    return true;
}

bool FasterlioRegister::Align(PointCloudXYZIPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) {
    // filter cloud
    current_lidar_ = cloud_lidar;
    is_keyframe_ = false;

    kf_ptr_->UpdateLidar();

    PoseTrans curr_pose(kf_ptr_->GetState().rot, kf_ptr_->GetState().pos);
    auto T_WL = curr_pose * system_config_->lidar2imu_;
    if (keyframes_.size() > keyframe_size_) {
        keyframes_.pop_front();
    }
    PoseTrans delta_pose = last_keypose_.inverse() * curr_pose;
    if ((delta_pose.norm() > keyframe_distance_) ||
        (use_angle_keyframe_ && delta_pose.RPY().norm() > keyframe_angle_distance_)) {
        is_keyframe_ = true;
        last_keypose_ = curr_pose;
        PointCloudXYZIPtr tmp_submap(new PointCloudXYZI);
        auto cloud_world_tmp = TransformLidarOMP(cloud_lidar, T_WL.R, T_WL.t);
        keyframes_.push_back({T_WL, cloud_world_tmp});

        std::lock_guard<std::mutex> lock(local_map_mutex_);
        for (auto &keyframe : keyframes_) {
            *tmp_submap += *keyframe.second;
        }
        submap_ = tmp_submap;

    } else {
        is_keyframe_ = false;
    }

    if (updated_failed_num_ > 3) {
        return false;
    }
    return true;
}

void FasterlioRegister::UpdateLidarFunc(State &nav_state, ESKFShareState &shared_data) {
    auto cur_pts = current_lidar_->size();
    total_res = 0.0;
    residuals_.resize(cur_pts, 0);
    point_selected_surf_.resize(cur_pts, true);
    plane_coef_.resize(cur_pts, Vec4f::Zero());
    nearest_points_.resize(cur_pts);
    const State &current_state = kf_ptr_->GetState();
    PoseTrans T_WL = PoseTrans(current_state.rot, current_state.pos) * system_config_->lidar2imu_;
    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
        auto &point_body = current_lidar_->points[i];
        const auto pt_lidar = ToV3D(point_body);

        PointXYZI point_world;
        auto pt_world = T_WL * pt_lidar;
        point_world.x = pt_world.x();
        point_world.y = pt_world.y();
        point_world.z = pt_world.z();
        point_world.intensity = point_body.intensity;

        auto &points_near = nearest_points_[i];

        /** Find the closest surfaces in the map **/
        // if (obs.converge_) {
        ivox_->GetClosestPoint(point_world, points_near, 5);
        point_selected_surf_[i] = points_near.size() >= 3;
        if (point_selected_surf_[i]) {
            point_selected_surf_[i] =
                math::esti_plane(plane_coef_[i], points_near, static_cast<float>(planner_threshold_));
        }

        if (point_selected_surf_[i]) {
            auto temp = point_world.getVector4fMap();
            temp[3] = 1.0;
            float pd2 = plane_coef_[i].dot(temp);

            bool valid_corr = pt_lidar.norm() > 81 * pd2 * pd2;
            if (valid_corr) {
                point_selected_surf_[i] = true;
                residuals_[i] = pd2;
            }
        }
    });
    corr_pts_.resize(cur_pts);
    corr_norm_.resize(cur_pts);
    effect_feat_num_ = 0;
    for (int i = 0; i < cur_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_num_] = plane_coef_[i];
            corr_pts_[effect_feat_num_] = current_lidar_->points[i].getVector4fMap();
            corr_pts_[effect_feat_num_][3] = residuals_[i];

            effect_feat_num_++;
        }
    }
    if (effect_feat_num_ < 1) {
        shared_data.valid = false;
        updated_failed_num_++;
        LOG_INFO("NO Effective Points!");
        return;
    }
    shared_data.valid = true;
    shared_data.H_.setZero();
    shared_data.b_.setZero();
    Eigen::Matrix<double, 1, 12> J;
    for (int i = 0; i < effect_feat_num_; ++i) {
        J.Zero();
        const auto pt_lidar = corr_pts_[i].head<3>().cast<double>();
        const auto norm_vec = corr_norm_[i].head<3>().cast<double>();

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
        shared_data.H_ += J.transpose() * 1000 * J;
        shared_data.b_ += J.transpose() * 1000 * corr_pts_[i][3];
        total_res += std::fabs(corr_pts_[i][3]);
        // std::cout << "H:" << shared_data.H_ << std::endl;
        // std::cout << "b:" << shared_data.b_ << std::endl;
    }
    updated_success = true;
    updated_failed_num_ = 0;
}
void FasterlioRegister::UpdateMap() {
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
        if (nearest_points_[i].empty()) {
            point_to_add.push_back(cloud_world->points[i]);
            continue;
        }

        const PointVec &points_near = nearest_points_[i];
        bool need_add = true;
        PointXYZI downsample_result, mid_point;
        mid_point.x = std::floor(cloud_world->points[i].x / 0.5) * 0.5 + 0.5 * 0.5;
        mid_point.y = std::floor(cloud_world->points[i].y / 0.5) * 0.5 + 0.5 * 0.5;
        mid_point.z = std::floor(cloud_world->points[i].z / 0.5) * 0.5 + 0.5 * 0.5;

        // 如果该点所在的voxel没有点，则直接加入地图，且不需要降采样
        if (fabs(points_near[0].x - mid_point.x) > 0.5 * 0.5 && fabs(points_near[0].y - mid_point.y) > 0.5 * 0.5 &&
            fabs(points_near[0].z - mid_point.z) > 0.5 * 0.5) {
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
    ivox_->AddPoints(point_to_add);
    ivox_->AddPoints(point_no_need_downsample);
}
PointCloudXYZIPtr FasterlioRegister::GetSubmap() {
    std::lock_guard<std::mutex> lock(local_map_mutex_);
    return submap_;
}