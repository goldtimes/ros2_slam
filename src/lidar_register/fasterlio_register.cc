#include "fasterlio_register.hh"
#include "math.hh"
#include "system_config.hh"

using namespace slam;

FasterlioRegister::FasterlioRegister(
    const std::shared_ptr<SystemConfig> &system_config,
    std::shared_ptr<IESKF> kf_ptr)
    : LidarRegister(system_config, kf_ptr) {
  ivox_grid_resolution_ =
      system_config->frontend_config_.fasterlio_config.ivox_grid_resolution;
  planner_threshold_ =
      system_config->frontend_config_.fasterlio_config.planner_threshold;
  ivox_nearby_type_ =
      system_config->frontend_config_.fasterlio_config.ivox_nearby_type;

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

  // localmap init (after LoadParams)
  ivox_ = std::make_shared<IVoxType>(ivox_options_);

  submap_.reset(new PointCloudXYZI);
  kf_ptr_->SetMaxIterNum(system_config_->frontend_config_.max_iteration);
  kf_ptr_->SetLidarLossFunc([this](State &state, ESKFShareState &shared_data) {
    UpdateLidarFunc(state, shared_data);
  });
  // 设置迭代停止的条件
  kf_ptr_->SetStopFunc([&](const V33D &delta) -> bool {
    V3D rot_delta = delta.block<3, 1>(0, 0);
    V3D t_delta = delta.block<3, 1>(3, 0);
    return (rot_delta.norm() * 57.3 < 0.01) && (t_delta.norm() * 100 < 0.015);
  });
}

FasterlioRegister::~FasterlioRegister() {}

bool FasterlioRegister::InitMap(PointCloudXYZIPtr &cloud_lidar,
                                std::shared_ptr<IESKF> kf_ptr_) {
  if (first_frame_) {
    // transform cloud_lidar to world frame
    auto current_pose =
        PoseTrans(kf_ptr_->GetState().rot, kf_ptr_->GetState().pos);
    auto T_WL = current_pose * system_config_->lidar2imu_;
    auto cloud_world_tmp = TransformLidarOMP(cloud_lidar, T_WL.R, T_WL.t);
    // pcl::io::savePCDFileBinary("cloud_world_tmp.pcd", *cloud_world_tmp);
    ivox_->AddPoints(cloud_world_tmp->points);
    LOG_INFO("Build Map Size:{}, cloud  size:{}", ivox_->NumValidGrids(),
             cloud_world_tmp->size());
    first_frame_ = false;
    keyframes_.emplace_back(T_WL, cloud_world_tmp);
    {
      std::lock_guard<std::mutex> lock(local_map_mutex_);
      submap_ = cloud_world_tmp;
    }
  }
  return true;
}

bool FasterlioRegister::Align(PointCloudXYZIPtr &cloud_lidar,
                              std::shared_ptr<IESKF> kf_ptr_) {
  // filter cloud
  current_lidar_ = cloud_lidar;
  is_keyframe_ = false;
  // TrimCloud();
  kf_ptr_->UpdateLidar();

  PoseTrans curr_pose(kf_ptr_->GetState().rot, kf_ptr_->GetState().pos);
  auto T_WL = curr_pose * system_config_->lidar2imu_;
  if (keyframes_.size() > keyframe_size_) {
    keyframes_.pop_front();
  }
  PoseTrans delta_pose = last_keypose_.inverse() * curr_pose;
  if ((delta_pose.norm() > keyframe_distance_) ||
      (use_angle_keyframe_ &&
       delta_pose.RPY().norm() > keyframe_angle_distance_)) {
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

void FasterlioRegister::UpdateLidarFunc(State &nav_state,
                                        ESKFShareState &shared_data) {
  auto cur_pts = current_lidar_->size();
  residuals_.resize(cur_pts, 0);
  point_selected_surf_.resize(cur_pts, true);
  plane_coef_.resize(cur_pts, Vec4f::Zero());
  nearest_points_.resize(cur_pts);
  const State &current_state = kf_ptr_->GetState();
  PoseTrans T_WL = PoseTrans(current_state.rot, current_state.pos) *
                   system_config_->lidar2imu_;
  std::vector<size_t> index(cur_pts);
  for (size_t i = 0; i < index.size(); ++i) {
    index[i] = i;
  }
  std::for_each(std::execution::par_unseq, index.begin(), index.end(),
                [&](const size_t &i) {
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
                    point_selected_surf_[i] = math::esti_plane(
                        plane_coef_[i], points_near,
                        static_cast<float>(planner_threshold_));
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
}
void FasterlioRegister::UpdateMap() {}
PointCloudXYZIPtr FasterlioRegister::GetSubmap() {
  return PointCloudXYZIPtr(new PointCloudXYZI);
}