#include "superlio_register.hh"
#include <sys/resource.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

namespace slam {
SuperLIORegister::SuperLIORegister(
    const std::shared_ptr<SystemConfig> &system_config,
    std::shared_ptr<IESKF> kf_ptr)
    : LidarRegister(system_config, kf_ptr) {
  LOG_INFO("SuperLIORegister constructor");
  first_frame_ = true;
  float ivox_size = 0.5;
  std::size_t ivox_num = 1000000;
  ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{ivox_size, ivox_num}));
  keyframe_size_ = system_config->frontend_config_.keyframe_size;
  keyframe_distance_ = system_config->frontend_config_.keyframe_distance;
  keyframe_angle_distance_ =
      system_config->frontend_config_.keyframe_angle_distance;
  use_angle_keyframe_ = system_config->frontend_config_.use_angle_keyframe;

  // 协方差
  lidar_noise_std_ = system_config->lidar_config_.lidar_noise_std;
  // 信息矩阵
  lidar_info_matrix_ = lidar_noise_std_ == 0.0 ? 1000 : 1.0 / lidar_noise_std_;
  // 分配空间
  current_lidar_.reset(new PointCloudXYZI);
  cloud_world.reset(new PointCloudXYZI);
  submap_.reset(new PointCloudXYZI);

  if (save_map) {
    mapCloud.reset(new PointCloudXYZI);
  }

  points_world.reserve(21000);
  points_body.reserve(21000);
  abcd_vec_.resize(20000);
  effect_knn_idxs_.resize(20000);
  voxel_grid_fliter_.setLeafSize(0.2);
  // 设置雷达损失函数
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
SuperLIORegister::~SuperLIORegister() {}

bool SuperLIORegister::InitMap(PointCloudXYZIPtr &cloud_lidar,
                               std::shared_ptr<IESKF> kf_ptr_) {

  if (first_frame_) {
    // transform cloud_lidar to world frame
    auto current_pose =
        PoseTrans(kf_ptr_->GetState().rot, kf_ptr_->GetState().pos);
    auto T_WL = current_pose * system_config_->lidar2imu_;
    auto cloud_world_tmp = TransformLidarOMP(cloud_lidar, T_WL.R, T_WL.t);
    auto ptsize = cloud_world_tmp->size();
    LOG_INFO("InitMap point_size:{}", ptsize);
    points_world.resize(ptsize);
    points_body.resize(ptsize);
    // tbb 加速
    tbb::parallel_for(tbb::blocked_range<size_t>(0, ptsize),
                      // 每个线程执行这个 lambda 函数
                      [&](const tbb::blocked_range<size_t> &r) {
                        // 遍历当前线程负责的那一段点
                        for (size_t idx = r.begin(); idx < r.end(); ++idx) {
                          auto &point_world_pcl = cloud_world_tmp->points[idx];
                          Eigen::Vector3d point_world(point_world_pcl.x,
                                                      point_world_pcl.y,
                                                      point_world_pcl.z);
                          points_world[idx] = point_world;
                        }
                      });
    ivox_->insert(points_world);
    first_frame_ = false;
    keyframes_.emplace_back(T_WL, cloud_world_tmp);
    {
      std::lock_guard<std::mutex> lock(local_map_mutex_);
      submap_ = cloud_world_tmp;
    }
  }
  // const auto current_state = kf_ptr_->GetState();
  // SE3 T_WI(current_state.r_wi, current_state.t_wi);
  // V3D pos_lidar = SE3(current_state.r_wi, current_state.t_wi) *
  // system_config_->lidar2imu_.translation(); V3D pose_lidar_test =
  // T_WI.so3().matrix() * system_config_->lidar2imu_.translation() +
  // T_WI.translation(); LOG_INFO("pos_lidar: {}, pose_lidar_test: {}",
  // pos_lidar.transpose(), pose_lidar_test.transpose());
  return true;
}

bool SuperLIORegister::Align(PointCloudXYZIPtr &cloud_lidar,
                             std::shared_ptr<IESKF> kf_ptr_) {
  // filter cloud
  voxel_grid_fliter_.setInputCloud(cloud_lidar);
  voxel_grid_fliter_.filter(current_lidar_);
  is_keyframe_ = false;

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
void SuperLIORegister::UpdateLidarFunc(State &nav_state,
                                       ESKFShareState &shared_data) {
  int ptsize = current_lidar_->size();
  // 预分配空间,用来存储点的距离信息
  static std::vector<float> _lengths;
  points_body.resize(ptsize);
  _lengths.resize(ptsize);

  effect_knn_num_ = ptsize;
  std::iota(effect_knn_idxs_.begin(), effect_knn_idxs_.begin() + ptsize, 0);

  for (size_t i = 0; i < ptsize; ++i) {
    const auto &point_lidar = current_lidar_->points[i];

    points_body[i] =
        system_config_->lidar2imu_ *
        Eigen::Vector3d(point_lidar.x, point_lidar.y, point_lidar.z);
    _lengths[i] = points_body[i].norm();
  }

  ivox_->reset_max_group();

  double total_res = 0;
  const State &current_state = kf_ptr_->GetState();
  PoseTrans T_WL = PoseTrans(current_state.rot, current_state.pos) *
                   system_config_->lidar2imu_;

  tbb::enumerable_thread_specific<ThreadACC> tls_acc;
  int calc_plane_failed_num = 0;
  int compute_error_failed_num = 0;
  int not_find_top_k_num = 0;
  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, effect_knn_num_),
      [&](const tbb::blocked_range<size_t> &r) {
        KNNHeapType top_K;
        auto &local_acc = tls_acc.local();
        for (size_t r_s = r.begin(); r_s < r.end(); ++r_s) {
          int idx = effect_knn_idxs_[r_s];
          V3D &point_body = points_body[idx];
          V3D point_world =
              PoseTrans(current_state.rot, current_state.pos) * point_body;

          top_K.reset();
          ivox_->getTopK(point_world, top_K);
          if (top_K.count < 4) {
            not_find_top_k_num++;
            effect_mask_[idx] = false;
            effect_knn_mask_[idx] = false;
            continue;
          }
          effect_knn_mask_[idx] = true;
          effect_mask_[idx] =
              calc_plane_coeff(top_K.count, top_K.points_, abcd_vec_[idx]);
          if (!effect_mask_[idx]) {
            calc_plane_failed_num++;
            continue;
          }
          auto &abcd = abcd_vec_[idx];
          double error;
          effect_mask_[idx] =
              compute_error(abcd, point_world, _lengths[idx], error);
          if (!effect_mask_[idx]) {
            compute_error_failed_num++;
            continue;
          }
          {
            V3D normvec(abcd[0], abcd[1], abcd[2]);
            V3D nb = current_state.rot.transpose() * normvec;
            V3D point_body_d = point_body.cast<double>();
            V12D J;
            J.setZero();
            J.head<3>() = normvec;
            J.segment<3>(3) = point_body_d.cross(nb);

            local_acc.HTVH += J * lidar_info_matrix_ * J.transpose();
            local_acc.HTVr += J * lidar_info_matrix_ * error;
          }
        }
      });
  LOG_INFO("UpdateLidarFunc ptsize:{},calc_plane_failed_num: {}, "
           "compute_error_failed_num: {}, not_find_top_k_num: {}",
           ptsize, calc_plane_failed_num, compute_error_failed_num,
           not_find_top_k_num);
  // 合并线程的所有结果
  M12D sum_HTVH = M12D::Zero();
  V12D sum_HTVr = V12D::Zero();
  for (const auto &local_acc : tls_acc) {
    sum_HTVH += local_acc.HTVH;
    sum_HTVr += local_acc.HTVr;
  }
  shared_data.valid = true;
  shared_data.H_.setZero();
  shared_data.b_.setZero();
  shared_data.H_ = sum_HTVH.cast<double>();
  shared_data.b_ = sum_HTVr.cast<double>();

  int _effect_knn_num = 0;
  for (size_t i = 0; i < effect_knn_num_; ++i) {
    int idx = effect_knn_idxs_[i];
    if (!effect_knn_mask_[idx])
      continue;
    effect_knn_idxs_[_effect_knn_num] = idx;
    _effect_knn_num++;
  }

  LOG_INFO("effect_knn_num: {}, _effect_knn_num: {}", effect_knn_num_,
           _effect_knn_num);
  effect_knn_num_ = _effect_knn_num;
  if (effect_knn_num_ < 5) {
    shared_data.valid = false;
    updated_success = false;
    updated_failed_num_++;
    LOG_INFO("NO Effective Points!");
    return;
  }
  updated_success = true;
  updated_failed_num_ = 0;
  // LOG_INFO("iter:{},effect_feat_num:{}, res:{}, aver res:{}",
  // shared_data.iter_num, effect_feat_num, total_res,
  //          total_res / effect_feat_num);
}

void SuperLIORegister::UpdateMap() {
  const size_t ptsize = current_lidar_->size();
  if (ptsize == 0)
    return;

  const State &current_state = kf_ptr_->GetState();
  // 注意: points_body[i] 已经是 IMU 系下的点 (T_IL * p_L)
  // 直接用 IMU 位姿 T_WI 变换到世界系, 等价于 T_WL * p_L
  points_world.resize(ptsize);

  for (size_t i = 0; i < ptsize; ++i) {
    const auto &pt = points_body[i];
    points_world[i] = current_state.rot * pt + current_state.pos;
  }

  ivox_->insert(points_world);
}

PointCloudXYZIPtr SuperLIORegister::GetSubmap() {
  std::lock_guard<std::mutex> lock(local_map_mutex_);
  return submap_;
}

bool SuperLIORegister::calc_plane_coeff(const int N,
                                        const std::array<V3D, 5> &points,
                                        std::array<double, 4> &abcd) {
  Eigen::Vector3d normvec;
  if (N == 5) {
    Eigen::Matrix<double, 5, 3> A;
    Eigen::Matrix<double, 5, 1> b;
    for (int j = 0; j < 5; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  } else {
    Eigen::Matrix<double, 4, 3> A;
    Eigen::Matrix<double, 4, 1> b;

    for (int j = 0; j < N; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }

  double n = normvec.norm();
  if (n < 1e-6f)
    return false;

  abcd[3] = 1.0 / n;
  normvec *= abcd[3];
  abcd[0] = normvec[0];
  abcd[1] = normvec[1];
  abcd[2] = normvec[2];

  for (int i = 0; i < N; ++i) {
    const V3D &p = points[i];
    auto dist = abcd[0] * p(0) + abcd[1] * p(1) + abcd[2] * p(2) + abcd[3];
    if (std::abs(dist) > 0.1)
      return false;
  }
  return true;
}
bool SuperLIORegister::compute_error(const std::array<double, 4> &abcd,
                                     const V3D &point, const float length,
                                     double &error) {
  error =
      abcd[0] * point[0] + abcd[1] * point[1] + abcd[2] * point[2] + abcd[3];
  return length > 81 * error * error;
}

void SuperLIORegister::CacheData() {}
void SuperLIORegister::SaveMap() {}
} // namespace slam