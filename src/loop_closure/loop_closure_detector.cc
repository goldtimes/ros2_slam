#include "loop_closure/loop_closure_detector.hh"

namespace slam {

LoopClosureDetector::LoopClosureDetector() {
  voxel_grid_.setLeafSize(config_.voxel_resolution, config_.voxel_resolution,
                          config_.voxel_resolution);
  LOG_INFO("LoopClosureDetector init done!");
}

LoopClosureDetector::LoopClosureDetector(const LoopClosureConfig &config)
    : config_(config) {
  voxel_grid_.setLeafSize(config_.voxel_resolution, config_.voxel_resolution,
                          config_.voxel_resolution);
  LOG_INFO("LoopClosureDetector init done!");
}

void LoopClosureDetector::SetConfig(const LoopClosureConfig &config) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
  voxel_grid_.setLeafSize(config_.voxel_resolution, config_.voxel_resolution,
                          config_.voxel_resolution);
}

void LoopClosureDetector::AddKeyframe(int id, const PoseTrans &pose,
                                      const PointCloudXYZIPtr &cloud,
                                      double timestamp) {
  if (!config_.enable_loop_closure) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // 降采样点云以节省内存
  PointCloudXYZIPtr filtered_cloud(new PointCloudXYZI);
  if (cloud->size() > 100) {
    voxel_grid_.setInputCloud(cloud);
    voxel_grid_.filter(*filtered_cloud);
  } else {
    filtered_cloud = cloud;
  }

  KeyframeInDB kf;
  kf.id = id;
  kf.pose = pose;
  kf.cloud = filtered_cloud;
  kf.timestamp = timestamp;

  keyframe_db_.push_back(kf);

  // 限制数据库大小
  while (keyframe_db_.size() >
         static_cast<size_t>(config_.max_keyframes_in_db)) {
    keyframe_db_.erase(keyframe_db_.begin());
  }

  LOG_DEBUG("LoopClosure: Add keyframe {}, db size: {}", id,
            keyframe_db_.size());
}

bool LoopClosureDetector::DetectLoop(LoopClosureResult &result) {
  if (!config_.enable_loop_closure) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (keyframe_db_.size() <
      static_cast<size_t>(config_.min_keyframe_interval + 5)) {
    return false;
  }

  // 获取当前关键帧（数据库中的最后一个）
  current_kf_index_ = static_cast<int>(keyframe_db_.size()) - 1;
  const auto &current_kf = keyframe_db_.back();

  // 搜索候选关键帧
  auto candidate_indices = SearchCandidates(current_kf.pose);

  if (candidate_indices.empty()) {
    return false;
  }

  LOG_INFO("LoopClosure: Found {} candidates for keyframe {}",
           candidate_indices.size(), current_kf.id);

  // 遍历候选帧，用ICP验证
  double best_score = std::numeric_limits<double>::max();
  int best_candidate_idx = -1;
  PoseTrans best_relative_pose;

  for (const auto &idx : candidate_indices) {
    // 检查是否已存在相同回环
    if (IsLoopDuplicated(current_kf.id, keyframe_db_[idx].id)) {
      continue;
    }

    PoseTrans relative_pose;
    double score = 0.0;
    if (VerifyCandidate(current_kf.cloud, keyframe_db_[idx].cloud,
                        relative_pose, score)) {
      LOG_INFO("LoopClosure: Candidate {}->{} score: {:03.4f}", current_kf.id,
               keyframe_db_[idx].id, score);
      if (score < best_score) {
        best_score = score;
        best_candidate_idx = static_cast<int>(idx);
        best_relative_pose = relative_pose;
      }
    }
  }

  if (best_candidate_idx < 0 || best_score > config_.icp_score_threshold) {
    return false;
  }

  // 回环验证成功
  const auto &match_kf = keyframe_db_[best_candidate_idx];
  result.current_keyframe_id = current_kf.id;
  result.match_keyframe_id = match_kf.id;
  result.relative_pose = best_relative_pose;
  result.fitness_score = best_score;
  result.valid = true;

  // 记录回环
  detected_loops_.push_back(result);
  loop_pairs_.insert({current_kf.id, match_kf.id});
  loop_pairs_.insert({match_kf.id, current_kf.id}); // 双向记录

  LOG_INFO(GREEN "=== Loop Closure Detected! keyframe {} <-> {}, score: "
                 "{:03.4f} ===" RESET,
           current_kf.id, match_kf.id, best_score);

  return true;
}

std::vector<size_t>
LoopClosureDetector::SearchCandidates(const PoseTrans &current_pose) {
  std::vector<size_t> candidates;
  const auto &current_kf = keyframe_db_.back();

  // 如果数据库太小，无法搜索
  if (keyframe_db_.size() < 2) {
    return candidates;
  }

  // 最近N帧不参与回环检测（避免与邻近帧匹配）
  int start_check_id =
      std::max(0, current_kf_index_ - config_.min_keyframe_interval);

  // 构建候选列表：遍历数据库中所有符合条件的帧
  std::vector<std::pair<double, size_t>> scored_candidates;

  for (int i = 0; i < start_check_id; ++i) {
    const auto &kf = keyframe_db_[i];

    // 计算位置距离
    double dist = (current_kf.pose.t - kf.pose.t).norm();
    if (dist < config_.min_distance_to_keyframe ||
        dist > config_.search_radius) {
      continue;
    }

    // 计算角度差异
    double angle_diff = (current_kf.pose.R.transpose() * kf.pose.R).trace();
    angle_diff =
        std::acos(std::min(1.0, std::max(-1.0, (angle_diff - 1.0) / 2.0)));

    // 角度差太大则跳过（大于60度）
    if (angle_diff > M_PI / 3.0) {
      continue;
    }

    scored_candidates.emplace_back(dist, static_cast<size_t>(i));
  }

  // 按距离排序，取最近的几个候选
  std::sort(scored_candidates.begin(), scored_candidates.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });

  int max_candidates = std::min(config_.max_candidates,
                                static_cast<int>(scored_candidates.size()));
  for (int i = 0; i < max_candidates; ++i) {
    candidates.push_back(scored_candidates[i].second);
  }

  return candidates;
}

bool LoopClosureDetector::VerifyCandidate(
    const PointCloudXYZIPtr &current_cloud,
    const PointCloudXYZIPtr &candidate_cloud, PoseTrans &relative_pose,
    double &fitness_score) {
  if (current_cloud->size() < 50 || candidate_cloud->size() < 50) {
    LOG_WARN(
        "LoopClosure: Cloud too small for ICP verification: cur={}, cand={}",
        current_cloud->size(), candidate_cloud->size());
    return false;
  }

  // 使用GICP进行配准
  GICP gicp;
  gicp.setMaximumIterations(50);
  gicp.setTransformationEpsilon(1e-6);
  gicp.setEuclideanFitnessEpsilon(1e-6);
  gicp.setRANSACIterations(0);

  // 源点云：当前关键帧
  // 目标点云：候选关键帧
  PointCloudXYZIPtr aligned_cloud(new PointCloudXYZI);
  gicp.setInputSource(current_cloud);
  gicp.setInputTarget(candidate_cloud);
  gicp.align(*aligned_cloud);

  if (!gicp.hasConverged()) {
    LOG_DEBUG("LoopClosure: ICP did not converge");
    return false;
  }

  // 获取ICP结果
  Eigen::Matrix4f transform_mat = gicp.getFinalTransformation();
  Eigen::Matrix3f R = transform_mat.block<3, 3>(0, 0);
  Eigen::Vector3f t = transform_mat.block<3, 1>(0, 3);
  PoseTrans T_cur_to_cand(R.cast<double>(), t.cast<double>());

  // 计算匹配得分（使用fitness score）
  fitness_score = gicp.getFitnessScore();

  // 如果得分过高，认为匹配无效
  if (fitness_score > config_.icp_score_threshold) {
    LOG_DEBUG("LoopClosure: ICP score too high: {:03.4f}", fitness_score);
    return false;
  }

  relative_pose = T_cur_to_cand;
  return true;
}

bool LoopClosureDetector::IsLoopDuplicated(int cur_id, int match_id) {
  return loop_pairs_.find({cur_id, match_id}) != loop_pairs_.end();
}

void LoopClosureDetector::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  keyframe_db_.clear();
  detected_loops_.clear();
  loop_pairs_.clear();
  current_kf_index_ = -1;
  LOG_INFO("LoopClosureDetector reset!");
}

} // namespace slam
