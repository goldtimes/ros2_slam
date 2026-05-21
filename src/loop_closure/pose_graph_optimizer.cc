#include "loop_closure/pose_graph_optimizer.hh"
#include <pcl/filters/filter.h>

namespace slam {

PoseGraphOptimizer::PoseGraphOptimizer() {
  loop_detector_ = std::make_shared<LoopClosureDetector>();

  gtsam::ISAM2Params params;
  params.relinearizeThreshold = config_.relinearize_threshold;
  params.relinearizeSkip = config_.relinearize_skip;
  isam_ = new gtsam::ISAM2(params);

  InitNoiseModels();
  LOG_INFO("PoseGraphOptimizer init done!");
}

PoseGraphOptimizer::PoseGraphOptimizer(const PoseGraphConfig &config,
                                       const LoopClosureConfig &lc_config)
    : config_(config) {
  loop_detector_ = std::make_shared<LoopClosureDetector>(lc_config);

  gtsam::ISAM2Params params;
  params.relinearizeThreshold = config_.relinearize_threshold;
  params.relinearizeSkip = config_.relinearize_skip;
  isam_ = new gtsam::ISAM2(params);

  InitNoiseModels();
  LOG_INFO("PoseGraphOptimizer init done!");
}

PoseGraphOptimizer::~PoseGraphOptimizer() {
  if (isam_) {
    delete isam_;
    isam_ = nullptr;
  }
}

void PoseGraphOptimizer::SetConfig(const PoseGraphConfig &config) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
}

void PoseGraphOptimizer::InitNoiseModels() {
  // 先验噪声：非常小，固定第一帧
  gtsam::Vector6 prior_noise_vec;
  prior_noise_vec << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
  prior_noise_ = gtsam::noiseModel::Diagonal::Variances(prior_noise_vec);

  // 里程计噪声
  gtsam::Vector6 odom_noise_vec;
  odom_noise_vec << config_.odom_trans_noise, config_.odom_trans_noise,
      config_.odom_trans_noise, config_.odom_rot_noise, config_.odom_rot_noise,
      config_.odom_rot_noise;
  odom_noise_ = gtsam::noiseModel::Diagonal::Variances(odom_noise_vec);

  // 回环鲁棒核噪声
  gtsam::Vector6 loop_noise_vec;
  loop_noise_vec << config_.loop_noise_score, config_.loop_noise_score,
      config_.loop_noise_score, config_.loop_noise_score,
      config_.loop_noise_score, config_.loop_noise_score;
  robust_loop_noise_ = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(1),
      gtsam::noiseModel::Diagonal::Variances(loop_noise_vec));
}

gtsam::Pose3 PoseGraphOptimizer::ToGtsam(const PoseTrans &pose) const {
  V3D rpy = pose.RPY();
  return gtsam::Pose3(gtsam::Rot3::RzRyRx(rpy[0], rpy[1], rpy[2]),
                      gtsam::Point3(pose.t[0], pose.t[1], pose.t[2]));
}

PoseTrans PoseGraphOptimizer::FromGtsam(const gtsam::Pose3 &pose) const {
  return PoseTrans(pose.rotation().matrix(),
                   V3D(pose.translation().x(), pose.translation().y(),
                       pose.translation().z()));
}

void PoseGraphOptimizer::AddKeyframe(int id, const PoseTrans &pose,
                                     const PointCloudXYZIPtr &cloud,
                                     double timestamp) {
  if (!config_.enable_pgo)
    return;

  std::lock_guard<std::mutex> lock(mutex_);

  // 存储关键帧数据
  keyframe_poses_.push_back(pose);
  keyframe_poses_optimized_.push_back(pose);
  keyframe_times_.push_back(timestamp);

  // 添加关键帧到回环检测器
  loop_detector_->AddKeyframe(id, pose, cloud, timestamp);

  int curr_idx = keyframe_poses_.size() - 1;
  gtsam::Pose3 curr_pose_gtsam = ToGtsam(pose);

  if (!graph_initialized_) {
    // 首帧：添加先验因子
    AddPriorFactor(curr_idx, curr_pose_gtsam);
    graph_initialized_ = true;
    LOG_INFO("PGO: prior node {} added", curr_idx);
  } else {
    // 后续帧：添加里程计因子
    int prev_idx = curr_idx - 1;
    gtsam::Pose3 prev_pose_gtsam = ToGtsam(keyframe_poses_[prev_idx]);
    AddOdometryFactor(prev_idx, curr_idx, prev_pose_gtsam, curr_pose_gtsam);

    LOG_DEBUG("PGO: odom factor {} -> {} added", prev_idx, curr_idx);
  }

  // 定期执行回环检测
  loop_detect_counter_++;
  if (loop_detect_counter_ >= config_.loop_closure_frequency) {
    ProcessLoopClosure();
    loop_detect_counter_ = 0;
  }

  // 定期执行优化
  if (curr_idx % config_.graph_update_frequency == 0 && curr_idx > 0) {
    RunOptimization();
  }
}

void PoseGraphOptimizer::AddPriorFactor(int node_id, const gtsam::Pose3 &pose) {
  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(node_id, pose, prior_noise_));
  initial_estimate_.insert(node_id, pose);
}

void PoseGraphOptimizer::AddOdometryFactor(int prev_id, int curr_id,
                                           const gtsam::Pose3 &prev_pose,
                                           const gtsam::Pose3 &curr_pose) {
  gtsam::Pose3 delta = prev_pose.between(curr_pose);
  graph_.add(
      gtsam::BetweenFactor<gtsam::Pose3>(prev_id, curr_id, delta, odom_noise_));
  initial_estimate_.insert(curr_id, curr_pose);
}

void PoseGraphOptimizer::AddLoopFactor(int from_id, int to_id,
                                       const PoseTrans &relative_pose) {
  gtsam::Pose3 relative_gtsam = ToGtsam(relative_pose);
  graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(from_id, to_id, relative_gtsam,
                                                robust_loop_noise_));

  LoopEdge edge;
  edge.from_id = from_id;
  edge.to_id = to_id;
  edge.relative_pose = relative_pose;
  loop_edges_.push_back(edge);

  LOG_INFO(GREEN "PGO: loop factor added {} <-> {}" RESET, from_id, to_id);
}

void PoseGraphOptimizer::RunOptimization() {
  if (!graph_initialized_)
    return;

  isam_->update(graph_, initial_estimate_);
  isam_->update();

  // 多次迭代提高收敛
  for (int i = 0; i < 2; ++i) {
    isam_->update();
  }

  graph_.resize(0);
  initial_estimate_.clear();

  isam_estimate_ = isam_->calculateEstimate();
  UpdateOptimizedPoses();
}

void PoseGraphOptimizer::UpdateOptimizedPoses() {
  size_t n = isam_estimate_.size();
  for (size_t i = 0; i < n && i < keyframe_poses_optimized_.size(); ++i) {
    keyframe_poses_optimized_[i] =
        FromGtsam(isam_estimate_.at<gtsam::Pose3>(i));
  }
  recent_optimized_idx_ = static_cast<int>(n) - 1;
}

void PoseGraphOptimizer::ProcessLoopClosure() {
  LoopClosureResult result;
  if (!loop_detector_->DetectLoop(result))
    return;

  // 回环检测成功，添加回环因子
  int curr_id = keyframe_poses_.size() - 1;
  // result.match_keyframe_id 是 keyframe_db_ 中的索引
  // 需要映射到 pose 图节点 ID
  int match_id = result.match_keyframe_id;

  if (match_id < 0 || match_id >= static_cast<int>(keyframe_poses_.size())) {
    LOG_WARN("PGO: loop match id {} out of range", match_id);
    return;
  }

  AddLoopFactor(curr_id, match_id, result.relative_pose);
  LOG_INFO(GREEN "=== PGO Loop! {} <-> {}, score: {:03.4f} ===" RESET, curr_id,
           match_id, result.fitness_score);

  // 添加回环后立即优化
  RunOptimization();
}

std::vector<PoseTrans> PoseGraphOptimizer::GetOptimizedPoses() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return keyframe_poses_optimized_;
}

bool PoseGraphOptimizer::GetOptimizedPose(int id, PoseTrans &pose) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (id < 0 || id >= static_cast<int>(keyframe_poses_optimized_.size()))
    return false;
  pose = keyframe_poses_optimized_[id];
  return true;
}

void PoseGraphOptimizer::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  keyframe_poses_.clear();
  keyframe_poses_optimized_.clear();
  keyframe_times_.clear();
  loop_edges_.clear();
  recent_optimized_idx_ = 0;
  graph_initialized_ = false;

  graph_.resize(0);
  initial_estimate_.clear();

  if (isam_) {
    delete isam_;
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = config_.relinearize_threshold;
    params.relinearizeSkip = config_.relinearize_skip;
    isam_ = new gtsam::ISAM2(params);
  }

  loop_detector_->Reset();
  loop_detect_counter_ = 0;
  LOG_INFO("PoseGraphOptimizer reset!");
}

} // namespace slam
