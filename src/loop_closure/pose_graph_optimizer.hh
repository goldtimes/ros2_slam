#pragma once

#include <map>
#include <mutex>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "common/commons.hh"
#include "common/eigen_type.hh"
#include "common/logger.hh"
#include "common/pose_trans.hh"
#include "loop_closure/loop_closure_detector.hh"

namespace slam {

/**
 * @brief PGO 配置参数
 */
struct PoseGraphConfig {
  bool enable_pgo = false;             // 是否启用PGO
  double keyframe_distance = 1.0;      // 关键帧间距(m)
  double keyframe_angle = 0.2;         // 关键帧角度差(rad)
  int graph_update_frequency = 1;      // 图优化频率(帧数间隔)
  double loop_closure_frequency = 3.0; // 回环检测频率(Hz)
  // iSAM2 参数
  double relinearize_threshold = 0.01;
  int relinearize_skip = 1;
  // 噪声参数
  double odom_trans_noise = 1e-6;
  double odom_rot_noise = 1e-4;
  double loop_noise_score = 0.5;

  void print() const {
    LOG_INFO(BLUE "PoseGraphConfig:" RESET);
    LOG_INFO("  enable_pgo: {}", enable_pgo);
    LOG_INFO("  keyframe_distance: {:03.3f}", keyframe_distance);
    LOG_INFO("  keyframe_angle: {:03.3f}", keyframe_angle);
    LOG_INFO("  graph_update_frequency: {}", graph_update_frequency);
    LOG_INFO("  loop_closure_frequency: {:03.3f}", loop_closure_frequency);
    LOG_INFO("  loop_noise_score: {:03.3f}", loop_noise_score);
  }
};

/**
 * @brief 回环约束（优化后用）
 */
struct LoopEdge {
  int from_id = -1;
  int to_id = -1;
  PoseTrans relative_pose;
};

/**
 * @brief 位姿图优化器
 *
 * 基于 GTSAM iSAM2 的增量式位姿图优化。
 * 工作流程：
 *   1. 由 FrontEnd 在关键帧时调用 AddKeyframe
 *   2. 内部自动添加 prior/odometry factor
 *   3. 通过 LoopClosureDetector 检测回环
 *   4. 添加 loop closure factor 后执行 iSAM2 优化
 *   5. 获取优化后位姿
 */
class PoseGraphOptimizer {
public:
  PoseGraphOptimizer();
  explicit PoseGraphOptimizer(const PoseGraphConfig &config,
                              const LoopClosureConfig &lc_config);
  ~PoseGraphOptimizer();

  /** @brief 设置参数 */
  void SetConfig(const PoseGraphConfig &config);

  /** @brief 获取配置 */
  const PoseGraphConfig &GetConfig() const { return config_; }

  /** @brief 添加关键帧 */
  void AddKeyframe(int id, const PoseTrans &pose,
                   const PointCloudXYZIPtr &cloud, double timestamp);

  /** @brief 获取关键帧数量 */
  size_t GetKeyframeCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return keyframe_poses_.size();
  }

  /** @brief 获取优化后的关键帧位姿 */
  std::vector<PoseTrans> GetOptimizedPoses() const;

  /** @brief 获取指定ID的优化后位姿 */
  bool GetOptimizedPose(int id, PoseTrans &pose) const;

  /** @brief 获取最新优化位姿索引 */
  int GetRecentOptimizedIdx() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return recent_optimized_idx_;
  }

  /** @brief 获取闭环边信息 */
  std::vector<LoopEdge> GetLoopEdges() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return loop_edges_;
  }

  /** @brief 获取回环检测器 */
  std::shared_ptr<LoopClosureDetector> GetLoopDetector() const {
    return loop_detector_;
  }

  /** @brief 重置 */
  void Reset();

private:
  /** @brief 初始化 GTSAM 噪声模型 */
  void InitNoiseModels();

  /** @brief PoseTrans -> gtsam::Pose3 */
  gtsam::Pose3 ToGtsam(const PoseTrans &pose) const;

  /** @brief gtsam::Pose3 -> PoseTrans */
  PoseTrans FromGtsam(const gtsam::Pose3 &pose) const;

  /** @brief 添加先验因子（首个节点）*/
  void AddPriorFactor(int node_id, const gtsam::Pose3 &pose);

  /** @brief 添加里程计因子 */
  void AddOdometryFactor(int prev_id, int curr_id,
                         const gtsam::Pose3 &prev_pose,
                         const gtsam::Pose3 &curr_pose);

  /** @brief 添加回环因子 */
  void AddLoopFactor(int from_id, int to_id, const PoseTrans &relative_pose);

  /** @brief 执行 iSAM2 优化 */
  void RunOptimization();

  /** @brief 更新优化后位姿缓存 */
  void UpdateOptimizedPoses();

  /** @brief 执行回环检测与约束添加 */
  void ProcessLoopClosure();

private:
  mutable std::mutex mutex_;
  PoseGraphConfig config_;

  // ===== 关键帧数据 =====
  std::vector<PoseTrans> keyframe_poses_;           // 原始位姿(T_WL)
  std::vector<PoseTrans> keyframe_poses_optimized_; // 优化后位姿
  std::vector<double> keyframe_times_;              // 时间戳
  int recent_optimized_idx_ = 0;

  // ===== GTSAM iSAM2 =====
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimate_;
  gtsam::ISAM2 *isam_ = nullptr;
  gtsam::Values isam_estimate_;
  bool graph_initialized_ = false;

  // ===== 噪声模型 =====
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;
  gtsam::noiseModel::Base::shared_ptr robust_loop_noise_;

  // ===== 回环检测 =====
  std::shared_ptr<LoopClosureDetector> loop_detector_;
  std::vector<LoopEdge> loop_edges_;
  int loop_detect_counter_ = 0;
};

} // namespace slam
