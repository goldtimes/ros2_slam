#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include <pcl/search/kdtree.h>

#include "common/commons.hh"
#include "common/eigen_type.hh"
#include "common/logger.hh"
#include "common/pose_trans.hh"
#include "utils/pointcloud_utils.hh"

namespace slam {

/**
 * @brief 回环检测结果
 */
struct LoopClosureResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int current_keyframe_id = -1; // 当前关键帧ID
  int match_keyframe_id = -1;   // 匹配到的历史关键帧ID
  PoseTrans relative_pose; // 当前帧到匹配帧的相对位姿 (T_cur_match)
  double fitness_score = 0.0; // ICP匹配得分
  bool valid = false;
};

/**
 * @brief 关键帧数据库中的关键帧
 */
struct KeyframeInDB {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int id = -1;
  PoseTrans pose;          // T_WL: world到lidar的位姿
  PointCloudXYZIPtr cloud; // 降采样后的点云
  double timestamp = 0.0;
};

/**
 * @brief 回环检测参数
 */
struct LoopClosureConfig {
  bool enable_loop_closure = false; // 是否启用回环检测
  double search_radius = 10.0;      // 候选关键帧搜索半径(m)
  int min_keyframe_interval = 30;   // 最小关键帧间隔(跳过最近N帧)
  double min_distance_to_keyframe = 3.0; // 候选帧距离当前帧的最小距离
  double icp_score_threshold = 0.5; // ICP得分阈值(低于此值认为回环有效)
  double voxel_resolution = 0.5;   // 点云降采样分辨率
  int max_candidates = 5;          // 最大候选关键帧数量
  int max_keyframes_in_db = 10000; // 数据库中最大关键帧数

  void print() const {
    LOG_INFO(BLUE "LoopClosureConfig:" RESET);
    LOG_INFO("  enable_loop_closure: {}", enable_loop_closure);
    LOG_INFO("  search_radius: {:03.3f}", search_radius);
    LOG_INFO("  min_keyframe_interval: {}", min_keyframe_interval);
    LOG_INFO("  min_distance_to_keyframe: {:03.3f}", min_distance_to_keyframe);
    LOG_INFO("  icp_score_threshold: {:03.3f}", icp_score_threshold);
    LOG_INFO("  voxel_resolution: {:03.3f}", voxel_resolution);
    LOG_INFO("  max_candidates: {}", max_candidates);
    LOG_INFO("  max_keyframes_in_db: {}", max_keyframes_in_db);
  }
};

/**
 * @brief 回环检测器
 *
 * 负责维护关键帧数据库，检测回环并计算回环约束。
 * 使用距离搜索结合ICP配准的方式检测回环。
 */
class LoopClosureDetector {
public:
  using GICP = pcl::GeneralizedIterativeClosestPoint<PointXYZI, PointXYZI>;

  LoopClosureDetector();
  explicit LoopClosureDetector(const LoopClosureConfig &config);
  ~LoopClosureDetector() = default;

  /** @brief 设置回环检测参数 */
  void SetConfig(const LoopClosureConfig &config);

  /** @brief 获取回环检测参数 */
  const LoopClosureConfig &GetConfig() const { return config_; }

  /** @brief 添加关键帧到数据库 */
  void AddKeyframe(int id, const PoseTrans &pose,
                   const PointCloudXYZIPtr &cloud, double timestamp = 0.0);

  /** @brief 检测回环，返回检测结果 */
  bool DetectLoop(LoopClosureResult &result);

  /** @brief 获取所有已检测到的回环结果 */
  const std::vector<LoopClosureResult> &GetDetectedLoops() const {
    return detected_loops_;
  }

  /** @brief 获取关键帧数据库大小 */
  size_t GetKeyframeDBSize() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return keyframe_db_.size();
  }

  /** @brief 重置回环检测器 */
  void Reset();

private:
  /** @brief 搜索候选关键帧 */
  std::vector<size_t> SearchCandidates(const PoseTrans &current_pose);

  /** @brief 使用ICP验证候选关键帧 */
  bool VerifyCandidate(const PointCloudXYZIPtr &current_cloud,
                       const PointCloudXYZIPtr &candidate_cloud,
                       PoseTrans &relative_pose, double &fitness_score);

  /** @brief 检查回环是否已存在(避免重复回环) */
  bool IsLoopDuplicated(int cur_id, int match_id);

  /** @brief 构建候选关键帧的KD树 */
  void BuildKDTree();

private:
  LoopClosureConfig config_;
  std::mutex mutex_;

  // 关键帧数据库
  std::vector<KeyframeInDB> keyframe_db_;

  // 已检测到的回环
  std::vector<LoopClosureResult> detected_loops_;

  // 当前正在处理的关键帧索引
  int current_kf_index_ = -1;

  // 用于位置搜索的KD树
  pcl::KdTreeFLANN<PointXYZI> kdtree_positions_;

  // 点云降采样滤波器
  pcl::VoxelGrid<PointXYZI> voxel_grid_;

  // 最近添加的关键帧ID集合(用于去重)
  std::set<std::pair<int, int>> loop_pairs_;
};

} // namespace slam
