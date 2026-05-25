#pragma once
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>

namespace slam {
class PosegraphOptimization {
public:
  PosegraphOptimization(ros::NodeHandle &nh);
  ~PosegraphOptimization();

private:
  int deg2rad(double deg) { return deg * M_PI / 180.0; }
  void initNoise();
  void init_subpub();
  void laserOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void gspCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

private:
  ros::NodeHandle nh_;

  // 订阅雷达里程计
  ros::Subscriber lidarOdom_sub_;
  // 订阅里程计给的雷达数据
  ros::Subscriber lidarScan_sub_;
  ros::Subscriber gps_sub_;

  // 关键帧的距离
  double keyframeMeterGap;
  double keyframeDegGap;
  double keyframeRadGap;
  // 闭环检测的参数
  // 闭环检测的距离
  double historyKeyframeSearchRadius;
  double historyKeyframeSearchTimeDiff;
  int historyKeyframeSearchNum;
  double loopNoise;
  double loopFitnessScoreThreshold;

  // 图优化线程的频率
  double speedFactor;

  int graphUpdateTimes;
  double graphUpdateFrequency;
  double loopClosureFrequency;
  double vizmapFrequency;

  // keyframe发布
  ros::Publisher keyframe_pub_;

  // 闭环检测的边
  ros::Publisher pubLoopConstraintEdge;
  ros::Publisher pubLoopScanLocalRegisted;

  // 优化后的路径
  ros::Publisher pubPathAftPGO;
  // 优化后的里程计
  ros::Publisher pubOdomAftPGO;
  // 优化后的地图
  ros::Publisher pubMapAftPGO;

  // 地图可视化线程
  std::thread map_visualization_thread_;
  // 位姿图优化线程
  std::thread posegraph_thread_;
  // 回环检测线程
  std::thread loopdetection_thread_;
  // isam优化线程
  std::thread isam_update_thread_;

  // gtsam

  gtsam::ISAM2 *isam;                     // 优化器
  gtsam::NonlinearFactorGraph gtSAMgraph; // 因子图
  gtsam::Values initialEstimate;          // 初始值
  gtsam::Values isamCurrentEstimate;      // 优化后的结果
  bool gtSAMgraphMade = false;

  // 噪声
  gtsam::noiseModel::Diagonal::shared_ptr priorNoise;    // 先验因子噪声
  gtsam::noiseModel::Diagonal::shared_ptr odometryNoise; // 里程计因子噪声
  gtsam::noiseModel::Base::shared_ptr robustGPSNoise;    // GPS因子
  gtsam::noiseModel::Base::shared_ptr robustLoopNoise; // 回环因子鲁棒核函数
};
} // namespace slam