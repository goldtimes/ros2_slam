#pragma once
#include "common/logger.hh"
#include "common/pose_trans.hh"
#include "utils/pointcloud_utils.hh"
#include <deque>
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
#include <iostream>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include <unordered_map>
namespace slam {

struct KFPose {
  size_t index;
  PoseTrans pose;
  KFPose(size_t index, const PoseTrans &pose) : index(index), pose(pose) {}
};

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

  void run();
  void runLoopDetection();
  void runLoopConstraint();
  void runISAMUpdate();
  void runMapVisualization();

  void performRSLoopClosure();
  void visualizeLoopClosure();
  bool detectLoopClosureDistance(int &loopKeyCur, int &loopKeyPre);
  void isamUpdate();
  void updatePose();
  void publishState();
  void publishMap();

  void odomToPoseTrans(const nav_msgs::Odometry::ConstPtr &odom,
                       PoseTrans &pose);
  void addKFPoseToCloud(const PoseTrans &pose);
  gtsam::Pose3 poseTransToPose3(const PoseTrans &pose);

private:
  const std::string PGODir = "/home/kilox/catkin_ws/src/lio_slam/PGO_result/";
  ros::NodeHandle nh_;

  // tf
  tf2_ros::TransformBroadcaster tfBroadcaster;

  // 订阅雷达里程计
  ros::Subscriber lidarOdom_sub_;
  // 订阅里程计给的雷达数据
  ros::Subscriber lidarScan_sub_;
  ros::Subscriber gps_sub_;

  // 关键帧的距离
  double keyframeMeterGap;
  double keyframeDegGap;
  double keyframeRadGap;

  double keyframe_downsample = 0.2;
  // 闭环检测的参数
  // 闭环检测的距离
  double historyKeyframeSearchRadius;
  double historyKeyframeSearchTimeDiff;
  int historyKeyframeSearchNum;
  double loopNoise;
  double loopFitnessScoreThreshold;

  // 半径搜索的keyframe索引
  pcl::PointCloud<pcl::PointXYZ>::Ptr keyframePoseCloud;
  pcl::KdTreeFLANN<pcl::PointXYZ> keyframePoseKdTree;
  // 回环检测到的配对关系
  std::queue<std::pair<int, int>> loopClosureQueue;
  std::map<int, int>
      loopIndexContainer; // key是当前帧索引，value是闭环帧索引,这里用map,方便重复的帧不检测回环

  // 图优化线程的频率
  double speedFactor;

  int graphUpdateTimes; // 图优化迭代的次数
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
  // 位姿图优化线程——主线程
  std::thread posegraph_thread_;
  // 回环检测线程
  std::thread loopdetection_thread_;
  // 回环约束线程
  std::thread loopconstraint_thread_;
  // isam优化线程
  std::thread isam_update_thread_;

  // gtsam
  std::mutex mGraph;
  gtsam::ISAM2 *isam;                     // 优化器
  gtsam::NonlinearFactorGraph gtSAMgraph; // 因子图
  gtsam::Values initialEstimate;          // 初始值
  gtsam::Values isamCurrentEstimate;      // 优化后的结果
  bool gtSAMgraphMade = false;

  double recentOptimizedX = 0.0;
  double recentOptimizedY = 0.0;
  int recentIdxUpdated = 0;
  std::mutex mOptimizedPose;

  // 噪声
  gtsam::noiseModel::Diagonal::shared_ptr priorNoise;    // 先验因子噪声
  gtsam::noiseModel::Diagonal::shared_ptr odometryNoise; // 里程计因子噪声
  gtsam::noiseModel::Base::shared_ptr robustGPSNoise;    // GPS因子
  gtsam::noiseModel::Base::shared_ptr robustLoopNoise; // 回环因子鲁棒核函数

  // 存储数据
  std::mutex mBuf;                                  // 互斥锁
  std::deque<nav_msgs::Odometry::ConstPtr> odomBuf; // 雷达里程计缓冲区
  std::deque<sensor_msgs::NavSatFix::ConstPtr> gpsBuf;     //
  std::deque<sensor_msgs::PointCloud2::ConstPtr> cloudBuf; // 雷达点云缓冲区
  std::deque<double> cloudTimeBuf; // 雷达点云时间戳缓冲区
  PointCloudXYZIPtr laserCloud;
  PointCloudXYZIPtr mapCloud;
  double globalMapDownSize = 0.3;

  std::mutex mKF;
  std::deque<PointCloudXYZIPtr> keyframeCloudBuf; // 关键帧点云缓冲区
  std::vector<KFPose>
      keyframePoseIds; // 关键帧位姿缓冲区，key是关键帧索引，value是位姿
  size_t keyframeIndex = 0; // 关键帧索引

  // 优化后的位姿
  std::vector<KFPose> keyframePoseOptimized; // 关键帧优化后的位姿缓冲区
  std::deque<double> keyframeTimeBuf;        // 关键帧时间戳缓冲区
  double timeLaserOdometry = 0.0;
  double timeLaser = 0.0;
  PoseTrans lastKeyframePose;
  PoseTrans currentPose;
  double translationAccumulated;
  double rotationAccumulated;
  bool isKeyframe;
  bool use_gps;
};

} // namespace slam