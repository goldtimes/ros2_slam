#include "PosegraphOptimization.hh"
#include <chrono>
#include <pcl_conversions/pcl_conversions.h>
using namespace slam;

PosegraphOptimization::PosegraphOptimization(ros::NodeHandle &nh) : nh_(nh) {
  // 加载参数
  nh.param<double>("keyframe_meter_gap", keyframeMeterGap,
                   2.0); // pose assignment every k m move
  nh.param<double>("keyframe_deg_gap", keyframeDegGap,
                   10.0); // pose assignment every k deg rot
  keyframeRadGap = deg2rad(keyframeDegGap);
  LOG_INFO("keyframeMeterGap: {}, keyframeDegGap: {}, keyframeRadGap: {}",
           keyframeMeterGap, keyframeDegGap, keyframeRadGap);

  //   nh.param<double>("sc_dist_thres", scDistThres, 0.2);
  //   nh.param<double>("sc_max_radius", scMaximumRadius,
  //                    80.0); // 80 is recommended for outdoor, and lower (ex,
  //                    20,
  //                           // 40) values are recommended for indoor

  // for loop closure detection
  nh.param<double>("historyKeyframeSearchRadius", historyKeyframeSearchRadius,
                   10.0);
  nh.param<double>("historyKeyframeSearchTimeDiff",
                   historyKeyframeSearchTimeDiff, 30.0);
  nh.param<int>("historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
  nh.param<double>("loopNoise", loopNoise, 0.5);
  LOG_INFO("historyKeyframeSearchRadius: {}, historyKeyframeSearchTimeDiff: "
           "{}, historyKeyframeSearchNum: {}, loopNoise: {}",
           historyKeyframeSearchRadius, historyKeyframeSearchTimeDiff,
           historyKeyframeSearchNum, loopNoise);
  nh.param<int>("graphUpdateTimes", graphUpdateTimes, 2);
  nh.param<double>("loopFitnessScoreThreshold", loopFitnessScoreThreshold, 0.3);
  LOG_INFO("graphUpdateTimes: {}, loopFitnessScoreThreshold: {}",
           graphUpdateTimes, loopFitnessScoreThreshold);
  nh.param<bool>("use_gps", use_gps, false);
  nh.param<double>("speedFactor", speedFactor, 1);
  {
    nh.param<double>("loopClosureFrequency", loopClosureFrequency, 2);
    loopClosureFrequency *= speedFactor;
    nh.param<double>("graphUpdateFrequency", graphUpdateFrequency, 1.0);
    graphUpdateFrequency *= speedFactor;
    nh.param<double>("vizmapFrequency", vizmapFrequency, 0.1);
    vizmapFrequency *= speedFactor;
    // nh.param<double>("vizPathFrequency", vizPathFrequency, 10);
    // vizPathFrequency *= speedFactor;
  }
  LOG_INFO(
      "loopClosureFrequency: {}, graphUpdateFrequency: {}, vizmapFrequency: {}",
      loopClosureFrequency, graphUpdateFrequency, vizmapFrequency);

  // 初始化gtsam参数
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam = new gtsam::ISAM2(parameters);
  // 初始化因子图噪声
  initNoise();

  // 初始化点云
  laserCloud.reset(new PointCloudXYZI());
  // 主线程启动
  posegraph_thread_ = std::thread(&PosegraphOptimization::run, this);
}

PosegraphOptimization::~PosegraphOptimization() {}

void PosegraphOptimization::initNoise() {
  // 先验因子噪声
  priorNoise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12)
          .finished()); // x,y,z,roll,pitch,yaw
  // 里程计因子噪声
  odometryNoise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6)
          .finished()); // x,y,z,roll,pitch,yaw
  // 闭环因子噪声
  gtsam::Vector robustNoiseVector6(6);
  robustNoiseVector6 << loopNoise, loopNoise, loopNoise, loopNoise, loopNoise,
      loopNoise; // x,y,z,roll,pitch,yaw
  robustLoopNoise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(
          1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but
              // Cauchy is empirically good.
      gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));
  // gps因子噪声设置的很大
  double bigNoiseTolerentToXY = 1000000000.0; // 1e9
  double gpsAltitudeNoiseScore = 250.0; // if height is misaligned after loop
                                        // clsosing, use this value bigger
  gtsam::Vector robustNoiseVector3(3);  // gps factor has 3 elements (xyz)
  robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY,
      gpsAltitudeNoiseScore; // means only caring altitude here. (because
                             // LOAM-like-methods tends to be asymptotically
                             // flyging)
  robustGPSNoise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(
          1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but
              // Cauchy is empirically good.
      gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3));
}

void PosegraphOptimization::init_subpub() {
  lidarOdom_sub_ = nh_.subscribe(
      "lidar_odom", 100, &PosegraphOptimization::laserOdomCallback, this);
  lidarScan_sub_ = nh_.subscribe("lidar_registered_cloud", 100,
                                 &PosegraphOptimization::cloudCallback, this);
  if (use_gps) {
    gps_sub_ =
        nh_.subscribe("gps", 100, &PosegraphOptimization::gspCallback, this);
    LOG_INFO("gps subscribed on topic: {}", "gps");
  }

  LOG_INFO("lidar odometry subscribed on topic: {}", "lidar_odom");
  LOG_INFO("lidar scan subscribed on topic: {}", "lidar_registered_cloud");
}

void PosegraphOptimization::laserOdomCallback(
    const nav_msgs::Odometry::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mBuf);
  odomBuf.push_back(msg);
}
void PosegraphOptimization::gspCallback(
    const sensor_msgs::NavSatFix::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mBuf);
  gpsBuf.push_back(msg);
}
void PosegraphOptimization::cloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mBuf);
  cloudBuf.push_back(msg);
}

void PosegraphOptimization::run() {
  while (ros::ok()) {
    // 确保里程计和点云数据都有了再处理
    while (!odomBuf.empty() && !cloudBuf.empty()) {
      mBuf.lock();
      // 如果里程计时间戳比点云时间戳小，说明这个里程计数据还没有对应的点云数据，丢弃这个里程计数据
      while (!odomBuf.empty() && odomBuf.front()->header.stamp.toSec() <
                                     cloudBuf.front()->header.stamp.toSec()) {
        odomBuf.pop_front();
      }
      // 如果队列空了，说明没有里程计数据了，等待下一轮循环
      if (odomBuf.empty()) {
        mBuf.unlock();
        break;
      }

      // 开始处理数据
      timeLaserOdometry = odomBuf.front()->header.stamp.toSec();
      timeLaser = cloudBuf.front()->header.stamp.toSec();
      // 判断是否为keyframe
      laserCloud->clear();
      PointCloudXYZIPtr thisKeyframe(new PointCloudXYZI());
      pcl::fromROSMsg(*cloudBuf.front(), *thisKeyframe);
      cloudBuf.pop_front();
      PoseTrans thisPose;
      odomToPoseTrans(odomBuf.front(), thisPose);
      odomBuf.pop_front();
      // TODO GPS
      mBuf.unlock();
      // 更新上一次的位姿
      lastKeyframePose = currentPose;
      currentPose = thisPose;
      PoseTrans deltaPose = lastKeyframePose.inverse() * currentPose;
      translationAccumulated += deltaPose.t.norm();
      rotationAccumulated += deltaPose.RPY().norm();
      if (translationAccumulated >= keyframeMeterGap ||
          rotationAccumulated >= keyframeRadGap) {
        // 满足关键帧条件，构建里程计因子图
        translationAccumulated = 0.0;
        rotationAccumulated = 0.0;
        isKeyframe = true;
      } else {
        isKeyframe = false;
        continue;
      }
      // 存储关键点云和位姿
      // 构建里程计因子图
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void PosegraphOptimization::odomToPoseTrans(
    const nav_msgs::Odometry::ConstPtr &odom, PoseTrans &pose) {
  auto tx = odom->pose.pose.position.x;
  auto ty = odom->pose.pose.position.y;
  auto tz = odom->pose.pose.position.z;
  Eigen::Quaterniond quat(
      odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
  quat.normalize();
  pose = PoseTrans(quat.toRotationMatrix(), Eigen::Vector3d(tx, ty, tz));
}