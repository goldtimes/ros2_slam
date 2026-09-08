#include "PosegraphOptimization.hh"
#include "utils/global_map_split.hh"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
using namespace slam;

namespace {
// 将 double 秒写入消息头时间戳(ROS1: ros::Time / ROS2: builtin_interfaces::msg::Time)
#if ROS_AVAILABLE == 1
inline void PgoSetStamp(ros::Time &t, double sec) {
    t = ros::Time(sec);
}
inline double PgoNow() {
    return ros::Time::now().toSec();
}
#else
inline void PgoSetStamp(builtin_interfaces::msg::Time &t, double sec) {
    int32_t s = static_cast<int32_t>(std::floor(sec));
    int64_t ns = static_cast<int64_t>((sec - static_cast<double>(s)) * 1e9);
    t.sec = s;
    t.nanosec = static_cast<uint32_t>(ns);
}
inline double PgoNow() {
    return rclcpp::Clock().now().seconds();
}
#endif
}  // namespace

#if ROS_AVAILABLE == 1
PosegraphOptimization::PosegraphOptimization(ros::NodeHandle &nh) : nh_(nh) {
#else
PosegraphOptimization::PosegraphOptimization(const rclcpp::Node::SharedPtr &node) : nh_(node) {
#endif
    // PGO 结果目录:ROS1 默认包路径;ROS2 默认 share 目录;均可由参数 pgo_result_dir 覆盖
#if ROS_AVAILABLE == 1
    std::string default_pgo_dir = ros::package::getPath("lio_slam") + "/PGO_result/";
#else
    std::string default_pgo_dir;
    try {
        default_pgo_dir = ament_index_cpp::get_package_share_directory("lio_slam") + "/PGO_result/";
    } catch (const std::exception &) {
        default_pgo_dir = "/tmp/lio_slam_PGO_result/";
    }
#endif
    GetParam<std::string>("pgo_result_dir", PGODir, default_pgo_dir);

    // 加载参数
    GetParam<double>("keyframe_meter_gap", keyframeMeterGap,
                     1.0);  // pose assignment every k m move
    GetParam<double>("keyframe_deg_gap", keyframeDegGap,
                     30.0);  // pose assignment every k deg rot
    keyframeRadGap = deg2rad(keyframeDegGap);
    LOG_INFO("keyframeMeterGap: {}, keyframeDegGap: {}, keyframeRadGap: {}", keyframeMeterGap, keyframeDegGap,
             keyframeRadGap);

    // 2. 每次启动清空目录（安全删除所有文件，不删文件夹）
    try {
        if (std::filesystem::exists(PGODir)) {
            // 遍历删除所有文件
            for (const auto &entry : std::filesystem::directory_iterator(PGODir)) {
                std::filesystem::remove_all(entry.path());
            };
            LOG_INFO("PGODir cleared: {}", PGODir);
        } else {
            // 不存在则创建
            std::filesystem::create_directories(PGODir);
            LOG_INFO("PGODir created: {}", PGODir);
        }
    } catch (const std::filesystem::filesystem_error &e) {
        LOG_WARN("PGODir clear failed: {}", e.what());
    }

    // for loop closure detection
    GetParam<double>("historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
    GetParam<double>("historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
    GetParam<int>("historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
    GetParam<double>("loopNoise", loopNoise, 0.5);
    LOG_INFO(
        "historyKeyframeSearchRadius: {}, historyKeyframeSearchTimeDiff: "
        "{}, historyKeyframeSearchNum: {}, loopNoise: {}",
        historyKeyframeSearchRadius, historyKeyframeSearchTimeDiff, historyKeyframeSearchNum, loopNoise);
    GetParam<int>("graphUpdateTimes", graphUpdateTimes, 2);
    GetParam<double>("loopFitnessScoreThreshold", loopFitnessScoreThreshold, 0.3);
    LOG_INFO("graphUpdateTimes: {}, loopFitnessScoreThreshold: {}", graphUpdateTimes, loopFitnessScoreThreshold);
    GetParam<bool>("use_gps", use_gps, false);
    GetParam<double>("speedFactor", speedFactor, 1);
    {
        GetParam<double>("loopClosureFrequency", loopClosureFrequency, 2);
        loopClosureFrequency *= speedFactor;
        GetParam<double>("graphUpdateFrequency", graphUpdateFrequency, 1.0);
        graphUpdateFrequency *= speedFactor;
        GetParam<double>("vizmapFrequency", vizmapFrequency, 0.1);
        vizmapFrequency *= speedFactor;
    }
    LOG_INFO("loopClosureFrequency: {}, graphUpdateFrequency: {}, vizmapFrequency: {}", loopClosureFrequency,
             graphUpdateFrequency, vizmapFrequency);

    // 保存地图参数
    GetParam<bool>("map_save_enable", map_save_enable_, false);
    GetParam<double>("map_save_idle_sec", map_save_idle_sec_, 10.0);
    GetParam<double>("map_save_voxel", map_save_voxel_, 0.1);
    GetParam<std::string>("map_save_dir", map_save_dir_, std::string("/home/li/ros2_ws/maps"));
    GetParam<std::string>("map_save_name", map_save_name_, std::string("pgo_optimized_map"));
    GetParam<bool>("map_save_tile_enable", map_save_tile_enable_, true);
    GetParam<double>("map_save_tile_size", map_save_tile_size_, 50.0);
    GetParam<std::string>("map_save_leaf", map_save_leaf_, std::string("MAP_GLOBAL"));
    GetParam<bool>("map_save_keyframes", map_save_keyframes_, true);
    LOG_INFO("map_save_enable: {}, map_save_idle_sec: {}, map_save_voxel: {}, map_save_dir: {}, map_save_name: {}",
             map_save_enable_, map_save_idle_sec_, map_save_voxel_, map_save_dir_, map_save_name_);
    LOG_INFO("map_save_tile_enable: {}, map_save_tile_size: {}, map_save_leaf: {}, map_save_keyframes: {}",
             map_save_tile_enable_, map_save_tile_size_, map_save_leaf_, map_save_keyframes_);

    // 初始化gtsam参数
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);
    // 初始化因子图噪声
    initNoise();
    init_subpub();

#if ROS_AVAILABLE == 2
    // 保存地图服务: ros2 service call /pgo_node/save_map lio_slam/srv/SaveMap "{map_name: 'xxx'}"
    save_map_srv_ = nh_->create_service<lio_slam::srv::SaveMap>(
        "~/save_map", [this](const std::shared_ptr<lio_slam::srv::SaveMap::Request> req,
                              std::shared_ptr<lio_slam::srv::SaveMap::Response> res) {
            std::string name = req->map_name;
            // 过滤不安全字符, 防止路径逃逸
            for (auto &ch : name) {
                if (!std::isalnum(static_cast<unsigned char>(ch)) && ch != '_' && ch != '-' && ch != '.') {
                    ch = '_';
                }
            }
            if (name.empty()) {
                res->success = false;
                res->message = "invalid/empty map name";
                return;
            }
            const bool ok = SaveMapByName(name);
            res->success = ok;
            res->message = (ok ? (map_save_dir_ + "/" + name) : std::string("save failed"));
        });
    LOG_INFO("save_map service ready: /pgo_node/save_map (map_save_dir={})", map_save_dir_);
#endif

    // 初始化点云
    laserCloud.reset(new PointCloudXYZI());
    // 存储位姿的点云，半径搜索用
    keyframePoseCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    // 存储地图的点云，发布用
    mapCloud.reset(new PointCloudXYZI());

    // 主线程启动
    posegraph_thread_ = std::thread(&PosegraphOptimization::run, this);
    // 回环检测线程启动
    loopdetection_thread_ = std::thread(&PosegraphOptimization::runLoopDetection, this);
    // 回环约束线程启动
    loopconstraint_thread_ = std::thread(&PosegraphOptimization::runLoopConstraint, this);
    // isam优化线程启动
    isam_update_thread_ = std::thread(&PosegraphOptimization::runISAMUpdate, this);
    // 地图可视化线程启动
    map_visualization_thread_ = std::thread(&PosegraphOptimization::runMapVisualization, this);
}

PosegraphOptimization::~PosegraphOptimization() {
}

void PosegraphOptimization::initNoise() {
    // 先验因子噪声
    priorNoise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());  // x,y,z,roll,pitch,yaw
    // 里程计因子噪声
    odometryNoise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());  // x,y,z,roll,pitch,yaw
    // 闭环因子噪声
    gtsam::Vector robustNoiseVector6(6);
    robustNoiseVector6 << loopNoise, loopNoise, loopNoise, loopNoise, loopNoise,
        loopNoise;  // x,y,z,roll,pitch,yaw
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),  // optional: replacing Cauchy by DCS or GemanMcClure is okay
                                                           // but Cauchy is empirically good.
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));
    // gps因子噪声设置的很大
    double bigNoiseTolerentToXY = 1000000000.0;  // 1e9
    double gpsAltitudeNoiseScore = 250.0;        // if height is misaligned after loop
                                                 // clsosing, use this value bigger
    gtsam::Vector robustNoiseVector3(3);         // gps factor has 3 elements (xyz)
    robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY,
        gpsAltitudeNoiseScore;  // means only caring altitude here. (because
                                // LOAM-like-methods tends to be asymptotically
                                // flyging)
    robustGPSNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),  // optional: replacing Cauchy by DCS or GemanMcClure is okay
                                                           // but Cauchy is empirically good.
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3));
}

void PosegraphOptimization::init_subpub() {
#if ROS_AVAILABLE == 1
    lidarOdom_sub_ = nh_.subscribe("/lidar_odom", 100, &PosegraphOptimization::laserOdomCallback, this);
    lidarScan_sub_ = nh_.subscribe("/lidar_registered_body", 100, &PosegraphOptimization::cloudCallback, this);

    LOG_INFO("subscribed on topic: {}", lidarOdom_sub_.getTopic());
    LOG_INFO("subscribed on topic: {}", lidarScan_sub_.getTopic());
    if (use_gps) {
        gps_sub_ = nh_.subscribe("gps", 100, &PosegraphOptimization::gspCallback, this);
        LOG_INFO("gps subscribed on topic: {}", "gps");
    }

    pubLoopScanLocal = nh_.advertise<CloudMsg>("loop_scan_local", 100);
    pubLoopSubmapLocal = nh_.advertise<CloudMsg>("loop_submap_local", 100);
    pubLoopScanLocalRegisted = nh_.advertise<CloudMsg>("loop_scan_local_registed", 100);

    // 发布路径
    pubPathAftPGO = nh_.advertise<PathMsg>("aft_pgo_path", 100);
    pubOdomAftPGO = nh_.advertise<OdomMsg>("aft_pgo_odom", 100);
    pubMapAftPGO = nh_.advertise<CloudMsg>("aft_pgo_map", 100);

    pubLoopConstraintEdge = nh_.advertise<MarkerArrayMsg>("loop_closure_edges", 100);
#else
    // 节点私有话题,便于 launch remap
    lidarOdom_sub_ = nh_->create_subscription<OdomMsg>(
        "~/lidar_odom", rclcpp::SensorDataQoS(),
        std::bind(&PosegraphOptimization::laserOdomCallback, this, std::placeholders::_1));
    lidarScan_sub_ = nh_->create_subscription<CloudMsg>(
        "~/lidar_registered_body", rclcpp::SensorDataQoS(),
        std::bind(&PosegraphOptimization::cloudCallback, this, std::placeholders::_1));
    if (use_gps) {
        gps_sub_ = nh_->create_subscription<NavSatMsg>(
            "~/gps", rclcpp::SensorDataQoS(),
            std::bind(&PosegraphOptimization::gspCallback, this, std::placeholders::_1));
    }

    pubLoopScanLocal = nh_->create_publisher<CloudMsg>("~/loop_scan_local", 10);
    pubLoopSubmapLocal = nh_->create_publisher<CloudMsg>("~/loop_submap_local", 10);
    pubLoopScanLocalRegisted = nh_->create_publisher<CloudMsg>("~/loop_scan_local_registed", 10);

    pubPathAftPGO = nh_->create_publisher<PathMsg>("~/aft_pgo_path", 10);
    pubOdomAftPGO = nh_->create_publisher<OdomMsg>("~/aft_pgo_odom", 10);
    pubMapAftPGO = nh_->create_publisher<CloudMsg>("~/aft_pgo_map", 10);

    pubLoopConstraintEdge = nh_->create_publisher<MarkerArrayMsg>("~/loop_closure_edges", 10);

    tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
#endif
    LOG_INFO("lidar odometry subscribed on topic: {}", "lidar_odom");
    LOG_INFO("lidar scan subscribed on topic: {}", "lidar_registered_cloud");
}

#if ROS_AVAILABLE == 1
void PosegraphOptimization::laserOdomCallback(const OdomPtr &msg) {
    std::lock_guard<std::mutex> lock(mBuf);
    odomBuf.push_back(msg);
}
void PosegraphOptimization::gspCallback(const NavSatPtr &msg) {
    std::lock_guard<std::mutex> lock(mBuf);
    gpsBuf.push_back(msg);
}
void PosegraphOptimization::cloudCallback(const CloudPtr &msg) {
    std::lock_guard<std::mutex> lock(mBuf);
    cloudBuf.push_back(msg);
}
#else
void PosegraphOptimization::laserOdomCallback(const OdomMsg::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mBuf);
    odomBuf.push_back(msg);
}
void PosegraphOptimization::gspCallback(const NavSatMsg::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mBuf);
    gpsBuf.push_back(msg);
}
void PosegraphOptimization::cloudCallback(const CloudMsg::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mBuf);
    cloudBuf.push_back(msg);
}
#endif

void PosegraphOptimization::run() {
    while (slam::RosOk()) {
        // 确保里程计和点云数据都有了再处理
        while (!odomBuf.empty() && !cloudBuf.empty()) {
            mBuf.lock();
            // 如果里程计时间戳比点云时间戳小，说明这个里程计数据还没有对应的点云数据，丢弃这个里程计数据
            while (!odomBuf.empty() &&
                   slam::StampToSec(odomBuf.front()->header.stamp) < slam::StampToSec(cloudBuf.front()->header.stamp)) {
                odomBuf.pop_front();
            }
            // 如果队列空了，说明没有里程计数据了，等待下一轮循环
            if (odomBuf.empty()) {
                mBuf.unlock();
                break;
            }

            // 开始处理数据
            timeLaserOdometry = slam::StampToSec(odomBuf.front()->header.stamp);
            timeLaser = slam::StampToSec(cloudBuf.front()->header.stamp);
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
            if (translationAccumulated >= keyframeMeterGap || rotationAccumulated >= keyframeRadGap) {
                LOG_INFO("keyframe: {}, translation: {}, rotation: {}", keyframeIndex, translationAccumulated,
                         rotationAccumulated);
                // 满足关键帧条件，构建里程计因子图
                translationAccumulated = 0.0;
                rotationAccumulated = 0.0;
                isKeyframe = true;

            } else {
                isKeyframe = false;
                continue;
            }
            // 存储关键点云和位姿
            // 对关键点云进行滤波
            PointCloudXYZIPtr filteredKeyframe = VoxelFilter(thisKeyframe, keyframe_downsample);
            // 加锁
            mKF.lock();
            keyframeCloudBuf.push_back(filteredKeyframe);
            keyframePoseIds.push_back(KFPose(keyframeIndex, thisPose));
            // TODO发布关键帧点云
            keyframeTimeBuf.push_back(timeLaserOdometry);
            keyframePoseOptimized.push_back(KFPose(keyframeIndex, thisPose));
            // 添加到关键帧位姿点云
            addKFPoseToCloud(thisPose);
            keyframeIndex++;
            // TODO ScanContext
            mKF.unlock();
            // 构建里程计因子图
            const int prev_node_idx = keyframePoseIds.size() - 2;
            const int curr_node_idx = keyframePoseIds.size() - 1;
            if (!gtSAMgraphMade) {
                // 第一帧
                const int init_node_idx = 0;
                auto init_pose = keyframePoseIds.at(init_node_idx).pose;
                gtsam::Pose3 poseOrigin = poseTransToPose3(init_pose);
                mGraph.lock();
                gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
                initialEstimate.insert(init_node_idx, poseOrigin);
                mGraph.unlock();
                gtSAMgraphMade = true;
                LOG_INFO("First keyframe added to graph with index: {}", init_node_idx);
            } else {
                // 之后的帧 确保 > 2
                auto prev_pose = keyframePoseIds.at(prev_node_idx).pose;
                auto curr_pose = keyframePoseIds.at(curr_node_idx).pose;
                gtsam::Pose3 posePrev = poseTransToPose3(prev_pose);
                gtsam::Pose3 poseCurr = poseTransToPose3(curr_pose);
                gtsam::Pose3 relativePose = posePrev.between(poseCurr);
                mGraph.lock();
                gtSAMgraph.add(
                    gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relativePose, odometryNoise));
                initialEstimate.insert(curr_node_idx, poseCurr);
                mGraph.unlock();
                LOG_INFO("Keyframe added to graph with index: {}", curr_node_idx);
            }
            // save 点云
            std::string current_node_str = std::to_string(curr_node_idx);
            std::string current_node_cloud_path = PGODir + current_node_str + ".pcd";
            pcl::io::savePCDFile(current_node_cloud_path, *filteredKeyframe);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

void PosegraphOptimization::runLoopDetection() {
    slam::Rate rate(loopClosureFrequency);
    while (slam::RosOk()) {
        rate.sleep();
        // 基于距离的回环检测
        performRSLoopClosure();
        visualizeLoopClosure();
    }
}

void PosegraphOptimization::runLoopConstraint() {
    while (slam::RosOk()) {
        // ICP确认,确保回环候选队列不为空
        while (!loopClosureQueue.empty()) {
            if (loopClosureQueue.size() > 30) {
                LOG_WARN(
                    "Too many loop clousre candidates to be ICPed is waiting ... "
                    "Do process_lcd less frequently "
                    "(adjust loopClosureFrequency)");
            }
            mBuf.lock();
            auto loopPair = loopClosureQueue.front();
            loopClosureQueue.pop();
            mBuf.unlock();
            auto prev_node_idx = loopPair.first;
            auto curr_node_idx = loopPair.second;
            auto relativePose = doICPVirtualRelative(prev_node_idx, curr_node_idx);
            gtsam::Pose3 relativePose3 = poseTransToPose3(relativePose);
            // 注意: doICPVirtualRelative 在 ICP 失败时返回 PoseTrans()(恒等位姿)作为失败哨兵。
            // 只有 ICP 成功(相对位姿非恒等)才添加回环约束; 旧逻辑"恒等才加"是反的,
            // 会把失败的 ICP 当作恒等回环硬塞进图, 与里程计链严重冲突, 导致 ISAM2
            // 线性系统奇异而抛 IndeterminantLinearSystemException。
            if (!relativePose3.equals(gtsam::Pose3::Identity())) {
                mGraph.lock();
                gtSAMgraph.add(
                    gtsam::BetweenFactor<gtsam::Pose3>(curr_node_idx, prev_node_idx, relativePose3, robustLoopNoise));
                mGraph.unlock();
                LOG_INFO("Add loop edge between keyframes {} and {}", curr_node_idx, prev_node_idx);
            } else {
                LOG_WARN("Skip loop edge: ICP failed (relative pose == identity) between {} and {}", curr_node_idx,
                         prev_node_idx);
            }
            std::chrono::milliseconds dura(2);
            std::this_thread::sleep_for(dura);
        }
    }
}

// icp 计算相对位姿
PoseTrans PosegraphOptimization::doICPVirtualRelative(int loopKeyPre, int loopKeyCur) {
    PointCloudXYZIPtr currCloud(new PointCloudXYZI());
    PointCloudXYZIPtr targetCloud(new PointCloudXYZI());

    loopFindNearKeyframe(currCloud, loopKeyCur, 5);
    loopFindNearKeyframe(targetCloud, loopKeyPre, historyKeyframeSearchNum);

    // 发布当前帧和目标帧的点云信息
    publishCloud(pubLoopScanLocal, currCloud);
    publishCloud(pubLoopSubmapLocal, targetCloud);
    // ICP Settings
    pcl::IterativeClosestPoint<PointXYZI, PointXYZI> icp;
    icp.setMaxCorrespondenceDistance(150);  // giseop , use a value can cover 2*historyKeyframeSearchNum range
                                            // in meter
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align pointclouds
    icp.setInputSource(currCloud);
    icp.setInputTarget(targetCloud);
    pcl::PointCloud<PointXYZI>::Ptr unused_result(new pcl::PointCloud<PointXYZI>());
    icp.align(*unused_result);
    publishCloud(pubLoopScanLocalRegisted, unused_result);

    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold) {
        LOG_INFO("[SC loop] ICP fitness test failed ({} > {}). Reject this SC loop.", icp.getFitnessScore(),
                 loopFitnessScoreThreshold);
        return PoseTrans();
    } else {
        LOG_INFO("[SC loop] ICP fitness test passed ({} < {}). Add this SC loop.", icp.getFitnessScore(),
                 loopFitnessScoreThreshold);
    }
    auto icp_transform = icp.getFinalTransformation();
    auto opt_before_pose = keyframePoseIds.at(loopKeyCur);
    auto opt_after_pose =
        PoseTrans(icp_transform.block<3, 3>(0, 0).cast<double>(), icp_transform.block<3, 1>(0, 3).cast<double>()) *
        opt_before_pose.pose;
    auto poseTo = keyframePoseIds.at(loopKeyPre);
    return opt_after_pose.inverse() * poseTo.pose;
}

// 根据当前传入的id, 搜索附近的点云组合起来
void PosegraphOptimization::loopFindNearKeyframe(PointCloudXYZIPtr &nearKeyframes, const int key,
                                                 const int search_num) {
    nearKeyframes->clear();
    int cloud_size = keyframeCloudBuf.size();
    for (int i = -search_num; i <= search_num; ++i) {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloud_size) {
            continue;
        }
        mKF.lock();
        auto kfcloud_inLidar = keyframeCloudBuf.at(keyNear);
        auto kfPose = keyframePoseIds.at(keyNear);
        PointCloudXYZIPtr kfcloud_inMap(new PointCloudXYZI());
        TransformCloud(kfcloud_inLidar, kfcloud_inMap, kfPose.pose.R, kfPose.pose.t);
        *nearKeyframes += *kfcloud_inMap;
        mKF.unlock();
    }
    if (nearKeyframes->empty()) {
        return;
    }
    // 降采样
    // nearKeyframes = VoxelFilter(nearKeyframes, 0.3);
}

void PosegraphOptimization::runISAMUpdate() {
    slam::Rate rate(graphUpdateFrequency);
    size_t last_kf_count = 0;
    int idle_cycles = 0;
    while (slam::RosOk()) {
        rate.sleep();

        if (gtSAMgraphMade) {
            mGraph.lock();
            isamUpdate();
            mGraph.unlock();
            // TODO save pose
        }

        // 自动保存优化后地图: 输入(关键帧)停止增长一段时间后保存一次;
        // 若之后又有新关键帧(比如重新播放 bag), 则允许再次保存。
        if (map_save_enable_) {
            size_t kf_count = 0;
            {
                std::lock_guard<std::mutex> lock(mKF);
                kf_count = keyframePoseIds.size();
            }
            if (kf_count > 1) {
                if (kf_count == last_kf_count) {
                    idle_cycles++;
                } else {
                    idle_cycles = 0;
                    auto_map_saved_ = false;  // 有新的关键帧进来, 重新武装
                }
                last_kf_count = kf_count;
                if (!auto_map_saved_ && idle_cycles >= static_cast<int>(map_save_idle_sec_ * graphUpdateFrequency)) {
                    auto_map_saved_ = true;
                    LOG_INFO("PGO input idle {}s, saving optimized map '{}' ...", idle_cycles / graphUpdateFrequency,
                             map_save_name_);
                    SaveMapByName(map_save_name_);
                }
            } else {
                last_kf_count = kf_count;
            }
        }
    }
}

void PosegraphOptimization::isamUpdate() {
    // 优化
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    for (int i = graphUpdateTimes; i > 0; i--) {
        isam->update();
    }
    // 清空图优化
    gtSAMgraph.resize(0);
    initialEstimate.clear();
    // 获取优化的结果
    isamCurrentEstimate = isam->calculateEstimate();
    // updatePose
    updatePose();
    // publish 路径，里程计以及tf信息
    publishState();
}

void PosegraphOptimization::updatePose() {
    mKF.lock();
    for (int node_idx = 0; node_idx < static_cast<int>(isamCurrentEstimate.size()); node_idx++) {
        KFPose &kfPose = keyframePoseOptimized.at(node_idx);
        gtsam::Pose3 optimizedPose = isamCurrentEstimate.at<gtsam::Pose3>(node_idx);
        Eigen::Vector3d t = optimizedPose.translation();
        Eigen::Matrix3d R = optimizedPose.rotation().matrix();
        kfPose.pose = PoseTrans(R, t);
    }
    mKF.unlock();
    mOptimizedPose.lock();
    const gtsam::Pose3 &latestOptimizedPose =
        isamCurrentEstimate.at<gtsam::Pose3>(static_cast<int>(isamCurrentEstimate.size()) - 1);

    recentOptimizedX = latestOptimizedPose.translation().x();
    recentOptimizedY = latestOptimizedPose.translation().y();
    recentIdxUpdated = static_cast<int>(isamCurrentEstimate.size()) - 1;
    mOptimizedPose.unlock();
}

void PosegraphOptimization::publishState() {
    PathMsg pathMsg;
    OdomMsg odomMsg;
    pathMsg.header.frame_id = "map";
    mKF.lock();
    for (int i = 0; i < recentIdxUpdated; i++) {
        const KFPose &kfPose = keyframePoseOptimized.at(i);
        OdomMsg odom;
        odom.header.frame_id = "map";
        odom.child_frame_id = "aft_pgo";
        PgoSetStamp(odom.header.stamp, keyframeTimeBuf.at(i));
        odom.pose.pose.position.x = kfPose.pose.t.x();
        odom.pose.pose.position.y = kfPose.pose.t.y();
        odom.pose.pose.position.z = kfPose.pose.t.z();
        Eigen::Quaterniond q(kfPose.pose.R);
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odomMsg = odom;
        PoseStampedMsg poseStamped;
        poseStamped.header = odom.header;
        poseStamped.pose = odom.pose.pose;
        PgoSetStamp(pathMsg.header.stamp, keyframeTimeBuf.at(i));
        pathMsg.header.frame_id = "map";
        pathMsg.poses.push_back(poseStamped);
#if ROS_AVAILABLE == 1
        pubPathAftPGO.publish(pathMsg);
#else
        pubPathAftPGO->publish(pathMsg);
#endif
    }
    mKF.unlock();
    TransformMsg tfMsg;
    tfMsg.header.stamp = odomMsg.header.stamp;
    tfMsg.header.frame_id = "map";
    tfMsg.child_frame_id = "aft_pgo";
    tfMsg.transform.translation.x = odomMsg.pose.pose.position.x;
    tfMsg.transform.translation.y = odomMsg.pose.pose.position.y;
    tfMsg.transform.translation.z = odomMsg.pose.pose.position.z;
    tfMsg.transform.rotation = odomMsg.pose.pose.orientation;
#if ROS_AVAILABLE == 1
    tfBroadcaster.sendTransform(tfMsg);
#else
    tfBroadcaster->sendTransform(tfMsg);
#endif
}

void PosegraphOptimization::performRSLoopClosure() {
    if (keyframePoseIds.size() < 10) {
        return;  // 关键帧太少，无法进行回环检测
    }
    int loopKeyCur = keyframePoseIds.size() - 1;
    int loopKeyPre = -1;
    // 回环检测成功
    if (detectLoopClosureDistance(loopKeyCur, loopKeyPre)) {
        LOG_INFO("Loop closure detected between keyframes {} and {}", loopKeyCur, loopKeyPre);
        mBuf.lock();
        loopClosureQueue.push({loopKeyPre, loopKeyCur});
        loopIndexContainer[loopKeyCur] = loopKeyPre;
        mBuf.unlock();
    } else {
        return;
    }
}

bool PosegraphOptimization::detectLoopClosureDistance(int &loopKeyCur, int &loopKeyPre) {
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end()) {
        // 说明该关键帧找到过回环，然后假设你一直停留在这个地方，就不需要一直消耗资源检测回环了
        return false;
    }
    std::vector<int> pointSearchIdxLoop;
    std::vector<float> pointSearchSqDisLoop;
    keyframePoseKdTree.setInputCloud(keyframePoseCloud);
    keyframePoseKdTree.radiusSearch(keyframePoseCloud->back(), historyKeyframeSearchRadius, pointSearchIdxLoop,
                                    pointSearchSqDisLoop, 0);
    // 遍历被搜索到的点，判断是否有回环
    for (int i = 0; i < pointSearchIdxLoop.size(); i++) {
        int id = pointSearchIdxLoop[i];
        // 时间差满足条件，说明找到时间上比较老的关键帧了，认为是回环
        if (std::abs(keyframeTimeBuf[id] - keyframeTimeBuf[loopKeyCur]) > historyKeyframeSearchTimeDiff) {
            loopKeyPre = id;
            break;
        }
    }
    // -1 说明没有找到回环
    if (loopKeyPre == -1 || loopKeyPre == loopKeyCur) {
        return false;
    }
    return true;
}

void PosegraphOptimization::addKFPoseToCloud(const PoseTrans &pose) {
    keyframePoseCloud->points.push_back(pcl::PointXYZ(pose.t.x(), pose.t.y(), pose.t.z()));
}

void PosegraphOptimization::visualizeLoopClosure() {
    if (loopIndexContainer.empty()) {
        return;
    }
    MarkerArrayMsg markerArray;
    // 闭环顶点
    MarkerMsg markerNode;
    markerNode.header.frame_id = "map";
    PgoSetStamp(markerNode.header.stamp, keyframeTimeBuf[keyframePoseIds.size() - 1]);
    markerNode.action = MarkerMsg::ADD;
    markerNode.type = MarkerMsg::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3;
    markerNode.scale.y = 0.3;
    markerNode.scale.z = 0.3;
    markerNode.color.r = 0;
    markerNode.color.g = 0.8;
    markerNode.color.b = 1;
    markerNode.color.a = 1;
    // 闭环边
    MarkerMsg markerEdge;
    markerEdge.header.frame_id = "map";
    PgoSetStamp(markerEdge.header.stamp, keyframeTimeBuf[keyframePoseIds.size() - 1]);
    markerEdge.action = MarkerMsg::ADD;
    markerEdge.type = MarkerMsg::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0.9;
    markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    // 遍历闭环
    for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it) {
        int key_cur = it->first;
        int key_pre = it->second;
        PointMsg p;
        p.x = keyframePoseOptimized[key_cur].pose.t.x();
        p.y = keyframePoseOptimized[key_cur].pose.t.y();
        p.z = keyframePoseOptimized[key_cur].pose.t.z();
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = keyframePoseOptimized[key_pre].pose.t.x();
        p.y = keyframePoseOptimized[key_pre].pose.t.y();
        p.z = keyframePoseOptimized[key_pre].pose.t.z();
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
#if ROS_AVAILABLE == 1
    pubLoopConstraintEdge.publish(markerArray);
#else
    pubLoopConstraintEdge->publish(markerArray);
#endif
}

#if ROS_AVAILABLE == 1
void PosegraphOptimization::odomToPoseTrans(const OdomPtr &odom, PoseTrans &pose) {
    auto tx = odom->pose.pose.position.x;
    auto ty = odom->pose.pose.position.y;
    auto tz = odom->pose.pose.position.z;
    Eigen::Quaterniond quat(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
                            odom->pose.pose.orientation.z);
    quat.normalize();
    pose = PoseTrans(quat.toRotationMatrix(), Eigen::Vector3d(tx, ty, tz));
}
#else
void PosegraphOptimization::odomToPoseTrans(const OdomPtr odom, PoseTrans &pose) {
    auto tx = odom->pose.pose.position.x;
    auto ty = odom->pose.pose.position.y;
    auto tz = odom->pose.pose.position.z;
    Eigen::Quaterniond quat(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
                            odom->pose.pose.orientation.z);
    quat.normalize();
    pose = PoseTrans(quat.toRotationMatrix(), Eigen::Vector3d(tx, ty, tz));
}
#endif

gtsam::Pose3 PosegraphOptimization::poseTransToPose3(const PoseTrans &pose) {
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(pose.RPY().x(), pose.RPY().y(), pose.RPY().z()),
                        gtsam::Point3(pose.t.x(), pose.t.y(), pose.t.z()));
}

void PosegraphOptimization::runMapVisualization() {
    slam::Rate rate(vizmapFrequency);
    while (slam::RosOk()) {
        rate.sleep();
        if (recentIdxUpdated > 1) publishMap();
    }
}

void PosegraphOptimization::publishMap() {
    int SKIP_FRAMES = 1;  // sparse map visulalization to save computations
    int counter = 0;
    std::lock_guard<std::mutex> lock(mKF);  // 自动解锁，更安全
    PointCloudXYZIPtr local_map(new PointCloudXYZI());
    mapCloud->clear();
    for (int node_idx = 0; node_idx < recentIdxUpdated; node_idx++) {
        if (counter % SKIP_FRAMES == 0) {
            PointCloudXYZIPtr tmpCloud(new PointCloudXYZI());
            LOG_INFO("node_idx: {}, keyframe size:{}", node_idx, keyframePoseOptimized.size());
            TransformCloud(keyframeCloudBuf.at(node_idx), tmpCloud, keyframePoseOptimized.at(node_idx).pose.R,
                           keyframePoseOptimized.at(node_idx).pose.t);
            *mapCloud += *tmpCloud;
        }
        counter++;
    }
    // to ros msg
    CloudMsg mapCloudMsg;
    pcl::toROSMsg(*mapCloud, mapCloudMsg);
    mapCloudMsg.header.frame_id = "map";
    PgoSetStamp(mapCloudMsg.header.stamp, PgoNow());
#if ROS_AVAILABLE == 1
    pubMapAftPGO.publish(mapCloudMsg);
#else
    pubMapAftPGO->publish(mapCloudMsg);
#endif
}

// 保存优化后的结果(按地图名称): 保存到 <map_save_dir_>/<map_name>/
//   整图pcd + keyframes/(点云+位姿txt) + tiles/(50x50 图元)
bool PosegraphOptimization::SaveMapByName(const std::string &map_name) {
    // 地图名称净化, 防止路径逃逸
    std::string name = map_name;
    for (auto &ch : name) {
        if (!std::isalnum(static_cast<unsigned char>(ch)) && ch != '_' && ch != '-' && ch != '.') {
            ch = '_';
        }
    }
    if (name.empty()) {
        LOG_WARN("PGO map save skipped: empty map name");
        return false;
    }

    const std::filesystem::path root = std::filesystem::path(map_save_dir_) / name;
    const std::string save_path = (root / (name + ".pcd")).string();
    const std::string kf_dir = (root / "keyframes").string();
    const std::string tiles_root = (root / "tiles").string();
    try {
        std::filesystem::create_directories(kf_dir);
    } catch (const std::filesystem::filesystem_error &e) {
        LOG_WARN("create dir {} failed: {}", kf_dir, e.what());
    }

    // 1) 锁内快照: 复制关键帧点云(shared_ptr)与优化位姿, 之后在锁外做重计算
    size_t n = 0;
    std::vector<PoseTrans> kf_poses;
    std::vector<PointCloudXYZIPtr> kf_clouds;
    {
        std::lock_guard<std::mutex> lock(mKF);
        n = std::min(keyframeCloudBuf.size(), keyframePoseOptimized.size());
        if (n < 2) {
            LOG_WARN("PGO map save '{}' skipped: not enough keyframes ({})", name, n);
            return false;
        }
        kf_poses.reserve(n);
        kf_clouds.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            kf_clouds.push_back(keyframeCloudBuf.at(i));
            kf_poses.push_back(keyframePoseOptimized.at(i).pose);
        }
    }

    // 2) 拼装全局地图; 同时保存每个关键帧点云(地图系)与位姿 txt
    PointCloudXYZIPtr map_out(new PointCloudXYZI());
    std::string poses_path = kf_dir + "/keyframe_poses.txt";
    std::ofstream ofs(poses_path);
    ofs << "# keyframe poses (optimized, map frame)\n"
        << "# index tx ty tz qx qy qz qw\n";
    ofs << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < n; ++i) {
        PointCloudXYZIPtr tmp(new PointCloudXYZI());
        TransformCloud(kf_clouds[i], tmp, kf_poses[i].R, kf_poses[i].t);
        if (map_save_keyframes_) {
            tmp->width = static_cast<std::uint32_t>(tmp->size());
            tmp->height = 1;
            tmp->is_dense = false;
            std::string kf_pcd = kf_dir + "/keyframe_" + std::to_string(i) + ".pcd";
            pcl::io::savePCDFileBinary(kf_pcd, *tmp);
        }
        *map_out += *tmp;
        // 位姿行
        Eigen::Quaterniond q(kf_poses[i].R);
        q.normalize();
        ofs << i << " " << kf_poses[i].t.x() << " " << kf_poses[i].t.y() << " " << kf_poses[i].t.z() << " " << q.x()
            << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
    ofs.close();
    LOG_INFO("PGO keyframes saved: {} clouds -> {}, poses -> {}", n, kf_dir, poses_path);

    if (map_out->empty()) {
        LOG_WARN("PGO map save '{}' skipped: empty map", name);
        return false;
    }

    // 3) 全局地图体素降采样后保存
    PointCloudXYZIPtr filtered =
        (map_save_voxel_ > 0.0) ? VoxelFilter(map_out, static_cast<float>(map_save_voxel_)) : map_out;
    filtered->width = static_cast<std::uint32_t>(filtered->size());
    filtered->height = 1;
    filtered->is_dense = false;
    try {
        std::filesystem::create_directories(root);
    } catch (const std::filesystem::filesystem_error &e) {
        LOG_WARN("create dir for {} failed: {}", save_path, e.what());
    }
    if (pcl::io::savePCDFileBinary(save_path, *filtered) < 0) {
        LOG_ERROR("PGO map save failed: {}", save_path);
        return false;
    }
    LOG_INFO(GREEN "PGO optimized map saved: {} (points {})" RESET, save_path, filtered->size());

    // 4) 切图元(默认 50x50), 目录结构兼容 Localizer::LoadMetaMapsFromDir
    if (map_save_tile_enable_ && map_save_tile_size_ > 0.0) {
        int tile_cnt = slam::SplitGlobalMapToTiles(filtered, tiles_root, map_save_leaf_, map_save_tile_size_, 10);
        LOG_INFO(GREEN "PGO optimized map tiles: {} tiles ({}x{} m) -> {}, leaf {}" RESET, tile_cnt, map_save_tile_size_,
                 map_save_tile_size_, tiles_root, map_save_leaf_);
    }
    return true;
}

#if ROS_AVAILABLE == 1
void PosegraphOptimization::publishCloud(ros::Publisher &pub, const PointCloudXYZIPtr cloud, std::string frame_id) {
    CloudMsg cloudMsg;
    pcl::toROSMsg(*cloud, cloudMsg);
    cloudMsg.header.frame_id = frame_id;
    PgoSetStamp(cloudMsg.header.stamp, PgoNow());
    pub.publish(cloudMsg);
}
#else
void PosegraphOptimization::publishCloud(const rclcpp::Publisher<CloudMsg>::SharedPtr &pub, const PointCloudXYZIPtr cloud,
                                         std::string frame_id) {
    CloudMsg cloudMsg;
    pcl::toROSMsg(*cloud, cloudMsg);
    cloudMsg.header.frame_id = frame_id;
    PgoSetStamp(cloudMsg.header.stamp, PgoNow());
    pub->publish(cloudMsg);
}
#endif
