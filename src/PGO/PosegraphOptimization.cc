#include "PosegraphOptimization.hh"
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>
using namespace slam;

PosegraphOptimization::PosegraphOptimization(ros::NodeHandle &nh) : nh_(nh) {
    // 加载参数
    nh.param<double>("keyframe_meter_gap", keyframeMeterGap,
                     2.0);  // pose assignment every k m move
    nh.param<double>("keyframe_deg_gap", keyframeDegGap,
                     10.0);  // pose assignment every k deg rot
    keyframeRadGap = deg2rad(keyframeDegGap);
    LOG_INFO("keyframeMeterGap: {}, keyframeDegGap: {}, keyframeRadGap: {}", keyframeMeterGap, keyframeDegGap,
             keyframeRadGap);

    //   nh.param<double>("sc_dist_thres", scDistThres, 0.2);
    //   nh.param<double>("sc_max_radius", scMaximumRadius,
    //                    80.0); // 80 is recommended for outdoor, and lower (ex,
    //                    20,
    //                           // 40) values are recommended for indoor

    // for loop closure detection
    nh.param<double>("historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
    nh.param<double>("historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
    nh.param<int>("historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
    nh.param<double>("loopNoise", loopNoise, 0.5);
    LOG_INFO(
        "historyKeyframeSearchRadius: {}, historyKeyframeSearchTimeDiff: "
        "{}, historyKeyframeSearchNum: {}, loopNoise: {}",
        historyKeyframeSearchRadius, historyKeyframeSearchTimeDiff, historyKeyframeSearchNum, loopNoise);
    nh.param<int>("graphUpdateTimes", graphUpdateTimes, 2);
    nh.param<double>("loopFitnessScoreThreshold", loopFitnessScoreThreshold, 0.3);
    LOG_INFO("graphUpdateTimes: {}, loopFitnessScoreThreshold: {}", graphUpdateTimes, loopFitnessScoreThreshold);
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
    LOG_INFO("loopClosureFrequency: {}, graphUpdateFrequency: {}, vizmapFrequency: {}", loopClosureFrequency,
             graphUpdateFrequency, vizmapFrequency);

    // 初始化gtsam参数
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);
    // 初始化因子图噪声
    initNoise();

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
    lidarOdom_sub_ = nh_.subscribe("lidar_odom", 100, &PosegraphOptimization::laserOdomCallback, this);
    lidarScan_sub_ = nh_.subscribe("lidar_registered_cloud", 100, &PosegraphOptimization::cloudCallback, this);
    if (use_gps) {
        gps_sub_ = nh_.subscribe("gps", 100, &PosegraphOptimization::gspCallback, this);
        LOG_INFO("gps subscribed on topic: {}", "gps");
    }

    pubLoopScanLocal = nh_.advertise<sensor_msgs::PointCloud2>("loop_scan_local", 100);
    pubLoopSubmapLocal = nh_.advertise<sensor_msgs::PointCloud2>("loop_submap_local", 100);
    pubLoopScanLocalRegisted = nh_.advertise<sensor_msgs::PointCloud2>("loop_scan_local_registed", 100);

    // 发布路径
    pubPathAftPGO = nh_.advertise<nav_msgs::Path>("aft_pgo_path", 100);
    pubOdomAftPGO = nh_.advertise<nav_msgs::Odometry>("aft_pgo_odom", 100);
    pubMapAftPGO = nh_.advertise<sensor_msgs::PointCloud2>("aft_pgo_map", 100);

    pubLoopConstraintEdge = nh_.advertise<visualization_msgs::MarkerArray>("loop_closure_edges", 100);

    LOG_INFO("lidar odometry subscribed on topic: {}", "lidar_odom");
    LOG_INFO("lidar scan subscribed on topic: {}", "lidar_registered_cloud");
}

void PosegraphOptimization::laserOdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mBuf);
    odomBuf.push_back(msg);
}
void PosegraphOptimization::gspCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mBuf);
    gpsBuf.push_back(msg);
}
void PosegraphOptimization::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mBuf);
    cloudBuf.push_back(msg);
}

void PosegraphOptimization::run() {
    while (ros::ok()) {
        // 确保里程计和点云数据都有了再处理
        while (!odomBuf.empty() && !cloudBuf.empty()) {
            mBuf.lock();
            // 如果里程计时间戳比点云时间戳小，说明这个里程计数据还没有对应的点云数据，丢弃这个里程计数据
            while (!odomBuf.empty() && odomBuf.front()->header.stamp.toSec() < cloudBuf.front()->header.stamp.toSec()) {
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
            if (translationAccumulated >= keyframeMeterGap || rotationAccumulated >= keyframeRadGap) {
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
            // 构建先验的里程计因子图
            const int prev_node_idx = keyframePoseIds.size() - 2;
            const int curr_node_idx = keyframePoseIds.size() - 1;
            if (!gtSAMgraphMade) {
                // 第一帧
                const int init_node_idx = 0;
                auto init_pose = keyframePoseIds.at(init_node_idx).pose;
                gtsam::Pose3 poseOrigin = poseTransToPose3(init_pose);
                // 对因子图加锁
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
                // 对因子图加锁
                mGraph.lock();
                gtSAMgraph.add(
                    gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relativePose, odometryNoise));
                // TODO GPS factor
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
    ros::Rate rate(loopClosureFrequency);
    while (ros::ok()) {
        rate.sleep();
        // 基于距离的回环检测
        performRSLoopClosure();
        visualizeLoopClosure();
    }
}

void PosegraphOptimization::runLoopConstraint() {
    while (ros::ok()) {
        // ICP确认,确保回环候选队列不为空
        while (!loopClosureQueue.empty()) {
            if (loopClosureQueue.size() > 30) {
                LOG_WARN(
                    "Too many loop clousre candidates to be ICPed is waiting ... "
                    "Do process_lcd less frequently "
                    "(adjust loopClosureFrequency)");
            }
            // 取出队列中的回环候选对
            mBuf.lock();
            auto loopPair = loopClosureQueue.front();
            loopClosureQueue.pop();
            mBuf.unlock();
            auto prev_node_idx = loopPair.first;
            auto curr_node_idx = loopPair.second;
            auto relativePose = doICPVirtualRelative(prev_node_idx, curr_node_idx);
            gtsam::Pose3 relativePose3 = poseTransToPose3(relativePose);
            if (relativePose3.equals(gtsam::Pose3::Identity())) {
                mGraph.lock();
                gtSAMgraph.add(
                    gtsam::BetweenFactor<gtsam::Pose3>(curr_node_idx, prev_node_idx, relativePose3, robustLoopNoise));
                mGraph.unlock();
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
    icp.setMaxCorrespondenceDistance(150);  // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
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
        LOG_INFO("[SC loop] ICP fitness test failed ({} > {}). Reject this SC loop.");
        return PoseTrans();
    } else {
        LOG_INFO("[SC loop] ICP fitness test passed ({} < {}). Add this SC loop.", icp.getFitnessScore(),
                 loopFitnessScoreThreshold);
    }
    auto icp_transform = icp.getFinalTransformation();
    // 闭环优化前的loop_pre的位姿是T1, 当前的位姿是T2
    // 现在我们将current配准到了target,于是我们要更新正确的T2
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
    nearKeyframes = VoxelFilter(nearKeyframes, 0.3);
}

void PosegraphOptimization::runISAMUpdate() {
    ros::Rate rate(graphUpdateFrequency);
    while (ros::ok()) {
        rate.sleep();

        if (gtSAMgraphMade) {
            mGraph.lock();
            isamUpdate();
            mGraph.unlock();
            // TODO save pose
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
    nav_msgs::Path pathMsg;
    nav_msgs::Odometry odomMsg;
    pathMsg.header.frame_id = "map";
    mKF.lock();
    for (int i = 0; i < recentIdxUpdated; i++) {
        const KFPose &kfPose = keyframePoseOptimized.at(i);
        nav_msgs::Odometry odom;
        odom.header.frame_id = "map";
        odom.child_frame_id = "aft_pgo";
        odom.header.stamp = ros::Time().fromSec(keyframeTimeBuf.at(i));
        odom.pose.pose.position.x = kfPose.pose.t.x();
        odom.pose.pose.position.y = kfPose.pose.t.y();
        odom.pose.pose.position.z = kfPose.pose.t.z();
        Eigen::Quaterniond q(kfPose.pose.R);
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odomMsg = odom;
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header = odom.header;
        poseStamped.pose = odom.pose.pose;
        pathMsg.header.stamp = odom.header.stamp;
        pathMsg.header.frame_id = "map";
        pathMsg.poses.push_back(poseStamped);
    }
    mKF.unlock();
    geometry_msgs::TransformStamped tfMsg;
    tfMsg.header.stamp = odomMsg.header.stamp;
    tfMsg.header.frame_id = "map";
    tfMsg.child_frame_id = "aft_pgo";
    tfMsg.transform.translation.x = odomMsg.pose.pose.position.x;
    tfMsg.transform.translation.y = odomMsg.pose.pose.position.y;
    tfMsg.transform.translation.z = odomMsg.pose.pose.position.z;
    tfMsg.transform.rotation = odomMsg.pose.pose.orientation;
    tfBroadcaster.sendTransform(tfMsg);
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
    visualization_msgs::MarkerArray markerArray;
    // 闭环顶点
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = "map";  // camera_init
    markerNode.header.stamp = ros::Time().fromSec(keyframeTimeBuf[keyframePoseIds.size() - 1]);
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
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
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = "map";
    markerEdge.header.stamp = ros::Time().fromSec(keyframeTimeBuf[keyframePoseIds.size() - 1]);
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
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
        geometry_msgs::Point p;
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
    pubLoopConstraintEdge.publish(markerArray);
}

void PosegraphOptimization::odomToPoseTrans(const nav_msgs::Odometry::ConstPtr &odom, PoseTrans &pose) {
    auto tx = odom->pose.pose.position.x;
    auto ty = odom->pose.pose.position.y;
    auto tz = odom->pose.pose.position.z;
    Eigen::Quaterniond quat(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
                            odom->pose.pose.orientation.z);
    quat.normalize();
    pose = PoseTrans(quat.toRotationMatrix(), Eigen::Vector3d(tx, ty, tz));
}

gtsam::Pose3 PosegraphOptimization::poseTransToPose3(const PoseTrans &pose) {
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(pose.RPY().x(), pose.RPY().y(), pose.RPY().z()),
                        gtsam::Point3(pose.t.x(), pose.t.y(), pose.t.z()));
}

void PosegraphOptimization::runMapVisualization() {
    ros::Rate rate(vizmapFrequency);
    while (ros::ok()) {
        rate.sleep();
        publishMap();
    }
}

void PosegraphOptimization::publishMap() {
    int SKIP_FRAMES = 2;  // sparse map visulalization to save computations
    int counter = 0;
    mKF.lock();

    for (int node_idx = 0; node_idx < recentIdxUpdated; node_idx++) {
        if (counter % SKIP_FRAMES == 0) {
            PointCloudXYZIPtr tmpCloud(new PointCloudXYZI());
            TransformCloud(keyframeCloudBuf.at(node_idx), tmpCloud, keyframePoseOptimized.at(node_idx).pose.R,
                           keyframePoseOptimized.at(node_idx).pose.t);
            *mapCloud += *tmpCloud;
        }
        counter++;
    }
    mKF.unlock();
    mapCloud = VoxelFilter(mapCloud, globalMapDownSize);
    // to ros msg
    sensor_msgs::PointCloud2 mapCloudMsg;
    pcl::toROSMsg(*mapCloud, mapCloudMsg);
    mapCloudMsg.header.frame_id = "map";
    mapCloudMsg.header.stamp = ros::Time().now();
    pubMapAftPGO.publish(mapCloudMsg);
}

void PosegraphOptimization::publishCloud(ros::Publisher &pub, const PointCloudXYZIPtr cloud, std::string frame_id) {
    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg(*cloud, cloudMsg);
    cloudMsg.header.frame_id = frame_id;
    cloudMsg.header.stamp = ros::Time().now();
    pub.publish(cloudMsg);
}