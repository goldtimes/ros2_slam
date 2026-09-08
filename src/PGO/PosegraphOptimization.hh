#pragma once
// ============================================================
//  PosegraphOptimization —— ROS1/ROS2 双版本
//  算法核心(gtsam / PCL)与 ROS 版本无关,仅接口层按 ROS_AVAILABLE 分支
// ============================================================
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
#include <deque>
#include <filesystem>  // C++17 自带，无需安装库
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include "common/logger.hh"
#include "common/pose_trans.hh"
#include "ros/ros_common.hh"
#include "utils/pointcloud_utils.hh"

#if ROS_AVAILABLE == 1
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/package.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <lio_slam/srv/save_map.hpp>
#endif

namespace slam {

// ==================== 消息类型别名 ====================
#if ROS_AVAILABLE == 1
using OdomMsg        = nav_msgs::Odometry;
using PathMsg        = nav_msgs::Path;
using NavSatMsg      = sensor_msgs::NavSatFix;
using CloudMsg       = sensor_msgs::PointCloud2;
using MarkerArrayMsg = visualization_msgs::MarkerArray;
using MarkerMsg      = visualization_msgs::Marker;
using PointMsg       = geometry_msgs::Point;
using PoseStampedMsg = geometry_msgs::PoseStamped;
using TransformMsg   = geometry_msgs::TransformStamped;
using OdomPtr        = nav_msgs::Odometry::ConstPtr;
using CloudPtr       = sensor_msgs::PointCloud2::ConstPtr;
using NavSatPtr      = sensor_msgs::NavSatFix::ConstPtr;
#else
using OdomMsg        = nav_msgs::msg::Odometry;
using PathMsg        = nav_msgs::msg::Path;
using NavSatMsg      = sensor_msgs::msg::NavSatFix;
using CloudMsg       = sensor_msgs::msg::PointCloud2;
using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;
using MarkerMsg      = visualization_msgs::msg::Marker;
using PointMsg       = geometry_msgs::msg::Point;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using TransformMsg   = geometry_msgs::msg::TransformStamped;
using OdomPtr        = nav_msgs::msg::Odometry::ConstSharedPtr;
using CloudPtr       = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
using NavSatPtr      = sensor_msgs::msg::NavSatFix::ConstSharedPtr;
#endif

struct KFPose {
    size_t index;
    PoseTrans pose;
    KFPose(size_t index, const PoseTrans &pose) : index(index), pose(pose) {
    }
};

class PosegraphOptimization {
   public:
#if ROS_AVAILABLE == 1
    explicit PosegraphOptimization(ros::NodeHandle &nh);
#else
    explicit PosegraphOptimization(const rclcpp::Node::SharedPtr &node);
#endif
    ~PosegraphOptimization();

   private:
    double deg2rad(double deg) {
        return deg * M_PI / 180.0;
    }
    // 参数读取(ROS1: nh.param / ROS2: get_parameter_or)
#if ROS_AVAILABLE == 1
    template <typename T>
    void GetParam(const std::string &name, T &out, const T &def) {
        nh_.param<T>(name, out, def);
    }
#else
    template <typename T>
    void GetParam(const std::string &name, T &out, const T &def) {
        nh_->get_parameter_or<T>(name, out, def);
    }
#endif

    void initNoise();
    void init_subpub();

#if ROS_AVAILABLE == 1
    void laserOdomCallback(const OdomPtr &msg);
    void gspCallback(const NavSatPtr &msg);
    void cloudCallback(const CloudPtr &msg);
    void odomToPoseTrans(const OdomPtr &odom, PoseTrans &pose);
    void publishCloud(ros::Publisher &pub, const PointCloudXYZIPtr cloud, std::string frame_id = "map");
#else
    void laserOdomCallback(const OdomMsg::SharedPtr msg);
    void gspCallback(const NavSatMsg::SharedPtr msg);
    void cloudCallback(const CloudMsg::SharedPtr msg);
    void odomToPoseTrans(const OdomPtr odom, PoseTrans &pose);
    void publishCloud(const rclcpp::Publisher<CloudMsg>::SharedPtr &pub, const PointCloudXYZIPtr cloud,
                      std::string frame_id = "map");
#endif

    void run();
    void runLoopDetection();
    void runLoopConstraint();
    void runISAMUpdate();
    void runMapVisualization();

    void performRSLoopClosure();
    void visualizeLoopClosure();
    bool detectLoopClosureDistance(int &loopKeyCur, int &loopKeyPre);
    PoseTrans doICPVirtualRelative(int loopKeyPre, int loopKeyCur);
    void loopFindNearKeyframe(PointCloudXYZIPtr &nearKeyframes, const int key, const int search_num);
    void isamUpdate();
    void updatePose();
    void publishState();
    void publishMap();
    // 保存优化后的地图(按名称): 保存到 map_save_dir_/<map_name>/
    // 产出 整图pcd + keyframes/ + tiles/ ; 成功返回 true
    bool SaveMapByName(const std::string &map_name);
    void addKFPoseToCloud(const PoseTrans &pose);
    gtsam::Pose3 poseTransToPose3(const PoseTrans &pose);

   private:
    // PGO 结果目录(ROS1: 包路径;ROS2: 可配置参数 pgo_result_dir)
    std::string PGODir;

#if ROS_AVAILABLE == 1
    ros::NodeHandle nh_;
    // tf
    tf2_ros::TransformBroadcaster tfBroadcaster;
    // 订阅
    ros::Subscriber lidarOdom_sub_;
    ros::Subscriber lidarScan_sub_;
    ros::Subscriber gps_sub_;
    // 发布
    ros::Publisher pubLoopScanLocal;
    ros::Publisher pubLoopSubmapLocal;
    ros::Publisher pubLoopScanLocalRegisted;
    ros::Publisher pubLoopConstraintEdge;
    ros::Publisher keyframe_pub_;
    ros::Publisher pubPathAftPGO;
    ros::Publisher pubOdomAftPGO;
    ros::Publisher pubMapAftPGO;
#else
    rclcpp::Node::SharedPtr nh_;
    // tf
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    // 订阅
    rclcpp::Subscription<OdomMsg>::SharedPtr lidarOdom_sub_;
    rclcpp::Subscription<CloudMsg>::SharedPtr lidarScan_sub_;
    rclcpp::Subscription<NavSatMsg>::SharedPtr gps_sub_;
    // 发布
    rclcpp::Publisher<CloudMsg>::SharedPtr pubLoopScanLocal;
    rclcpp::Publisher<CloudMsg>::SharedPtr pubLoopSubmapLocal;
    rclcpp::Publisher<CloudMsg>::SharedPtr pubLoopScanLocalRegisted;
    rclcpp::Publisher<MarkerArrayMsg>::SharedPtr pubLoopConstraintEdge;
    rclcpp::Publisher<CloudMsg>::SharedPtr keyframe_pub_;
    rclcpp::Publisher<PathMsg>::SharedPtr pubPathAftPGO;
    rclcpp::Publisher<OdomMsg>::SharedPtr pubOdomAftPGO;
    rclcpp::Publisher<CloudMsg>::SharedPtr pubMapAftPGO;
    // 保存地图服务(ROS2 only): 请求带地图名称
    rclcpp::Service<lio_slam::srv::SaveMap>::SharedPtr save_map_srv_;
#endif

    // 关键帧的距离
    double keyframeMeterGap;
    double keyframeDegGap;
    double keyframeRadGap;

    double keyframe_downsample = 0.2;
    // 闭环检测的参数
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
    std::map<int, int> loopIndexContainer;  // key是当前帧索引，value是闭环帧索引

    // 图优化线程的频率
    double speedFactor;

    int graphUpdateTimes;  // 图优化迭代的次数
    double graphUpdateFrequency;
    double loopClosureFrequency;
    double vizmapFrequency = 1.0;

    // 地图保存
    bool map_save_enable_ = false;       // 空闲自动保存开关(默认关, 用 srv 手动保存)
    double map_save_idle_sec_ = 10.0;    // 输入停止多久后(秒)自动保存一次
    double map_save_voxel_ = 0.1;        // 保存时体素降采样(<=0 表示不降采样)
    std::string map_save_dir_ = "/home/li/ros2_ws/maps";   // 保存根目录
    std::string map_save_name_ = "pgo_optimized_map";      // 自动保存使用的默认名称
    bool map_save_tile_enable_ = true;   // 是否把地图切成图元(META)
    double map_save_tile_size_ = 50.0;   // 图元边长(米)
    std::string map_save_leaf_ = "MAP_GLOBAL";  // 图元叶子地图 identity
    bool map_save_keyframes_ = true;     // 是否保存关键帧点云(地图系)与位姿 txt
    bool auto_map_saved_ = false;        // 已自动保存过(避免重复写盘)

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
    gtsam::ISAM2 *isam;                      // 优化器
    gtsam::NonlinearFactorGraph gtSAMgraph;  // 因子图
    gtsam::Values initialEstimate;           // 初始值
    gtsam::Values isamCurrentEstimate;       // 优化后的结果
    bool gtSAMgraphMade = false;

    double recentOptimizedX = 0.0;
    double recentOptimizedY = 0.0;
    int recentIdxUpdated = 0;
    std::mutex mOptimizedPose;

    // 噪声
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise;     // 先验因子噪声
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;  // 里程计因子噪声
    gtsam::noiseModel::Base::shared_ptr robustGPSNoise;     // GPS因子
    gtsam::noiseModel::Base::shared_ptr robustLoopNoise;    // 回环因子鲁棒核函数

    // 存储数据
    std::mutex mBuf;                            // 互斥锁
    std::deque<OdomPtr> odomBuf;                // 雷达里程计缓冲区
    std::deque<NavSatPtr> gpsBuf;               // gps 缓冲区
    std::deque<CloudPtr> cloudBuf;              // 雷达点云缓冲区
    std::deque<double> cloudTimeBuf;            // 雷达点云时间戳缓冲区
    PointCloudXYZIPtr laserCloud;
    PointCloudXYZIPtr mapCloud;
    double globalMapDownSize = 0.3;

    std::mutex mKF;
    std::deque<PointCloudXYZIPtr> keyframeCloudBuf;  // 关键帧点云缓冲区
    std::vector<KFPose> keyframePoseIds;  // 关键帧位姿缓冲区，key是关键帧索引，value是位姿
    size_t keyframeIndex = 0;             // 关键帧索引

    // 优化后的位姿
    std::vector<KFPose> keyframePoseOptimized;  // 关键帧优化后的位姿缓冲区
    std::deque<double> keyframeTimeBuf;         // 关键帧时间戳缓冲区
    double timeLaserOdometry = 0.0;
    double timeLaser = 0.0;
    PoseTrans lastKeyframePose;
    PoseTrans currentPose;
    double translationAccumulated;
    double rotationAccumulated;
    bool isKeyframe;
    bool use_gps;
};
}  // namespace slam
