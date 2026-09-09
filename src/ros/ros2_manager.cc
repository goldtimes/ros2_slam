// ============================================================
//  ROS2 版 SLAM 管理器实现(与 ros1_manager.cc 对应)
// ============================================================
#include "ros/ros2_manager.hh"
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <functional>
#include "lidar_register/voxel_map.hh"
#include "localizer/localizer.hh"
#include "system/system.hh"
#include "system/system_config.hh"
#include "utils/pointcloud_utils.hh"
#include "utils/timer_record.hh"

namespace {
// 将 double 秒写入 builtin_interfaces 时间戳
inline void SetStamp(builtin_interfaces::msg::Time &stamp, double sec) {
    int32_t s = static_cast<int32_t>(std::floor(sec));
    int64_t ns = static_cast<int64_t>((sec - static_cast<double>(s)) * 1e9);
    stamp.sec = s;
    stamp.nanosec = static_cast<uint32_t>(ns);
}
inline double NowSec() {
    return rclcpp::Clock().now().seconds();
}
}  // namespace

namespace slam {

ROS2Manager::ROS2Manager(const rclcpp::Node::SharedPtr &node, std::shared_ptr<System> system_ptr)
    : nh_(node), system_ptr_(system_ptr) {
    LOG_INFO("ROS2Manager init");
    std::string slam_mode;
    nh_->get_parameter_or("slam_mode", slam_mode, std::string("mapping"));

    if (slam_mode == "mapping") {
        slam_mode_ = SLAM_MODE::MAPPING;
    } else if (slam_mode == "slam") {
        slam_mode_ = SLAM_MODE::LOCALIZATION;
    } else if (slam_mode == "odometer") {
        slam_mode_ = SLAM_MODE::ODOMETER;
    }

    has_encoder_ = system_ptr_->GetSystemConfig()->has_encoder_;
    has_gnss_ = system_ptr_->GetSystemConfig()->has_gnss_;
    if (has_gnss_) {
        gnss_process_ = std::make_shared<GnssProcess>();
    }
    InitPub();
    InitSub();

    // 可视化定时器(10Hz)
    visualize_timer_ =
        nh_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ROS2Manager::Visualize, this));

    voxel_map_timer_ =
        nh_->create_wall_timer(std::chrono::seconds(5), std::bind(&ROS2Manager::voxelTimerCB, this));
    if (!system_ptr_->GetSystemConfig()->frontend_config_.voxel_config.pub_voxel_map) {
        voxel_map_timer_->cancel();
    }

    // 定位模式: 使用图元地图时, 启动即从 local_map_dir 目录扫描加载。
    // 替代 ROS1 的 robot_manager::metaset_info 消息(ROS2 无该话题)。
    if (slam_mode_ == SLAM_MODE::LOCALIZATION && system_ptr_->GetLocalizer() != nullptr &&
        system_ptr_->GetSystemConfig()->localizer_config_.use_meta_maps) {
        const std::string map_dir = system_ptr_->GetSystemConfig()->localizer_config_.local_map_dir;
        system_ptr_->GetLocalizer()->LoadMetaMapsFromDir(map_dir);
    }
}

ROS2Manager::~ROS2Manager() {
}

void ROS2Manager::voxelTimerCB() {
    // auto voxel_map = system_ptr_->GetVoxelMap();
}

void ROS2Manager::InitPub() {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);

    // 节点私有话题(默认展开为 /slam_node/lie_slam/xxx,可在 launch 中 remap)
    cloud_lidar_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("~/lie_slam/lidar", 10);
    cloud_robot_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("~/lie_slam/robot_lidar", 10);
    cloud_odom_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("~/lie_slam/world_lidar", 10);
    cloud_body_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("~/lie_slam/body", 10);

    lio_path_pub_ = nh_->create_publisher<nav_msgs::msg::Path>("~/lie_slam/lio_path", 10);
    lio_odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("~/lie_slam/lio_odom", 10);
    if (has_encoder_) {
        encoder_path_pub_ = nh_->create_publisher<nav_msgs::msg::Path>("~/lie_slam/encoder_path", 10);
    }
    if (has_gnss_) {
        gnss_path_pub_ = nh_->create_publisher<nav_msgs::msg::Path>("~/lie_slam/gnss_path", 10);
        gnss_odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("~/lie_slam/gnss_odom", 10);
    }
    // transient_local 模拟 ROS1 的 latch
    global_map_pub_ =
        nh_->create_publisher<sensor_msgs::msg::PointCloud2>("~/lie_slam/global_map", rclcpp::QoS(1).transient_local());
    submap_pub_ =
        nh_->create_publisher<sensor_msgs::msg::PointCloud2>("~/lie_slam/submap", rclcpp::QoS(1).transient_local());
}

void ROS2Manager::InitSub() {
    // 传感器话题使用 best_effort
    imu_sub_ = nh_->create_subscription<sensor_msgs::msg::Imu>(
        system_ptr_->GetSystemConfig()->imu_config_.imu_topic, rclcpp::SensorDataQoS(),
        std::bind(&ROS2Manager::ImuCallback, this, std::placeholders::_1));
    if (system_ptr_->GetSystemConfig()->lidar_config_.use_livox_driver == 0) {
        lidar_sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
            system_ptr_->GetSystemConfig()->lidar_config_.lidar_topic, rclcpp::SensorDataQoS(),
            std::bind(&ROS2Manager::StandarCloudCallback, this, std::placeholders::_1));
        LOG_INFO("use standard lidar driver");
    } else if (system_ptr_->GetSystemConfig()->lidar_config_.use_livox_driver == 2) {
        lidar_livox_sub_ = nh_->create_subscription<LivoxMsg2>(
            system_ptr_->GetSystemConfig()->lidar_config_.lidar_topic, rclcpp::SensorDataQoS(),
            std::bind(&ROS2Manager::Livox2CloudCallback, this, std::placeholders::_1));
        LOG_INFO("use livox driver 2 (ROS2)");
    } else {
        LOG_ERROR("ROS2 only supports use_livox_driver 0(standard) or 2(livox_ros_driver2)!");
        std::exit(1);
    }
    if (has_encoder_) { 
    encoder_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
        system_ptr_->GetSystemConfig()->encoder_config_.encoder_topic, rclcpp::SensorDataQoS(),
        std::bind(&ROS2Manager::EncoderCallback, this, std::placeholders::_1));
    }
    if (has_gnss_) {
        gnss_sub_ = nh_->create_subscription<sensor_msgs::msg::NavSatFix>(
            system_ptr_->GetSystemConfig()->gnss_config_.gnss_topic, rclcpp::SensorDataQoS(),
            std::bind(&ROS2Manager::GNSSCallback, this, std::placeholders::_1));
    }

    // rviz 的 /initialpose(跨节点标准话题,使用全局名;可在 launch 中 remap)
    ros_init_pose_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::QoS(1),
        std::bind(&ROS2Manager::RosInitPoseCallback, this, std::placeholders::_1));
}

void ROS2Manager::StandarCloudCallback(const PointCloud2Msg::SharedPtr cloud_msg) {
    static double last_record_lidar_time = slam::StampToSec(cloud_msg->header.stamp);
    double curr_lidar_time = slam::StampToSec(cloud_msg->header.stamp);
    if (curr_lidar_time - last_record_lidar_time >= 5.0) {
        lidar_fps_ = static_cast<int>(lidar_frame_count_ / 5);
        LOG_INFO(GREEN "receive lidar fps:{}" RESET, lidar_fps_);
        lidar_frame_count_ = 0;
        last_record_lidar_time = curr_lidar_time;
    }
    if (last_lidar_time_ != -1 && curr_lidar_time - last_lidar_time_ > 0.2) {
        LOG_ERROR("lidar data lost");
        LOG_ERROR("current_lidar_time:{}, last_lidar_time:{}", curr_lidar_time, last_lidar_time_);
    }
    last_lidar_time_ = curr_lidar_time;
    lidar_frame_count_++;

    PointCloudPtr cloud_ptr(new PointCloudType);
    evaluate_and_call([&]() { system_ptr_->GetLidarProcess()->Process(cloud_msg, cloud_ptr); }, "lidar_process");
    if (!system_ptr_->GetSystemConfig()->lidar_config_.is_tms_head) {
        curr_lidar_time = cloud_ptr->points[0].time;
    }
    system_ptr_->AddLidar(cloud_ptr, curr_lidar_time);
}

void ROS2Manager::Livox2CloudCallback(const LivoxMsg2::SharedPtr cloud_livox) {
    static double last_record_lidar_time = slam::StampToSec(cloud_livox->header.stamp);
    double curr_lidar_time = slam::StampToSec(cloud_livox->header.stamp);
    if (curr_lidar_time - last_record_lidar_time >= 5.0) {
        lidar_fps_ = static_cast<int>(lidar_frame_count_ / 5);
        LOG_INFO(GREEN "receive lidar fps:{}" RESET, lidar_fps_);
        lidar_frame_count_ = 0;
        last_record_lidar_time = curr_lidar_time;
    }
    if (last_lidar_time_ != -1 && curr_lidar_time - last_lidar_time_ > 0.2) {
        LOG_ERROR("lidar data lost");
        LOG_ERROR("current_lidar_time:{}, last_lidar_time:{}", curr_lidar_time, last_lidar_time_);
    }
    last_lidar_time_ = curr_lidar_time;
    lidar_frame_count_++;

    PointCloudPtr cloud_ptr(new PointCloudType);
    evaluate_and_call([&]() { system_ptr_->GetLidarProcess()->Process(cloud_livox, cloud_ptr); }, "lidar_process");
    system_ptr_->AddLidar(cloud_ptr, curr_lidar_time);
}

void ROS2Manager::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    static double last_record_imu_time = slam::StampToSec(imu_msg->header.stamp);
    double curr_imu_time = slam::StampToSec(imu_msg->header.stamp);
    if (curr_imu_time - last_record_imu_time >= 5.0) {
        imu_fps_ = static_cast<int>(imu_frame_count_ / 5);
        LOG_INFO(GREEN "receive imu fps:{}" RESET, imu_fps_);
        imu_frame_count_ = 0;
        last_record_imu_time = curr_imu_time;
    }
    if (last_imu_time_ != -1 && curr_imu_time - last_imu_time_ > 0.2) {
        LOG_ERROR("imu data lost");
        LOG_ERROR("current_imu_time:{}, last_imu_time:{}", curr_imu_time, last_imu_time_);
    }
    last_imu_time_ = curr_imu_time;
    imu_frame_count_++;
    V3D acc, gyro;
    acc << imu_msg->linear_acceleration.x * system_ptr_->GetSystemConfig()->imu_config_.imu_scale,
        imu_msg->linear_acceleration.y * system_ptr_->GetSystemConfig()->imu_config_.imu_scale,
        imu_msg->linear_acceleration.z * system_ptr_->GetSystemConfig()->imu_config_.imu_scale;
    gyro << imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;
    IMU imu(curr_imu_time, acc, gyro);
    system_ptr_->AddIMU(imu);
}

void ROS2Manager::EncoderCallback(const nav_msgs::msg::Odometry::SharedPtr encoder_msg) {
    static double last_record_encoder_time = slam::StampToSec(encoder_msg->header.stamp);
    double curr_encoder_time = slam::StampToSec(encoder_msg->header.stamp);
    if (curr_encoder_time - last_record_encoder_time >= 5.0) {
        encoder_fps_ = static_cast<int>(encoder_frame_count_ / 5);
        LOG_INFO(GREEN "receive encoder fps:{}" RESET, encoder_fps_);
        encoder_frame_count_ = 0;
        last_record_encoder_time = curr_encoder_time;
    }
    if (last_encoder_time_ != -1 && curr_encoder_time - last_encoder_time_ > 0.2) {
        LOG_ERROR("encoder data lost");
        LOG_ERROR("current_encoder_time:{}, last_encoder_time:{}", curr_encoder_time, last_encoder_time_);
    }
    last_encoder_time_ = curr_encoder_time;
    encoder_frame_count_++;
    V3D linear_vel, angular_vel;
    linear_vel << encoder_msg->twist.twist.linear.x, encoder_msg->twist.twist.linear.y,
        encoder_msg->twist.twist.linear.z;
    angular_vel << encoder_msg->twist.twist.angular.x, encoder_msg->twist.twist.angular.y,
        encoder_msg->twist.twist.angular.z;
    Encoder encoder(curr_encoder_time, linear_vel, angular_vel);
    system_ptr_->AddEncoder(encoder);
}

void ROS2Manager::GNSSCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gnss_msg) {
    static double last_record_gnss_time = slam::StampToSec(gnss_msg->header.stamp);
    double curr_gnss_time = slam::StampToSec(gnss_msg->header.stamp);
    if (curr_gnss_time - last_record_gnss_time >= 5.0) {
        gnss_fps_ = static_cast<int>(gnss_frame_count_ / 5);
        LOG_INFO(GREEN "receive gnss fps:{}" RESET, gnss_fps_);
        gnss_frame_count_ = 0;
        last_record_gnss_time = curr_gnss_time;
    }
    if (last_gnss_time_ != -1 && curr_gnss_time - last_gnss_time_ > 2) {
        LOG_ERROR("gnss data lost");
        LOG_ERROR("current_gnss_time:{}, last_gnss_time:{}", curr_gnss_time, last_gnss_time_);
    }
    last_gnss_time_ = curr_gnss_time;
    gnss_frame_count_++;

    V3D lla;
    lla << gnss_msg->latitude, gnss_msg->longitude, gnss_msg->altitude;
    if (!gnss_init_) {
        gnss_process_->InitOrigin(lla);
        gnss_init_ = true;
    } else {
        gnss_process_->UpdateXYZYaw(lla);
        V3D enu = gnss_process_->enu_;
        V3D pos_cov = V3D(gnss_msg->position_covariance[0], gnss_msg->position_covariance[4],
                          gnss_msg->position_covariance[8]);
        // 将gnss转到imu坐标系
        V3D gnss_in_imu = system_ptr_->GetSystemConfig()->gnss2imu_ * enu;
        GNSS gnss(curr_gnss_time, gnss_in_imu, pos_cov);
        system_ptr_->AddGNSS(gnss);

        // 发布gnss odom
        nav_msgs::msg::Odometry gnss_odom;
        SetStamp(gnss_odom.header.stamp, curr_gnss_time);
        gnss_odom.header.frame_id = "odom";
        gnss_odom.child_frame_id = "gnss";
        gnss_odom.pose.pose.position.x = gnss_in_imu(0);
        gnss_odom.pose.pose.position.y = gnss_in_imu(1);
        gnss_odom.pose.pose.position.z = gnss_in_imu(2);
        Eigen::Quaterniond q(system_ptr_->GetCurentNavState().rot);
        q.normalize();
        gnss_odom.pose.pose.orientation.x = q.x();
        gnss_odom.pose.pose.orientation.y = q.y();
        gnss_odom.pose.pose.orientation.z = q.z();
        gnss_odom.pose.pose.orientation.w = q.w();
        if (gnss_odom_pub_->get_subscription_count() != 0) {
            gnss_odom_pub_->publish(gnss_odom);
        }
        enu = system_ptr_->GetGnssHeading() * enu;
        PoseTrans T_WG(M3D::Identity(), enu);
        PublishPath(gnss_path_pub_, gnss_path_, "odom", curr_gnss_time, T_WG);
    }
}

void ROS2Manager::Visualize() {
    PointCloudXYZIPtr gmap_snapshot;
    if (system_ptr_->GetLocalizer() != nullptr && system_ptr_->GetLocalizer()->GetGlobalMapSnapshot(gmap_snapshot)) {
        auto global_map = ToPointCloud2(gmap_snapshot, "map", last_visualize_time_);
        global_map_pub_->publish(global_map);
    }
    auto submap = ToPointCloud2(system_ptr_->GetSubmap(), "odom", last_visualize_time_);
    submap_pub_->publish(submap);

    // 系统尚未初始化完成则不发布里程计信息(可视化线程可能快于里程计线程)
    if (last_visualize_time_ == system_ptr_->GetSystemTime() && system_ptr_->IsSystemInit()) {
        return;
    }
    if (!system_ptr_->IsSystemInit()) {
        return;
    }
    last_visualize_time_ = system_ptr_->GetSystemTime();

    PublishTF(last_visualize_time_);
    PublishState(last_visualize_time_);

    auto current_state = system_ptr_->GetCurentNavState();
    PoseTrans T_WI(current_state.rot, current_state.pos);
    PublishPath(lio_path_pub_, lio_path_, "odom", last_visualize_time_, T_WI);
    if (has_encoder_) {
        PoseTrans T_WE = system_ptr_->GetTransformEncodeToWorld();
        PublishPath(encoder_path_pub_, encoder_path_, "odom", last_visualize_time_, T_WE);
    }
    PublishLidar(last_visualize_time_);
}

void ROS2Manager::PublishTF(const double &sensor_time) {
    PoseTrans T_OtoM;
    if (slam_mode_ == SLAM_MODE::LOCALIZATION) {
        T_OtoM = system_ptr_->GetLocalizer()->GetT_OtoM();
    }
    geometry_msgs::msg::TransformStamped tran_OM = GetTransformStamped(sensor_time, T_OtoM);
    tran_OM.header.frame_id = "map";
    tran_OM.child_frame_id = "odom";
    tf_broadcaster_->sendTransform(tran_OM);

    auto current_state = system_ptr_->GetCurentNavState();
    PoseTrans T_iInG(current_state.rot, current_state.pos);
    PoseTrans T_bInG = T_iInG * system_ptr_->GetImuToBaselink().inverse();
    geometry_msgs::msg::TransformStamped tran_OB = GetTransformStamped(sensor_time, T_bInG);
    tran_OB.header.frame_id = "odom";
    tran_OB.child_frame_id = "robot_link";
    tf_broadcaster_->sendTransform(tran_OB);

    geometry_msgs::msg::TransformStamped tran_LI = GetTransformStamped(sensor_time, system_ptr_->GetTLidarToImu(), true);
    tran_LI.header.frame_id = "lidar_link";
    tran_LI.child_frame_id = "imu_link";
    tf_broadcaster_->sendTransform(tran_LI);

    geometry_msgs::msg::TransformStamped tran_BL = GetTransformStamped(sensor_time, system_ptr_->GetLidarToBaselink());
    tran_BL.header.frame_id = "robot_link";
    tran_BL.child_frame_id = "lidar_link";
    tf_broadcaster_->sendTransform(tran_BL);
}

void ROS2Manager::PublishState(const double &sensor_time) {
    if (lio_odom_pub_->get_subscription_count() == 0) {
        return;
    }
    auto current_state = system_ptr_->GetCurentNavState();
    PoseTrans current_pose(current_state.rot, current_state.pos);
    PoseTrans robot_pose = current_pose * system_ptr_->GetImuToBaselink().inverse();
    nav_msgs::msg::Odometry odom;
    PoseTransToOdomMsg(robot_pose, odom);
    odom.header.frame_id = "odom";
    odom.child_frame_id = "robot_link";
    SetStamp(odom.header.stamp, sensor_time);
    V3D vel = current_state.rot.transpose() * current_state.vel;
    odom.twist.twist.linear.x = vel.x();
    odom.twist.twist.linear.y = vel.y();
    odom.twist.twist.linear.z = vel.z();
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            odom.pose.covariance[i * 6 + j] = system_ptr_->GetCov()(i, j);
        }
    }
    lio_odom_pub_->publish(odom);
}

void ROS2Manager::PublishLidar(const double &sensor_time) {
    auto cloud_lidar = ToPointCloud2(system_ptr_->GetCloudInLidarLink(), "lidar_link", sensor_time);
    cloud_lidar_pub_->publish(cloud_lidar);
    auto cloud_robot = ToPointCloud2(system_ptr_->GetCloudInRobotLink(), "robot_link", sensor_time);
    cloud_robot_pub_->publish(cloud_robot);
    auto cloud_odom = ToPointCloud2(system_ptr_->GetCloudInOdomLink(), "odom", sensor_time);
    cloud_odom_pub_->publish(cloud_odom);
    auto cloud_body = ToPointCloud2(system_ptr_->GetCloudInBodyLink(), "robot_link", sensor_time);
    cloud_body_pub_->publish(cloud_body);
}

void ROS2Manager::PoseTransToPoseStampedMsg(const PoseTrans &pose_trans, geometry_msgs::msg::PoseStamped &pose_msg) {
    pose_msg.pose.position.x = pose_trans.t.x();
    pose_msg.pose.position.y = pose_trans.t.y();
    pose_msg.pose.position.z = pose_trans.t.z();
    pose_msg.pose.orientation.x = pose_trans.eigen_q().x();
    pose_msg.pose.orientation.y = pose_trans.eigen_q().y();
    pose_msg.pose.orientation.z = pose_trans.eigen_q().z();
    pose_msg.pose.orientation.w = pose_trans.eigen_q().w();
}

void ROS2Manager::RosPoseToPoseTrans(const geometry_msgs::msg::Pose &pose_msg, PoseTrans &pose_trans) {
    pose_trans.t.x() = pose_msg.position.x;
    pose_trans.t.y() = pose_msg.position.y;
    pose_trans.t.z() = pose_msg.position.z;
    Eigen::Quaterniond q;
    q.x() = pose_msg.orientation.x;
    q.y() = pose_msg.orientation.y;
    q.z() = pose_msg.orientation.z;
    q.w() = pose_msg.orientation.w;
    q.normalize();
    pose_trans.R = q.toRotationMatrix();
}

void ROS2Manager::PoseTransToOdomMsg(const PoseTrans &pose_trans, nav_msgs::msg::Odometry &odom_msg) {
    odom_msg.pose.pose.position.x = pose_trans.t.x();
    odom_msg.pose.pose.position.y = pose_trans.t.y();
    odom_msg.pose.pose.position.z = pose_trans.t.z();
    odom_msg.pose.pose.orientation.x = pose_trans.eigen_q().x();
    odom_msg.pose.pose.orientation.y = pose_trans.eigen_q().y();
    odom_msg.pose.pose.orientation.z = pose_trans.eigen_q().z();
    odom_msg.pose.pose.orientation.w = pose_trans.eigen_q().w();
}

geometry_msgs::msg::TransformStamped ROS2Manager::GetTransformStamped(const double timestamp,
                                                                      const PoseTrans &transform, bool flip_trans) {
    PoseTrans T = transform;
    if (flip_trans) {
        T = transform.inverse();
    }
    geometry_msgs::msg::TransformStamped trans;
    SetStamp(trans.header.stamp, timestamp);
    trans.transform.rotation.x = T.eigen_q().x();
    trans.transform.rotation.y = T.eigen_q().y();
    trans.transform.rotation.z = T.eigen_q().z();
    trans.transform.rotation.w = T.eigen_q().w();
    trans.transform.translation.x = T.t.x();
    trans.transform.translation.y = T.t.y();
    trans.transform.translation.z = T.t.z();
    return trans;
}

sensor_msgs::msg::PointCloud2 ROS2Manager::ToPointCloud2(const PointCloudXYZIPtr &cloud, const std::string &frame_id,
                                                         double timestamp) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    if (cloud != nullptr && !cloud->empty()) {
        pcl::toROSMsg(*cloud, cloud_msg);
    }
    cloud_msg.header.frame_id = frame_id;
    if (timestamp <= 0) {
        SetStamp(cloud_msg.header.stamp, NowSec());
    } else {
        SetStamp(cloud_msg.header.stamp, timestamp);
    }
    return cloud_msg;
}

void ROS2Manager::PublishPath(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub, nav_msgs::msg::Path &path,
                              const std::string &frame_id, double sensor_time, const PoseTrans &pose_trans) {
    if (pub->get_subscription_count() == 0) {
        return;
    }
    SetStamp(path.header.stamp, sensor_time);
    geometry_msgs::msg::PoseStamped pose;
    PoseTransToPoseStampedMsg(pose_trans, pose);
    path.poses.push_back(pose);
    path.header.frame_id = frame_id;
    pub->publish(path);
    if (path.poses.size() > 10000) {
        // 为了不让内存增长
        path.poses.erase(path.poses.begin());
    }
}

void ROS2Manager::RosInitPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg) {
    LOG_INFO("get init pose (ROS2 /initialpose)");
    if (system_ptr_->GetLocalizer() == nullptr) {
        LOG_WARN("localizer not enabled (not in localization mode), ignore init pose");
        return;
    }
    PoseTrans pose;
    RosPoseToPoseTrans(pose_msg->pose.pose, pose);
    auto current_map = system_ptr_->GetLocalizer()->GetCurrMetaInfo();
    system_ptr_->SetInitPose(pose, current_map.level, current_map.name);
}

}  // namespace slam
