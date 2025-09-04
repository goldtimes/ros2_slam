#include "ros/ros1_manager.hh"
#include "geometry_msgs/PoseStamped.h"
#include "lidar_register/voxel_map.hh"
#include "ros/time.h"
#include "system.hh"
#include "system_config.hh"
#include "utils.hh"
namespace slam {

ROS1Manager::ROS1Manager(const ros::NodeHandle& nh, std::shared_ptr<System> system_ptr)
    : nh_(nh), system_ptr_(system_ptr) {
    LOG_INFO("ROS1Manager init");

    has_encoder_ = system_ptr_->GetSystemConfig()->has_encoder_;
    has_gnss_ = system_ptr_->GetSystemConfig()->has_gnss_;
    if (has_gnss_) {
        gnss_process_ = std::make_shared<GnssProcess>();
    }
    InitPub();
    InitSub();
    InitService();

    visualize_thread_ = std::thread(&ROS1Manager::Visualize, this);

    voxel_map_timer_ = nh_.createTimer(ros::Duration(5.0), &ROS1Manager::voxelTimerCB, this, false, false);
    if (system_ptr_->GetSystemConfig()->frontend_config_.voxel_config.pub_voxel_map) {
        voxel_map_timer_.start();
    }
}
ROS1Manager::~ROS1Manager() {
    if (visualize_thread_.joinable()) {
        visualize_thread_.join();
    }
}

void ROS1Manager::voxelTimerCB(const ros::TimerEvent& event) {
    // auto voxel_map = system_ptr_->GetVoxelMap();
}

void ROS1Manager::InitPub() {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();

    cloud_lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lie_slam/lidar", 10);
    cloud_robot_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lie_slam/robot_lidar", 10);
    cloud_odom_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lie_slam/world_lidar", 10);

    lio_path_pub_ = nh_.advertise<nav_msgs::Path>("/lie_slam/lio_path", 10);
    lio_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/lie_slam/lio_odom", 10);
    if (has_encoder_) {
        encoder_path_pub_ = nh_.advertise<nav_msgs::Path>("/lie_slam/encoder_path", 10);
    }
    if (has_gnss_) {
        gnss_path_pub_ = nh_.advertise<nav_msgs::Path>("/lie_slam/gnss_path", 10);
        gnss_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/lie_slam/gnss_odom", 10);
    }
}

void ROS1Manager::InitSub() {
    imu_sub_ = nh_.subscribe(system_ptr_->GetSystemConfig()->imu_config_.imu_topic, 100, &ROS1Manager::ImuCallback,
                             this, ros::TransportHints().tcpNoDelay());
    if (system_ptr_->GetSystemConfig()->lidar_config_.use_livox_driver == 0) {
        lidar_sub_ = nh_.subscribe(system_ptr_->GetSystemConfig()->lidar_config_.lidar_topic, 100,
                                   &ROS1Manager::StandarCloudCallback, this, ros::TransportHints().tcpNoDelay());
        LOG_INFO("use standard lidar driver");
    } else if (system_ptr_->GetSystemConfig()->lidar_config_.use_livox_driver == 1) {
        lidar_sub_ = nh_.subscribe(system_ptr_->GetSystemConfig()->lidar_config_.lidar_topic, 10,
                                   &ROS1Manager::LivoxCloudCallback, this, ros::TransportHints().tcpNoDelay());
        LOG_INFO("use livox driver 1");

    } else if (system_ptr_->GetSystemConfig()->lidar_config_.use_livox_driver == 2) {
        lidar_sub_ = nh_.subscribe(system_ptr_->GetSystemConfig()->lidar_config_.lidar_topic, 10,
                                   &ROS1Manager::Livox2CloudCallback, this, ros::TransportHints().tcpNoDelay());
        LOG_INFO("use livox driver 2");
    } else {
        LOG_ERROR("use_livox_driver must be 0, 1 or 2!");
        std::exit(1);
    }
    if (has_encoder_) {
        encoder_sub_ = nh_.subscribe(system_ptr_->GetSystemConfig()->encoder_config_.encoder_topic, 100,
                                     &ROS1Manager::EncoderCallback, this, ros::TransportHints().tcpNoDelay());
    }
    if (has_gnss_) {
        gnss_sub_ = nh_.subscribe(system_ptr_->GetSystemConfig()->gnss_config_.gnss_topic, 100,
                                  &ROS1Manager::GNSSCallback, this, ros::TransportHints().tcpNoDelay());
    }
}
void ROS1Manager::InitService() {
}

void ROS1Manager::StandarCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    static double last_record_lidar_time = cloud_msg->header.stamp.toSec();
    double curr_lidar_time = cloud_msg->header.stamp.toSec();
    // 统计fps
    if (curr_lidar_time - last_record_lidar_time >= 5.0) {
        lidar_fps_ = static_cast<int>(lidar_frame_count_ / 5);
        LOG_INFO(GREEN "receive lidar fps:{}" RESET, lidar_fps_);
        lidar_frame_count_ = 0;
        last_record_lidar_time = curr_lidar_time;
    }
    // 检测是否有数据丢失
    if (last_lidar_time_ != -1 && curr_lidar_time - last_lidar_time_ > 0.2) {
        LOG_ERROR("lidar data lost");
        LOG_ERROR("current_lidar_time:{}, last_lidar_time:{}", curr_lidar_time, last_lidar_time_);
    }
    last_lidar_time_ = curr_lidar_time;
    lidar_frame_count_++;
    // 需要在这里处理lidar数据
    PointCloudPtr cloud_ptr(new PointCloudType);
    evaluate_and_call([&]() { system_ptr_->GetLidarProcess()->Process(cloud_msg, cloud_ptr); }, "lidar_process");
    // push to system
    system_ptr_->AddLidar(cloud_ptr, curr_lidar_time);
}

void ROS1Manager::Livox2CloudCallback(const livox_ros_driver2::CustomMsg::ConstPtr& cloud_livox) {
    static double last_record_lidar_time = cloud_livox->header.stamp.toSec();
    double curr_lidar_time = cloud_livox->header.stamp.toSec();
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
    // 需要在这里处理lidar数据
    PointCloudPtr cloud_ptr(new PointCloudType);
    evaluate_and_call([&]() { system_ptr_->GetLidarProcess()->Process(cloud_livox, cloud_ptr); }, "lidar_process");
    // push to system
    system_ptr_->AddLidar(cloud_ptr, curr_lidar_time);
}
void ROS1Manager::LivoxCloudCallback(const livox_ros_driver::CustomMsg::ConstPtr& cloud_livox) {
    static double last_record_lidar_time = cloud_livox->header.stamp.toSec();
    double curr_lidar_time = cloud_livox->header.stamp.toSec();
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
    // 需要在这里处理lidar数据
    PointCloudPtr cloud_ptr(new PointCloudType);
    evaluate_and_call([&]() { system_ptr_->GetLidarProcess()->Process(cloud_livox, cloud_ptr); }, "lidar_process");
    // push to system
    system_ptr_->AddLidar(cloud_ptr, curr_lidar_time);
}
void ROS1Manager::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    static double last_record_imu_time = imu_msg->header.stamp.toSec();
    double curr_imu_time = imu_msg->header.stamp.toSec();
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
    // push to system
    system_ptr_->AddIMU(imu);
}
void ROS1Manager::EncoderCallback(const nav_msgs::Odometry::ConstPtr& encoder_msg) {
    static double last_record_encoder_time = encoder_msg->header.stamp.toSec();
    double curr_encoder_time = encoder_msg->header.stamp.toSec();
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
    // std::cout << "encoder: " << encoder << std::endl;
    // push to system
    system_ptr_->AddEncoder(encoder);
}
// void AddLidar(const Lidar& lidar);
void ROS1Manager::GNSSCallback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msg) {
    // LOG_INFO("gnss");
    static double last_record_gnss_time = gnss_msg->header.stamp.toSec();
    double curr_gnss_time = gnss_msg->header.stamp.toSec();
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

    // 不是固定解，返回
    // if (gnss_msg->status.status != sensor_msgs::NavSatStatus::STATUS_FIX) {
    //     return;
    // }

    V3D lla;
    lla << gnss_msg->latitude, gnss_msg->longitude, gnss_msg->altitude;
    if (!gnss_init_) {
        gnss_process_->InitOrigin(lla);
        gnss_init_ = true;
    } else {
        gnss_process_->UpdateXYZYaw(lla);
        V3D enu = gnss_process_->enu_;
        V3D pos_cov =
            V3D(gnss_msg->position_covariance[0], gnss_msg->position_covariance[4], gnss_msg->position_covariance[8]);
        GNSS gnss(curr_gnss_time, enu, pos_cov);
        // pub gnss path
        system_ptr_->AddGNSS(gnss);
        // 将gnss转到imu坐标系
        V3D gnss_in_imu = system_ptr_->GetSystemConfig()->gnss2imu_ * enu;
        // 发布gnss odom
        nav_msgs::Odometry gnss_odom;
        gnss_odom.header.stamp = ros::Time(curr_gnss_time);
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
        if (gnss_odom_pub_.getNumSubscribers() != 0) {
            gnss_odom_pub_.publish(gnss_odom);
        }
        enu = system_ptr_->GetGnssHeading() * enu;
        PoseTrans T_WG(M3D::Identity(), enu);
        PublishPath(gnss_path_pub_, gnss_path_, "odom", curr_gnss_time, T_WG);
    }
}

void ROS1Manager::Visualize() {
    ros::Rate rate(50);
    while (ros::ok()) {
        rate.sleep();

        // 系统以及初始化完成后，但是还在处理雷达消息，可视化的线程要比里程计的线程快
        if (last_visualize_time_ == system_ptr_->GetSystemTime() && system_ptr_->IsSystemInit()) {
            continue;
        }
        // 系统为初始化，则不发布可视化信息
        if (!system_ptr_->IsSystemInit()) {
            continue;
        }
        last_visualize_time_ = system_ptr_->GetSystemTime();

        PublishTF(last_visualize_time_);
        // pub odom
        PublishState(last_visualize_time_);
        // pub path
        auto current_state = system_ptr_->GetCurentNavState();
        PoseTrans T_WI(current_state.rot, current_state.pos);
        PublishPath(lio_path_pub_, lio_path_, "odom", last_visualize_time_, T_WI);
        if (has_encoder_) {
            PoseTrans T_WE = system_ptr_->GetTransformEncodeToWorld();
            PublishPath(encoder_path_pub_, encoder_path_, "odom", last_visualize_time_, T_WE);
        }
        PublishLidar(last_visualize_time_);
    }
}

void ROS1Manager::PublishTF(const double& sensor_time) {
    // 发布robot_link在odom的tf信息
    auto current_state = system_ptr_->GetCurentNavState();
    PoseTrans T_iInG(current_state.rot, current_state.pos);
    PoseTrans T_bInG = T_iInG * system_ptr_->GetImuToBaselink().inverse();
    geometry_msgs::TransformStamped tran_OB = GetTransformStamped(sensor_time, T_bInG);
    tran_OB.header.frame_id = "odom";
    tran_OB.child_frame_id = "robot_link";
    tf_broadcaster_->sendTransform(tran_OB);
    // 发布lidar到imu的tf信息
    geometry_msgs::TransformStamped tran_LI = GetTransformStamped(sensor_time, system_ptr_->GetTLidarToImu(), true);
    tran_LI.header.frame_id = "lidar_link";
    tran_LI.child_frame_id = "imu_link";
    tf_broadcaster_->sendTransform(tran_LI);
    // 发布lidar到robot_link的tf信息
    geometry_msgs::TransformStamped tran_BL = GetTransformStamped(sensor_time, system_ptr_->GetLidarToBaselink());
    tran_BL.header.frame_id = "robot_link";
    tran_BL.child_frame_id = "lidar_link";
    tf_broadcaster_->sendTransform(tran_BL);
}

void ROS1Manager::PublishState(const double& sensor_time) {
    if (lio_odom_pub_.getNumSubscribers() != 0) {
        auto current_state = system_ptr_->GetCurentNavState();
        PoseTrans current_pose(current_state.rot, current_state.pos);
        PoseTrans robot_pose = current_pose * system_ptr_->GetImuToBaselink().inverse();
        nav_msgs::Odometry odom;
        PoseTransToOdomMsg(robot_pose, odom);
        odom.header.frame_id = "odom";
        odom.child_frame_id = "robot_link";
        odom.header.stamp = ros::Time(sensor_time);
        V3D vel = current_state.rot.transpose() * current_state.vel;
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();
        // odom.pose.covariance
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                odom.pose.covariance[i * 6 + j] = system_ptr_->GetCov()(i, j);
            }
        }
        lio_odom_pub_.publish(odom);
    }
    // if (lio_path_pub_.getNumSubscribers() != 0) {
    //     lio_path_.header.stamp = ros::Time(sensor_time);
    //     auto current_state = system_ptr_->GetCurentNavState();
    //     PoseTrans current_pose(current_state.rot, current_state.pos);
    //     geometry_msgs::PoseStamped pose;
    //     PoseTransToPoseStampedMsg(current_pose, pose);
    //     lio_path_.poses.push_back(pose);
    //     lio_path_.header.frame_id = "odom";
    //     lio_path_pub_.publish(lio_path_);
    //     if (lio_path_.poses.size() > 10000) {
    //         // 为了不让内存增长
    //         lio_path_.poses.erase(lio_path_.poses.begin());
    //     }
    // }
}

void ROS1Manager::PublishLidar(const double& sensor_time) {
    auto cloud_lidar = ToPointCloud2(system_ptr_->GetCloudInLidarLink(), "lidar_link", sensor_time);
    cloud_lidar_pub_.publish(cloud_lidar);
    auto cloud_robot = ToPointCloud2(system_ptr_->GetCloudInRobotLink(), "robot_link", sensor_time);
    cloud_robot_pub_.publish(cloud_robot);
    auto cloud_odom = ToPointCloud2(system_ptr_->GetCloudInOdomLink(), "odom", sensor_time);
    cloud_odom_pub_.publish(cloud_odom);
}

void ROS1Manager::PoseTransToPoseStampedMsg(const PoseTrans& pose_trans, geometry_msgs::PoseStamped& pose_msg) {
    pose_msg.pose.position.x = pose_trans.t.x();
    pose_msg.pose.position.y = pose_trans.t.y();
    pose_msg.pose.position.z = pose_trans.t.z();
    pose_msg.pose.orientation.x = pose_trans.eigen_q().x();
    pose_msg.pose.orientation.y = pose_trans.eigen_q().y();
    pose_msg.pose.orientation.z = pose_trans.eigen_q().z();
    pose_msg.pose.orientation.w = pose_trans.eigen_q().w();
}
void ROS1Manager::PoseTransToOdomMsg(const PoseTrans& pose_trans, nav_msgs::Odometry& odom_msg) {
    odom_msg.pose.pose.position.x = pose_trans.t.x();
    odom_msg.pose.pose.position.y = pose_trans.t.y();
    odom_msg.pose.pose.position.z = pose_trans.t.z();
    odom_msg.pose.pose.orientation.x = pose_trans.eigen_q().x();
    odom_msg.pose.pose.orientation.y = pose_trans.eigen_q().y();
    odom_msg.pose.pose.orientation.z = pose_trans.eigen_q().z();
    odom_msg.pose.pose.orientation.w = pose_trans.eigen_q().w();
}

geometry_msgs::TransformStamped ROS1Manager::GetTransformStamped(const double timestamp, const PoseTrans& transform,
                                                                 bool flip_trans) {
    PoseTrans T = transform;
    if (flip_trans) {
        T = transform.inverse();
    }
    geometry_msgs::TransformStamped trans;
    trans.header.stamp = ros::Time(timestamp);
    trans.transform.rotation.x = T.eigen_q().x();
    trans.transform.rotation.y = T.eigen_q().y();
    trans.transform.rotation.z = T.eigen_q().z();
    trans.transform.rotation.w = T.eigen_q().w();
    trans.transform.translation.x = T.t.x();
    trans.transform.translation.y = T.t.y();
    trans.transform.translation.z = T.t.z();
    return trans;
}

sensor_msgs::PointCloud2 ROS1Manager::ToPointCloud2(const PointCloudPtr& cloud, const std::string& frame_id,
                                                    double timestamp) {
    sensor_msgs::PointCloud2 cloud_msg;
    if (!cloud->empty()) {
        pcl::toROSMsg(*cloud, cloud_msg);
    }
    cloud_msg.header.frame_id = frame_id;
    if (timestamp <= 0) {
        cloud_msg.header.stamp = ros::Time::now();
    } else {
        cloud_msg.header.stamp = ros::Time(timestamp);
    }
    return cloud_msg;
}

void ROS1Manager::PublishPath(const ros::Publisher pub, nav_msgs::Path& path, const std::string& frame_id,
                              double sensor_time, const PoseTrans& pose_trans) {
    if (pub.getNumSubscribers() != 0) {
        path.header.stamp = ros::Time(sensor_time);
        geometry_msgs::PoseStamped pose;
        PoseTransToPoseStampedMsg(pose_trans, pose);
        path.poses.push_back(pose);
        path.header.frame_id = frame_id;
        pub.publish(path);
        if (path.poses.size() > 10000) {
            // 为了不让内存增长
            path.poses.erase(path.poses.begin());
        }
    }
}
}  // namespace slam