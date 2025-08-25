#include "ros/ros1_manager.hh"
#include "system.hh"
#include "system_config.hh"
#include "utils.hh"
namespace slam {

ROS1Manager::ROS1Manager(const ros::NodeHandle& nh, std::shared_ptr<System> system_ptr)
    : nh_(nh), system_ptr_(system_ptr) {
    LOG_INFO("ROS1Manager init");
    InitPub();
    InitSub();
    InitService();

    visualize_thread_ = std::thread(&ROS1Manager::Visualize, this);
}
ROS1Manager::~ROS1Manager() {
    if (visualize_thread_.joinable()) {
        visualize_thread_.join();
    }
}

void ROS1Manager::InitPub() {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();

    cloud_lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lie_slam/lidar", 10);
    cloud_robot_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lie_slam/robot_lidar", 10);
    cloud_odom_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lie_slam/world_lidar", 10);
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
    if (system_ptr_->GetSystemConfig()->has_encoder_) {
        encoder_sub_ = nh_.subscribe(system_ptr_->GetSystemConfig()->encoder_config_.encoder_topic, 100,
                                     &ROS1Manager::EncoderCallback, this, ros::TransportHints().tcpNoDelay());
    }
    if (system_ptr_->GetSystemConfig()->has_gnss_) {
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
    // push to system
    system_ptr_->AddEncoder(encoder);
}
// void AddLidar(const Lidar& lidar);
void ROS1Manager::GNSSCallback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msg) {
    static double last_record_gnss_time = gnss_msg->header.stamp.toSec();
    double curr_gnss_time = gnss_msg->header.stamp.toSec();
    if (curr_gnss_time - last_record_gnss_time >= 5.0) {
        gnss_fps_ = static_cast<int>(gnss_frame_count_ / 5);
        LOG_INFO(GREEN "receive gnss fps:{}" RESET, gnss_fps_);
        gnss_frame_count_ = 0;
        last_record_gnss_time = curr_gnss_time;
    }
    if (last_gnss_time_ != -1 && curr_gnss_time - last_gnss_time_ > 0.2) {
        LOG_ERROR("gnss data lost");
        LOG_ERROR("current_gnss_time:{}, last_gnss_time:{}", curr_gnss_time, last_gnss_time_);
    }
    last_gnss_time_ = curr_gnss_time;
    gnss_frame_count_++;
    V3D lla;
    lla << gnss_msg->latitude, gnss_msg->longitude, gnss_msg->altitude;
    GNSS gnss(curr_gnss_time, lla);
    // push to system
    system_ptr_->AddGNSS(gnss);
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
        // PublishState(last_visualize_time_);
        PublishLidar(last_visualize_time_);
    }
}

void ROS1Manager::PublishTF(const double& sensor_time) {
    // 发布robot_link在odom的tf信息
    auto current_state = system_ptr_->GetCurentNavState();
    SE3 T_iInG(current_state.r_wi, current_state.t_wi);
    SE3 T_bInG = T_iInG * system_ptr_->GetImuToBaselink().inverse();
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
}

void ROS1Manager::PublishLidar(const double& sensor_time) {
    auto cloud_lidar = ToPointCloud2(system_ptr_->GetCloudInLidarLink(), "lidar_link", sensor_time);
    cloud_lidar_pub_.publish(cloud_lidar);
    auto cloud_robot = ToPointCloud2(system_ptr_->GetCloudInRobotLink(), "robot_link", sensor_time);
    cloud_robot_pub_.publish(cloud_robot);
    auto cloud_odom = ToPointCloud2(system_ptr_->GetCloudInOdomLink(), "odom", sensor_time);
    cloud_odom_pub_.publish(cloud_odom);
}

geometry_msgs::TransformStamped ROS1Manager::GetTransformStamped(const double timestamp, const SE3& transform,
                                                                 bool flip_trans) {
    SE3 T = transform;
    if (flip_trans) {
        T = transform.inverse();
    }
    geometry_msgs::TransformStamped trans;
    trans.header.stamp = ros::Time(timestamp);
    trans.transform.rotation.x = T.so3().unit_quaternion().x();
    trans.transform.rotation.y = T.so3().unit_quaternion().y();
    trans.transform.rotation.z = T.so3().unit_quaternion().z();
    trans.transform.rotation.w = T.so3().unit_quaternion().w();
    trans.transform.translation.x = T.translation().x();
    trans.transform.translation.y = T.translation().y();
    trans.transform.translation.z = T.translation().z();
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
}  // namespace slam