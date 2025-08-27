#include "front_end.hh"
#include "ieskf.hh"
#include "lidar_register/inc_ndt_register.hh"
#include "lidar_register/p2plane_register.hh"
#include "lidar_register/voxelmap_register.hh"
#include "propogator.hh"
#include "system.hh"
#include "system_config.hh"

namespace slam {

FrontEnd::FrontEnd(System* system) : system_(system) {
    LOG_INFO("FrontEnd init done!");
    use_encoder_ = system_->GetSystemConfig()->has_encoder_;
    use_gnss_ = system_->GetSystemConfig()->has_gnss_;

    // 坐标信息
    T_IL = system_->GetSystemConfig()->lidar2imu_;
    T_BL = system_->GetSystemConfig()->lidar2robot_;
    auto T_LI = T_IL.inverse();
    // T_BI = T_BL * T_LI;
    T_BI = (T_BL * T_LI);
    // ieskf
    kf_ptr_ = std::make_shared<IESKF>();
    // propogator
    propogator_ptr_ = std::make_shared<Propogator>(system_->GetSystemConfig(), kf_ptr_);
    AllocateMemory();
    // voxel_map_odom
    if (system_->GetSystemConfig()->use_voxel_) {
        lidar_register_ptr_ = std::make_shared<VoxelMapRegister>(system_->GetSystemConfig(), kf_ptr_);
    } else if (system_->GetSystemConfig()->use_p2plane_) {
        lidar_register_ptr_ = std::make_shared<P2PlaneRegister>(system_->GetSystemConfig(), kf_ptr_);
    } else if (system_->GetSystemConfig()->use_ndt_) {
        lidar_register_ptr_ = std::make_shared<IncNdtRegister>(system_->GetSystemConfig(), kf_ptr_);
    }
}

void FrontEnd::AllocateMemory() {
    undistort_cloud_lidar_.reset(new PointCloudType);
    undistort_cloud_robot_.reset(new PointCloudType);
    undistort_cloud_odom_.reset(new PointCloudType);
}

FrontEnd::~FrontEnd() {
}

void FrontEnd::Run() {
    LOG_INFO("FrontEndThread Run!");
    while (ros::ok()) {
        // 获取锁
        std::unique_lock<std::mutex> lock(system_->m_buf_mutex_);
        // 因为需要在阻塞时释放锁，唤醒时重新获取锁。
        // 唤醒条件lamda表达式为true,或者超时被唤醒，返回false
        // 存在虚假唤醒，需要判断条件是否满足
        if (system_->m_buff_cv_.wait_for(lock, std::chrono::milliseconds(1000), [&] {
                // LOG_INFO("lidar_queue size: {}", system_->lidar_queue_.size());
                return !system_->lidar_queue_.empty();
            })) {
            // 验证了线程的唤醒条件是正确的
            // system_->lidar_queue_.pop_front();
            // system_->lidar_time_queue_.pop_front();
            // LOG_INFO("GetMeasureGroup success!");
            // get measurement
            MeasureGroup meas;
            if (GetMeasureGroup(meas)) {
                LOG_INFO("GetMeasureGroup success!");
                measure_group_ = meas;
                if (front_end_status_ == FrontEndStatus::IMU_INIT) {
                    // 静态初始化
                    if (!propogator_ptr_->GetInitSuccess()) {
                        if (propogator_ptr_->Initialize(measure_group_)) {
                            front_end_status_ = FrontEndStatus::MAP_INIT;
                            system_->SetSystemInit(true);
                            LOG_INFO("IMU_INIT!");
                            kf_ptr_->GetState().Print();
                        }
                    }
                    continue;
                }
                // 轮速计和gnss更新
                // 状态递推以及对雷达去畸变
                undistort_cloud_lidar_->clear();
                evaluate_and_call(
                    [&]() { propogator_ptr_->PropogateAndUndistort(measure_group_, undistort_cloud_lidar_); },
                    "propogate_and_undistort", false);
                // transform to robot_link
                undistort_cloud_robot_->clear();
                undistort_cloud_robot_ = TransformLidarOMP(undistort_cloud_lidar_, T_BL.R, T_BL.t);
                // transform to world
                undistort_cloud_odom_->clear();
                auto current_pose = PoseTrans(kf_ptr_->GetState().r_wi, kf_ptr_->GetState().t_wi);
                auto T_WL = current_pose * T_IL;
                undistort_cloud_odom_ = TransformLidarOMP(undistort_cloud_lidar_, T_WL.R, T_WL.t);
                if (front_end_status_ == FrontEndStatus::MAP_INIT) {
                    if (lidar_register_ptr_->InitMap(undistort_cloud_lidar_, kf_ptr_)) {
                        front_end_status_ = FrontEndStatus::MAPPING;
                    }
                    continue;
                }
                if (front_end_status_ == FrontEndStatus::MAPPING) {
                    // LOG_INFO("befor state: \n");
                    // kf_ptr_->GetState().Print();
                    auto t1 = std::chrono::high_resolution_clock::now();
                    if (lidar_register_ptr_->Align(undistort_cloud_lidar_, kf_ptr_)) {
                        LOG_INFO("Align Success");
                        lidar_register_ptr_->UpdateMap();
                        // LOG_INFO("after state: \n");
                        // kf_ptr_->GetState().Print();
                        auto t2 = std::chrono::high_resolution_clock::now();
                        auto total_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
                        LOG_INFO("Align and Update:{} ms", total_time * 1e3);
                    } else {
                        front_end_status_ = FrontEndStatus::LOST;
                    }
                }
                if (front_end_status_ == FrontEndStatus::LOST) {
                    // 丢失后，需要重新初始化
                    LOG_INFO("Lost, reinit");
                    continue;
                }
            }
        } else {
            continue;
        }
    }
}

bool FrontEnd::GetMeasureGroup(MeasureGroup& measures) {
    // 如果队列为空
    if (system_->imu_queue_.empty() || system_->lidar_queue_.empty() || system_->lidar_time_queue_.empty()) {
        return false;
    }
    // 使用轮速计，但是轮速计的队列为空
    if (use_encoder_ && system_->encoder_queue_.empty()) {
        opt_wheel_ = false;
    }
    // 使用gnss,但是gnss的队列为空
    if (use_gnss_ && system_->gnss_queue_.empty()) {
        opt_gnss_ = false;
    }
    // 这里lidar点是每个utc时间戳
    if (!lidar_pushed_) {
        measures.curent_cloud = system_->lidar_queue_.front();
        measures.lidar_beg_time = system_->lidar_time_queue_.front();
        // sort cloud
        std::sort(measures.curent_cloud->points.begin(), measures.curent_cloud->points.end(),
                  [](const PointType& a, const PointType& b) { return a.time < b.time; });
        if (measures.curent_cloud->size() < 1) {
            measures.lidar_end_time = measures.lidar_beg_time + lidar_mean_scantime_;
            LOG_ERROR("lidar cloud size is 0, begin time is {}, end time is {}", measures.lidar_beg_time,
                      measures.lidar_end_time);
        } else if (measures.curent_cloud->points.back().time < 0.5 * lidar_mean_scantime_) {
            measures.lidar_end_time = measures.lidar_beg_time + lidar_mean_scantime_;
            LOG_ERROR("lidar cloud end time is too small, begin time is {}, end time is {}", measures.lidar_beg_time,
                      measures.lidar_end_time);
        } else {
            scan_count_++;
            measures.lidar_end_time = measures.curent_cloud->points.back().time;
            lidar_mean_scantime_ +=
                (measures.lidar_end_time - measures.lidar_beg_time - lidar_mean_scantime_) / scan_count_;
        }
        if (lidar_mean_scantime_ > 0.1) {
            LOG_WARN("lidar mean scan time is too large, mean scan time is {}", lidar_mean_scantime_);
        }
        lidar_pushed_ = true;
        // LOG_INFO("lidar cloud size is {}, begin time is {}, end time is {}, mean scan time is {}",
        //         measures.curent_cloud->size(), measures.lidar_beg_time, measures.lidar_end_time,
        //         lidar_mean_scantime_);
    }
    // 处理imu数据
    double imu_time = system_->imu_queue_.front().timestamp_;
    while (!system_->imu_queue_.empty() && imu_time < measures.lidar_end_time) {
        measures.imus.push_back(system_->imu_queue_.front());
        system_->imu_queue_.pop_front();
        imu_time = system_->imu_queue_.front().timestamp_;
    }
    // LOG_INFO("imu size is {}, imu begin_time {}, imu_end_time {}", measures.imus.size(),
    //          measures.imus.front().timestamp_, measures.imus.end()->timestamp_);
    // 处理encoder数据
    double encoder_time = system_->encoder_queue_.front().timestamp_;
    if (use_encoder_) {
        while (!system_->encoder_queue_.empty() && encoder_time < measures.lidar_end_time) {
            measures.encoders.push_back(system_->encoder_queue_.front());
            system_->encoder_queue_.pop_front();
            encoder_time = system_->encoder_queue_.front().timestamp_;
        }
        // LOG_INFO("encoder size is {}, encoder begin_time {}, encoder_end_time {}", measures.encoders.size(),
        //          measures.encoders.front().timestamp_, measures.encoders.end()->timestamp_);
    }
    // 处理gnss数据
    double gnss_time = system_->gnss_queue_.front().timestamp_;
    if (use_gnss_) {
        while (!system_->gnss_queue_.empty() && gnss_time < measures.lidar_end_time) {
            measures.gnsss.push_back(system_->gnss_queue_.front());
            system_->gnss_queue_.pop_front();
            gnss_time = system_->gnss_queue_.front().timestamp_;
        }
        LOG_INFO("gnss size is {}, gnss begin_time {}, gnss_end_time {}", measures.gnsss.size(),
                 measures.gnsss.front().timestamp_, measures.gnsss.end()->timestamp_);
    }

    // 处理gnss数据
    system_->lidar_queue_.pop_front();
    system_->lidar_time_queue_.pop_front();
    lidar_pushed_ = false;
    if (measures.imus.empty()) {
        return false;
    }
    return true;
}

NavState FrontEnd::GetCurentNavState() {
    return kf_ptr_->GetState();
}

// lidar坐标系原始数据
const PointCloudPtr FrontEnd::GetCloudInLidarLink() const {
    return undistort_cloud_lidar_;
}

// robot_link坐标系点云
const PointCloudPtr FrontEnd::GetCloudInRobotLink() const {
    return undistort_cloud_robot_;
}

// odom坐标系点云
const PointCloudPtr FrontEnd::GetCloudInOdomLink() const {
    return undistort_cloud_odom_;
}

}  // namespace slam
