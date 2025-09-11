#include "front_end.hh"
#include "encoder_process.hh"
#include "ieskf.hh"
#include "lidar_register/inc_ndt_register.hh"
#include "lidar_register/p2plane_register.hh"
#include "lidar_register/voxelmap_register.hh"
#include "localizer/localizer.hh"
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
    T_BI = (T_BL * T_LI);
    T_EI = system_->GetSystemConfig()->imu2encoder_;
    // ieskf
    kf_ptr_ = std::make_shared<IESKF>();

    if (use_encoder_) {
        encoder_processor_ptr_ = std::make_shared<EncoderProcessor>(system_->GetSystemConfig());
        encoder_processor_ptr_->SetTransformWheelToImu(T_EI);
        kf_ptr_->SetWheelLossFunc(
            [&](const Encoder& encoder, const Input& input, State& x, ESKFShareState& share_state) {
                encoder_processor_ptr_->UpdateEncoder(encoder, input, x, share_state);
            });
        // 设置encoder外参
        // 保证旋转的正交性，因为sophus对矩阵要求很严格 1 0 -2.77515e-17 0 1 0 0 -2.77515e-17  0  1 这样的矩阵会报错
        kf_ptr_->x().rot_R_IE = Eigen::Quaterniond(T_EI.inverse().R).toRotationMatrix();
        kf_ptr_->x().pos_t_IE = T_EI.inverse().t;
    }

    if (use_gnss_) {
        kf_ptr_->SetGnssLossFunc(
            [&](const GNSS& gnss, State& x, ESKFShareState& share_state) { UpdateGnss(gnss, x, share_state); });
    }

    // propogator
    propogator_ptr_ = std::make_shared<Propogator>(system_->GetSystemConfig(), kf_ptr_);
    propogator_ptr_->SetTransformWheelToImu(T_EI);
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
    undistort_cloud_lidar_.reset(new PointCloudXYZI);
    undistort_cloud_robot_.reset(new PointCloudXYZI);
    undistort_cloud_odom_.reset(new PointCloudXYZI);
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
                return !system_->imu_queue_.empty();
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
                            // 初始化轮速计的朝向
                            PoseTrans T_WI(kf_ptr_->GetState().rot, kf_ptr_->GetState().pos);
                            T_WE = T_WI * T_EI.inverse();
                            kf_ptr_->GetState().Print();
                        }
                    }
                    continue;
                }

                // imu的前向传播
                undistort_cloud_lidar_->clear();
                PoseTrans start_pose = PoseTrans(kf_ptr_->GetState().rot, kf_ptr_->GetState().pos);
                evaluate_and_call([&]() { propogator_ptr_->PropogateState(meas); }, "propogate_and_undistort", false);
                PoseTrans end_pose = PoseTrans(kf_ptr_->GetState().rot, kf_ptr_->GetState().pos);
                PoseTrans state_delta_pose = end_pose * start_pose.inverse();
                // LOG_INFO("state_delta_pose t_norm: {}, rotation_norm:{}", state_delta_pose.t.norm(),
                //          state_delta_pose.RPY().norm());
                // 对轮速计进行积分，对首尾进行插值
                if (use_encoder_) {
                    encoder_processor_ptr_->AddEncoder(measure_group_.encoders);
                    PoseTrans encoder_delta_pose;
                    auto res = encoder_processor_ptr_->Propagation(encoder_delta_pose, measure_group_.lidar_beg_time,
                                                                   measure_group_.lidar_end_time);
                    if (res) {
                        // LOG_INFO("encoder_delta_pose t_norm: {}, rotation_norm:{}", encoder_delta_pose.t.norm(),
                        //          encoder_delta_pose.RPY().norm());
                    }
                    // 更新TWE
                    T_WE = T_WE * encoder_delta_pose;
                }

                propogator_ptr_->UndistortLidar(measure_group_, undistort_cloud_lidar_);
                // transform to robot_link
                undistort_cloud_robot_->clear();
                undistort_cloud_robot_ = TransformLidarOMP(undistort_cloud_lidar_, T_BL.R, T_BL.t);
                // transform to world
                undistort_cloud_odom_->clear();
                auto current_pose = PoseTrans(kf_ptr_->GetState().rot, kf_ptr_->GetState().pos);
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
                    static int frame_id = 0;
                    auto t1 = std::chrono::high_resolution_clock::now();
                    if (lidar_register_ptr_->Align(undistort_cloud_lidar_, kf_ptr_)) {
                        auto t2 = std::chrono::high_resolution_clock::now();
                        auto align_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
                        LOG_INFO("Align used time: {} ms", align_time * 1e3);
                        // LOG_INFO("Align Success");
                        lidar_register_ptr_->UpdateMap();
                        // get current cloud
                        PointCloudPtr world_cloud(new PointCloudType);
                        // auto t3 = std::chrono::high_resolution_clock::now();
                        world_cloud = lidar_register_ptr_->GetSubmap();
                        // pcl::io::savePCDFileASCII("/home/kilox/fast_lvio_ws/src/open_slam/PCD/world_cloud_" +
                        //                               std::to_string(frame_id) + ".pcd",
                        //                           *world_cloud);
                        frame_id++;
                        // 根据关键帧的生成来通知后台配准线程
                        if (lidar_register_ptr_->IsKeyFrame() || first_frame_) {
                            LOG_INFO("KeyFrame");
                            // set submap to localizer
                            // localizer_ptr_->SetSubmap(world_cloud);
                            // system_->GetLocalizer()->SetSubmapCloud(world_cloud, T_WL);
                            // first_frame_ = false;
                        }
                        // 传入robot坐标下的点云和robot在odom下的坐标
                        // auto T_RtoO = T_WL * T_BL.inverse();
                        // system_->GetLocalizer()->SetLidarCloud(undistort_cloud_robot_, T_RtoO);
                        // auto t4 = std::chrono::high_resolution_clock::now();
                        // auto total_time = std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3).count();
                        // LOG_INFO("Get Map used time: {} ms", total_time * 1e3);

                        // LOG_INFO("after state: \n");
                        // kf_ptr_->GetState().Print();
                        // auto t3 = std::chrono::high_resolution_clock::now();
                        // auto total_time = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count();
                        // LOG_INFO("Update Map used time: {} ms", total_time * 1e3);
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
        if (lidar_mean_scantime_ > 0.2) {
            LOG_WARN("lidar mean scan time is too large, mean scan time is {}", lidar_mean_scantime_);
        }
        lidar_pushed_ = true;
        LOG_INFO("lidar cloud size is {}, begin time is {:03.3f}, end time is {:03.3f}, mean scan time is {:03.3f}",
                 measures.curent_cloud->size(), measures.lidar_beg_time, measures.lidar_end_time, lidar_mean_scantime_);
    }
    // 处理imu数据
    double imu_time = system_->imu_queue_.front().timestamp_;
    while (!system_->imu_queue_.empty() && imu_time < measures.lidar_end_time) {
        measures.imus.push_back(system_->imu_queue_.front());
        system_->imu_queue_.pop_front();
        imu_time = system_->imu_queue_.front().timestamp_;
    }

    // 打印这里居然会有程序崩溃 log的double问题
    if (!measures.imus.empty()) {
        LOG_DEBUG("imu size is {}, imu begin_time {:03.3f}, imu_end_time {:03.3f}", measures.imus.size(),
                  measures.imus.front().timestamp_, measures.imus.end()->timestamp_);
    }
    // 处理encoder数据
    double encoder_time = system_->encoder_queue_.front().timestamp_;
    // for (const auto& encode : system_->encoder_queue_) {
    //     LOG_INFO("encoder time is {}", encode.timestamp_);
    // }
    if (use_encoder_) {
        while (!system_->encoder_queue_.empty() && encoder_time < measures.lidar_end_time) {
            measures.encoders.push_back(system_->encoder_queue_.front());
            system_->encoder_queue_.pop_front();
            encoder_time = system_->encoder_queue_.front().timestamp_;
            // LOG_INFO("encoder time is {}", encoder_time);
        }
        if (!measures.encoders.empty()) {
            LOG_DEBUG("encoder size is {}, encoder begin_time {}, encoder_end_time {}", measures.encoders.size(),
                      measures.encoders.front().timestamp_, measures.encoders.back().timestamp_);
        }
    }
    // 处理gnss数据
    double gnss_time = system_->gnss_queue_.front().timestamp_;
    if (use_gnss_) {
        while (!system_->gnss_queue_.empty() && gnss_time < measures.lidar_end_time) {
            measures.gnsss.push_back(system_->gnss_queue_.front());
            system_->gnss_queue_.pop_front();
            gnss_time = system_->gnss_queue_.front().timestamp_;
        }
        if (!measures.gnsss.empty()) {
            LOG_INFO("gnss size is {}, gnss begin_time {:03.3f}, gnss_end_time {:03.3f}", measures.gnsss.size(),
                     measures.gnsss.front().timestamp_, measures.gnsss.back().timestamp_);
        }
    }

    // 处理gnss数据
    system_->lidar_queue_.pop_front();
    system_->lidar_time_queue_.pop_front();
    lidar_pushed_ = false;
    if (measures.imus.empty()) {
        // LOG_INFO("imus is empty");
        return false;
    }
    return true;
}

void FrontEnd::UpdateGnss(const GNSS& gnss, State& state, ESKFShareState& share_state) {
    V3D z = V3D::Zero();
    Eigen::Matrix<double, 3, 33> H = Eigen::Matrix<double, 3, 33>::Zero();
    M3D R = M3D::Zero();
    V3D gnss_pos_in_imu(gnss.enu_[0], gnss.enu_[1], gnss.enu_[2]);
    // 计算残差
    V3D res = state.rot_R_IG * gnss_pos_in_imu - state.pos;
    LOG_INFO("gnss res:{}", res.transpose());
    // 计算雅可比矩阵
    Eigen::Matrix<double, 3, 33> J;
    J.setZero();
    // 对位置pos的雅可比矩阵
    J.block<3, 3>(0, 0) = -M3D::Identity();
    // 不对旋转求雅可比矩
    // 对rot_R_IG
    // 根据当前的协方差状态来判断是否要估计rot_R_IG
    M3D crossmat;
    crossmat << SKEW_SYM_MATRX(gnss_pos_in_imu);
    auto P = kf_ptr_->GetCov();
    if (sqrt(P(32, 32)) > 1e-4) {
        J.block<3, 3>(30, 30) = -state.rot_R_IG * crossmat;
    }
    M3D gnss_cov = M3D::Zero();
    gnss_cov(0, 0) = gnss.pos_cov_[0];
    gnss_cov(1, 1) = gnss.pos_cov_[7];
    gnss_cov(2, 2) = gnss.pos_cov_[14];
    M3D info_matrix = gnss_cov.inverse();
    // 方法2: 使用 SVD 伪逆（更稳定但更耗时）
    // Eigen::JacobiSVD<Eigen::MatrixXd> svd(gnss_cov, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // double tolerance = 1e-6;  // 奇异值阈值
    // Eigen::MatrixXd info_matrix = svd.matrixV() *
    //                               (svd.singularValues().array().abs() > tolerance)
    //                                   .select(svd.singularValues().array().inverse(), 0)
    //                                   .matrix()
    //                                   .asDiagonal() *
    //                               svd.matrixU().adjoint();

    share_state.valid = true;
    share_state.H33_.setZero();
    share_state.b33_.setZero();
    share_state.H33_ = J.transpose() * 10 * J;
    share_state.b33_ = J.transpose() * 10 * res;
    LOG_INFO("iter:{},res:{}", share_state.iter_num, res.transpose());
}

State FrontEnd::GetCurentNavState() {
    return kf_ptr_->GetState();
}

const M3D FrontEnd::GetGnssHeading() const {
    return propogator_ptr_->GetGnssHeading();
}

// lidar坐标系原始数据
const PointCloudXYZIPtr FrontEnd::GetCloudInLidarLink() const {
    return undistort_cloud_lidar_;
}

// robot_link坐标系点云
const PointCloudXYZIPtr FrontEnd::GetCloudInRobotLink() const {
    return undistort_cloud_robot_;
}

// odom坐标系点云
const PointCloudXYZIPtr FrontEnd::GetCloudInOdomLink() const {
    return undistort_cloud_odom_;
}

const M33D FrontEnd::GetCov() const {
    return kf_ptr_->GetCov();
}
}  // namespace slam
