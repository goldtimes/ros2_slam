#include "propogator.hh"
#include "common/pose_trans.hh"
#include "ieskf.hh"
#include "static_imu_init.hh"
#include "system_config.hh"

namespace slam {
Propogator::Propogator(std::shared_ptr<SystemConfig> config_, std::shared_ptr<IESKF> kf)
    : system_config_ptr_(config_), kf_(kf), Q_(Eigen::Matrix<double, 12, 12>::Zero()) {
    LOG_INFO("Propogator init");
    // imu_init_ptr_ = std::make_shared<StaticImuInit>();
    Q_.Zero();
    Q_.block<3, 3>(0, 0) = M3D::Identity() * system_config_ptr_->imu_config_.acc_noise_std;
    Q_.block<3, 3>(3, 3) = M3D::Identity() * system_config_ptr_->imu_config_.gyro_noise_std;
    Q_.block<3, 3>(6, 6) = M3D::Identity() * system_config_ptr_->imu_config_.acc_bias_noise_std;
    Q_.block<3, 3>(9, 9) = M3D::Identity() * system_config_ptr_->imu_config_.gyro_bias_noise_std;

    use_wheel_ = config_->has_encoder_;
    use_gnss_ = config_->has_gnss_;
}
Propogator::~Propogator() {
}

bool Propogator::Initialize(MeasureGroup& meas) {
    // if (imu_init_ptr_->GetInitSuccess()) {
    //     return true;
    // }
    imu_init_buffer_.insert(imu_init_buffer_.end(), meas.imus.begin(), meas.imus.end());
    if (imu_init_buffer_.size() < 20) {
        return false;
    }
    Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
    for (const auto& imu : imu_init_buffer_) {
        acc_mean += imu.acc;
        gyro_mean += imu.gyro;
    }
    acc_mean /= static_cast<double>(imu_init_buffer_.size());
    gyro_mean /= static_cast<double>(imu_init_buffer_.size());

    // 设置外参信息
    kf_->x().rot_ext = system_config_ptr_->lidar2imu_.R;
    kf_->x().pos_ext = system_config_ptr_->lidar2imu_.t;
    kf_->x().bg = gyro_mean;
    kf_->x().rot =
        (Eigen::Quaterniond::FromTwoVectors((-acc_mean).normalized(), Eigen::Vector3d(0.0, 0.0, -1.0)).matrix());
    kf_->x().InitGravityDir(Eigen::Vector3d(0, 0, -1.0));

    kf_->Cov().setIdentity();
    kf_->Cov().block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 0.00001;
    kf_->Cov().block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 0.00001;
    kf_->Cov().block<3, 3>(15, 15) = Eigen::Matrix3d::Identity() * 0.0001;
    kf_->Cov().block<3, 3>(18, 18) = Eigen::Matrix3d::Identity() * 0.0001;
    kf_->Cov().block<2, 2>(21, 21) = Eigen::Matrix2d::Identity() * 0.00001;

    init_success_ = true;

    // imu_init_ptr_->AddMeasurements(meas.imus);
    // if (imu_init_ptr_->TryInit()) {
    //     init_success_ = true;
    //     // 重力对齐
    //     auto mean_acc = imu_init_ptr_->GetMeanAcc();
    //     auto mean_gyro = imu_init_ptr_->GetMeanGyro();
    //     kf_->x().r_wi = (Eigen::Quaterniond::FromTwoVectors((-mean_acc).normalized(), V3D(0, 0, -1))).matrix();
    //     kf_->x().InitGravityDir(V3D(0, 0, -1));
    //     // 设置外参信息
    //     kf_->x().r_il = system_config_ptr_->lidar2imu_.R;
    //     kf_->x().t_il = system_config_ptr_->lidar2imu_.t;
    //     T_IL_ = PoseTrans(kf_->x().r_il, kf_->x().t_il);
    //     // 设置bg,ba
    //     kf_->x().bg = imu_init_ptr_->GetMeanGyro();
    //     // kf_->x().ba = imu_init_ptr_->GetMeanAcc() - kf_->GetState().r_wi.transpose() * V3D(0, 0, 9.8);

    //     // 设置初始协方差状态
    //     kf_->Cov().setIdentity();
    //     // 位置的协方差
    //     // kf_->Cov().block<3, 3>(0, 0) = M3D::Identity() * 0.00001;
    //     // kf_->Cov().block<3, 3>(3, 3) = M3D::Identity() * 0.00001;
    //     // 外参协方差置信度较高
    //     kf_->Cov().block<3, 3>(6, 6) = M3D::Identity() * 0.00001;
    //     kf_->Cov().block<3, 3>(9, 9) = M3D::Identity() * 0.00001;
    //     // bg ba
    //     kf_->Cov().block<3, 3>(15, 15) = M3D::Identity() * 0.0001;
    //     kf_->Cov().block<3, 3>(18, 18) = M3D::Identity() * 0.0001;
    //     // get last imu data
    last_imu_ = meas.imus.back();
    current_imu_time_ = last_imu_.timestamp_;
    Eigen::Quaterniond q_inG = Eigen::Quaterniond(kf_->GetState().rot);
    Eigen::Quaterniond q_il = Eigen::Quaterniond(kf_->GetState().rot_ext);
    LOG_INFO(REDPURPLE "System init success time:{}" RESET, last_imu_.timestamp_);
    LOG_INFO(REDPURPLE "r_il:{}" RESET, q_il.coeffs().transpose());
    LOG_INFO(REDPURPLE "t_il:{}" RESET, kf_->GetState().pos_ext.transpose());
    LOG_INFO(REDPURPLE "orientation:{}" RESET, q_inG.coeffs().transpose());
    LOG_INFO(REDPURPLE "position:{}" RESET, kf_->GetState().pos.transpose());
    LOG_INFO(REDPURPLE "vel:{}" RESET, kf_->GetState().vel.transpose());
    LOG_INFO(REDPURPLE "bg:{}" RESET, kf_->GetState().bg.transpose());
    LOG_INFO(REDPURPLE "ba:{}" RESET, kf_->GetState().ba.transpose());
    LOG_INFO(REDPURPLE "g:{}" RESET, kf_->GetState().g.transpose());
    return true;
    // }
    // return false;
}

NominalState Propogator::GetNominalState() const {
    return NominalState(current_imu_time_, kf_->GetState().rot, kf_->GetState().pos, kf_->GetState().vel,
                        kf_->GetState().bg, kf_->GetState().ba);
}

void Propogator::PropogateState(MeasureGroup& meas) {
    // 准备好imu数据
    imu_caches_.clear();
    imu_caches_.push_back(last_imu_);
    imu_caches_.insert(imu_caches_.end(), meas.imus.begin(), meas.imus.end());
    const double imu_time_begin = imu_caches_.front().timestamp_;
    const double imu_end_time = imu_caches_.back().timestamp_;
    const double cloud_begin_time = meas.lidar_beg_time;
    const double propogate_end_time = meas.lidar_end_time;
    // 准备好去畸变的数据
    // 准备好去畸变的数据
    imu_states_.clear();
    NominalState state = GetNominalState();
    imu_states_.push_back(state);

    imu_pose_cache_.clear();
    imu_pose_cache_.emplace_back(0.0, last_acc_, last_gyro_, kf_->GetState().vel, kf_->GetState().pos,
                                 kf_->GetState().rot);

    V3D mid_acc, mid_gyro;
    double dt = 0.0;
    Input input;
    input.acc = imu_caches_.back().acc;
    input.gyro = imu_caches_.back().gyro;
    // 到尾部的前一个数据
    for (auto imu_it = imu_caches_.begin(); imu_it < (imu_caches_.end() - 1); imu_it++) {
        IMU& head = *imu_it;
        IMU& tail = *(imu_it + 1);
        // 确保imu的时间大于上一次传播结束的时间
        if (tail.timestamp_ < last_propagate_time_) {
            continue;
        }
        mid_gyro = 0.5 * (head.gyro + tail.gyro);
        mid_acc = 0.5 * (head.acc + tail.acc);
        if (head.timestamp_ < last_propagate_time_) {
            dt = tail.timestamp_ - last_propagate_time_;
        } else {
            dt = tail.timestamp_ - head.timestamp_;
        }
        // LOG_INFO("DT:{}", dt);
        input.acc = mid_acc;
        input.gyro = mid_gyro;
        kf_->Predict(input, dt, Q_);
        // kf_->GetState().Print();
        // 在这里去做轮速计的更新
        if (use_wheel_ && !meas.encoders.empty()) {
            // double wheel_time = meas.encoders.front().timestamp_;
            // if (wheel_time < head.timestamp_) {
            //     meas.encoders.pop_front();
            // } else {
            //     if (wheel_time < tail.timestamp_) {
            // 轮速计的时间在两个imu之间，传入imu的速度值和当前轮速计的速度
            // kf_->UpdateEncoder(meas.encoders.front(), input);
            // 删除轮速计
            // meas.encoders.pop_front();
            // }
            // }
        }
        if (use_gnss_ && !meas.gnsss.empty()) {
            double gnss_time = meas.gnsss.front().timestamp_;
            if (gnss_time < head.timestamp_) {
                meas.gnsss.pop_front();
            } else {
                if (gnss_time < tail.timestamp_) {
                    if (gnss_heading_init) {
                        if (meas.gnsss.front().pos_cov_[0] < 200) {
                            // 时间在两个imu之间
                            LOG_INFO("update gnss");
                            kf_->UpdateGnss(meas.gnsss.front());
                            meas.gnsss.pop_front();
                        }
                    }
                }
            }
            // kf_->UpdateGnss(meas.gnsss.front());
            // meas.gnsss.pop_front();
        }
        // LOG_INFO("after encoder update");
        // kf_->GetState().Print();

        last_acc_ = kf_->GetState().rot * (mid_acc - kf_->GetState().ba) + kf_->GetState().g;
        last_gyro_ = mid_gyro - kf_->GetState().bg;
        double offset = tail.timestamp_ - cloud_begin_time;
        imu_pose_cache_.emplace_back(offset, last_acc_, last_gyro_, kf_->GetState().vel, kf_->GetState().pos,
                                     kf_->GetState().rot);
        // kf_->GetState().Print();
        current_imu_time_ = head.timestamp_;
        imu_states_.push_back(GetNominalState());
    }
    dt = propogate_end_time - imu_end_time;
    // LOG_INFO("DT:{}", dt);
    kf_->Predict(input, dt, Q_);
    // kf_->GetState().Print();
    last_imu_ = imu_caches_.back();
    last_propagate_time_ = propogate_end_time;
    // // 去畸变
    // UndistortLidar(meas, out_cloud);
    // 这里对gnss的heading初始化
    if (system_config_ptr_->has_gnss_ && !gnss_heading_init) {
        // 这里对gnss的heading初始化
        if (!meas.gnsss.empty()) {
            V3D gnss_pose = meas.gnsss.back().enu_;
            gnss_pose[2] = 0.0;
            if (gnss_pose.norm() > 5.0) {
                auto current_state = kf_->GetState();
                Eigen::Vector3d tmp_vec(current_state.pos.x(), current_state.pos.y(), 0.0);
                Gnss_heading_ = Eigen::Quaterniond::FromTwoVectors(gnss_pose, tmp_vec).toRotationMatrix();
                // SO3 so3(Gnss_heading_);
                // V3D euler = SO3ToEuler(so3);
                // ROS_WARN_STREAM("INITIAL GNSS HEADING " << euler.transpose());
                LOG_INFO("GNSS HEADING:{}", Gnss_heading_.eulerAngles(2, 1, 0).transpose());
                current_state.rot_R_IG = Gnss_heading_;
                gnss_heading_init = true;
            }
        }
    }
}

// void Propogator::UndistortLidar(MeasureGroup& meas, PointCloudPtr& cloud_out) {
//     M3D cur_r_wi = kf_->GetState().r_wi;
//     V3D cur_t_wi = kf_->GetState().t_wi;
//     M3D cur_r_il = kf_->GetState().r_il;
//     V3D cur_t_il = kf_->GetState().t_il;
//     auto it_pcl = meas.curent_cloud->points.end() - 1;
//     const double cloud_start_time = meas.lidar_beg_time;
//     for (auto it_kp = imu_pose_cache_.end() - 1; it_kp != imu_pose_cache_.begin(); it_kp--) {
//         auto head = it_kp - 1;
//         auto tail = it_kp;

//         M3D imu_r_wi = head->rot;
//         V3D imu_t_wi = head->pos;
//         V3D imu_vel = head->vel;
//         V3D imu_acc = tail->acc;
//         V3D imu_gyro = tail->gyro;
//         double dt = 0.0;
//         double offset_lidar_time = it_pcl->time - cloud_start_time;
//         for (; offset_lidar_time > head->offset; it_pcl--) {
//             dt = offset_lidar_time - head->offset;
//             V3D point(it_pcl->x, it_pcl->y, it_pcl->z);
//             M3D point_rot = imu_r_wi * Sophus::SO3d::exp(imu_gyro * dt).matrix();
//             V3D point_pos = imu_t_wi + imu_vel * dt + 0.5 * imu_acc * dt * dt;
//             V3D p_compensate =
//                 cur_r_il.transpose() *
//                 (cur_r_wi.transpose() * (point_rot * (cur_r_il * point + cur_t_il) + point_pos - cur_t_wi) -
//                 cur_t_il);
//             it_pcl->x = p_compensate(0);
//             it_pcl->y = p_compensate(1);
//             it_pcl->z = p_compensate(2);
//             if (it_pcl == meas.curent_cloud->points.begin()) break;
//         }
//     }
//     cloud_out = meas.curent_cloud;
// }

void Propogator::UndistortLidar(MeasureGroup& meas, PointCloudXYZIPtr& cloud_out) {
    NominalState imu_state_end = GetNominalState();
    // 末尾时刻的位姿
    SE3 T_end(imu_state_end.R_, imu_state_end.p_);
    SE3 T_IL = SE3(T_IL_.R, T_IL_.t);
    // 预分配空间
    cloud_out->reserve(meas.curent_cloud->points.size());
    // save pcd
    // pcl::io::savePCDFileBinary("/home/kilox/distort.pcd", *cloud_in);
    // 去畸变
    for (const auto& point : meas.curent_cloud->points) {
        SE3 Ti = T_end;
        NominalState best_mathc;
        InterpolatePose<NominalState>(
            point.time, imu_states_, [](const NominalState& state) { return state.timestamp_; },
            [](const NominalState& state) { return SE3(state.R_, state.p_); }, Ti, best_mathc);
        V3D pt_eigen = point.getVector3fMap().cast<double>();
        V3D pt_compensate = T_IL.inverse() * T_end.inverse() * Ti * T_IL * pt_eigen;
        PointXYZI trans_pt;
        trans_pt.x = pt_compensate(0);
        trans_pt.y = pt_compensate(1);
        trans_pt.z = pt_compensate(2);
        trans_pt.intensity = point.intensity;
        cloud_out->points.push_back(trans_pt);
    }
}

}  // namespace slam