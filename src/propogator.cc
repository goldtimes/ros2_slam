#include "propogator.hh"
#include "ieskf.hh"
#include "static_imu_init.hh"
#include "system_config.hh"

namespace slam {
Propogator::Propogator(std::shared_ptr<SystemConfig> config_, std::shared_ptr<IESKF> kf)
    : system_config_ptr_(config_), kf_(kf), Q_(Eigen::Matrix<double, 12, 12>::Zero()) {
    LOG_INFO("Propogator init");
    imu_init_ptr_ = std::make_shared<StaticImuInit>();
    Q_.Zero();
    Q_.block<3, 3>(0, 0) = M3D::Identity() * system_config_ptr_->imu_config_.acc_noise_std;
    Q_.block<3, 3>(3, 3) = M3D::Identity() * system_config_ptr_->imu_config_.gyro_noise_std;
    Q_.block<3, 3>(6, 6) = M3D::Identity() * system_config_ptr_->imu_config_.acc_bias_noise_std;
    Q_.block<3, 3>(9, 9) = M3D::Identity() * system_config_ptr_->imu_config_.gyro_bias_noise_std;
}
Propogator::~Propogator() {
}

bool Propogator::Initialize(MeasureGroup& meas) {
    if (imu_init_ptr_->GetInitSuccess()) {
        return true;
    }
    imu_init_ptr_->AddMeasurements(meas.imus);
    if (imu_init_ptr_->TryInit()) {
        init_success_ = true;
        // 重力对齐
        auto mean_acc = imu_init_ptr_->GetMeanAcc();
        auto mean_gyro = imu_init_ptr_->GetMeanGyro();
        kf_->State().r_wi = (Eigen::Quaterniond::FromTwoVectors((-mean_acc).normalized(), V3D(0, 0, -1))).matrix();
        kf_->State().InitGravityDir(V3D(0, 0, -1));
        // 设置外参信息
        kf_->State().r_il = system_config_ptr_->lidar2imu_.R;
        kf_->State().t_il = system_config_ptr_->lidar2imu_.t;
        T_IL_ = PoseTrans(kf_->State().r_il, kf_->State().t_il);
        // 设置bg,ba
        kf_->State().bg = imu_init_ptr_->GetMeanGyro();
        kf_->State().ba = imu_init_ptr_->GetMeanAcc() - kf_->GetState().r_wi.transpose() * V3D(0, 0, 9.8);

        // 设置初始协方差状态
        kf_->Cov().setIdentity();
        // 位置的协方差
        kf_->Cov().block<3, 3>(0, 0) = M3D::Identity() * 0.00001;
        kf_->Cov().block<3, 3>(3, 3) = M3D::Identity() * 0.00001;
        // 外参协方差置信度较高
        kf_->Cov().block<3, 3>(6, 6) = M3D::Identity() * 0.00001;
        kf_->Cov().block<3, 3>(9, 9) = M3D::Identity() * 0.00001;
        // bg ba
        kf_->Cov().block<3, 3>(15, 15) = M3D::Identity() * 0.0001;
        kf_->Cov().block<3, 3>(18, 18) = M3D::Identity() * 0.0001;
        // get last imu data
        last_imu_ = imu_init_ptr_->last_imu_;
        current_imu_time_ = last_imu_.timestamp_;
        Eigen::Quaterniond q_inG = Eigen::Quaterniond(kf_->GetState().r_wi);
        Eigen::Quaterniond q_il = Eigen::Quaterniond(kf_->GetState().r_il);
        LOG_INFO(REDPURPLE "System init success time:{}" RESET, last_imu_.timestamp_);
        LOG_INFO(REDPURPLE "r_il:{}" RESET, q_il.coeffs().transpose());
        LOG_INFO(REDPURPLE "t_il:{}" RESET, kf_->GetState().t_il.transpose());
        LOG_INFO(REDPURPLE "orientation:{}" RESET, q_inG.coeffs().transpose());
        LOG_INFO(REDPURPLE "position:{}" RESET, kf_->GetState().t_wi.transpose());
        LOG_INFO(REDPURPLE "vel:{}" RESET, kf_->GetState().v.transpose());
        LOG_INFO(REDPURPLE "bg:{}" RESET, kf_->GetState().bg.transpose());
        LOG_INFO(REDPURPLE "ba:{}" RESET, kf_->GetState().ba.transpose());
        LOG_INFO(REDPURPLE "g:{}" RESET, kf_->GetState().g.transpose());
        return true;
    }
    return false;
}

NominalState Propogator::GetNominalState() const {
    return NominalState(current_imu_time_, kf_->GetState().r_wi, kf_->GetState().t_wi, kf_->GetState().v,
                        kf_->GetState().bg, kf_->GetState().ba);
}

void Propogator::PropogateAndUndistort(MeasureGroup& meas, PointCloudPtr& out_cloud) {
    // 准备好imu数据
    imu_caches_.clear();
    imu_caches_.push_back(last_imu_);
    imu_caches_.insert(imu_caches_.end(), meas.imus.begin(), meas.imus.end());
    const double imu_time_begin = imu_caches_.front().timestamp_;
    const double imu_end_time = imu_caches_.back().timestamp_;
    const double cloud_begin_time = meas.lidar_beg_time;
    const double propogate_end_time = meas.lidar_end_time;
    // 准备好去畸变的数据
    imu_pose_cache_.clear();
    imu_pose_cache_.emplace_back(0.0, last_acc_, last_gyro_, kf_->GetState().v, kf_->GetState().t_wi,
                                 kf_->GetState().r_wi);

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
        last_acc_ = kf_->GetState().r_wi * (mid_acc - kf_->GetState().ba) + kf_->GetState().g;
        last_gyro_ = mid_gyro - kf_->GetState().bg;
        double offset = tail.timestamp_ - cloud_begin_time;
        imu_pose_cache_.emplace_back(offset, last_acc_, last_gyro_, kf_->GetState().v, kf_->GetState().t_wi,
                                     kf_->GetState().r_wi);
        // kf_->GetState().Print();
        current_imu_time_ = head.timestamp_;
    }
    dt = propogate_end_time - imu_end_time;
    // LOG_INFO("DT:{}", dt);
    kf_->Predict(input, dt, Q_);
    // kf_->GetState().Print();
    last_imu_ = imu_caches_.back();
    last_propagate_time_ = propogate_end_time;
    // 去畸变
    UndistortLidar(meas, out_cloud);
}

void Propogator::UndistortLidar(MeasureGroup& meas, PointCloudPtr& cloud_out) {
    M3D cur_r_wi = kf_->GetState().r_wi;
    V3D cur_t_wi = kf_->GetState().t_wi;
    M3D cur_r_il = kf_->GetState().r_il;
    V3D cur_t_il = kf_->GetState().t_il;
    auto it_pcl = meas.curent_cloud->points.end() - 1;
    const double cloud_start_time = meas.lidar_beg_time;
    for (auto it_kp = imu_pose_cache_.end() - 1; it_kp != imu_pose_cache_.begin(); it_kp--) {
        auto head = it_kp - 1;
        auto tail = it_kp;

        M3D imu_r_wi = head->rot;
        V3D imu_t_wi = head->pos;
        V3D imu_vel = head->vel;
        V3D imu_acc = tail->acc;
        V3D imu_gyro = tail->gyro;
        double dt = 0.0;
        double offset_lidar_time = it_pcl->time - cloud_start_time;
        for (; offset_lidar_time > head->offset; it_pcl--) {
            dt = offset_lidar_time - head->offset;
            V3D point(it_pcl->x, it_pcl->y, it_pcl->z);
            M3D point_rot = imu_r_wi * Sophus::SO3d::exp(imu_gyro * dt).matrix();
            V3D point_pos = imu_t_wi + imu_vel * dt + 0.5 * imu_acc * dt * dt;
            V3D p_compensate =
                cur_r_il.transpose() *
                (cur_r_wi.transpose() * (point_rot * (cur_r_il * point + cur_t_il) + point_pos - cur_t_wi) - cur_t_il);
            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);
            if (it_pcl == meas.curent_cloud->points.begin()) break;
        }
    }
    cloud_out = meas.curent_cloud;
}

// void Propogator::UndistortLidar(const PointCloudPtr& cloud_in, PointCloudPtr& cloud_out) {
//     NominalState imu_state_end = GetNominalState();
//     // 末尾时刻的位姿
//     SE3 T_end = SE3(imu_state_end.R_, imu_state_end.p_);
//     // save pcd
//     // pcl::io::savePCDFileBinary("/home/kilox/distort.pcd", *cloud_in);
//     // 去畸变
//     for (auto& point : cloud_in->points) {
//         SE3 Ti = T_end;
//         NominalState best_mathc;
//         InterpolatePose<NominalState>(
//             point.time, imu_states_, [](const NominalState& state) { return state.timestamp_; },
//             [](const NominalState& state) { return SE3(state.R_, state.p_); }, Ti, best_mathc);
//         V3D pt_eigen = point.getVector3fMap().cast<double>();
//         V3D pt_compensate = T_IL_.inverse() * T_end.inverse() * Ti * T_IL_ * pt_eigen;
//         point.x = pt_compensate(0);
//         point.y = pt_compensate(1);
//         point.z = pt_compensate(2);
//     }

//     cloud_out = cloud_in;
//     // pcl::io::savePCDFileBinary("/home/kilox/undistort.pcd", *cloud_out);
// }

}  // namespace slam