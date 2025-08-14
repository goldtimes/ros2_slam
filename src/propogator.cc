#include "propogator.hh"
#include "ieskf.hh"
#include "static_imu_init.hh"
#include "system_config.hh"

namespace slam {
Propogator::Propogator(std::shared_ptr<SystemConfig> config_, std::shared_ptr<IESKF> kf)
    : system_config_ptr_(config_), kf_(kf) {
    LOG_INFO("Propogator init");
    imu_init_ptr_ = std::make_shared<StaticImuInit>();
    Q_.Zero();
    Q_.block<3, 3>(0, 0) = M3D::Identity() * std::pow(system_config_ptr_->imu_config_.acc_noise_std, 2);
    Q_.block<3, 3>(3, 3) = M3D::Identity() * std::pow(system_config_ptr_->imu_config_.gyro_noise_std, 2);
    Q_.block<3, 3>(6, 6) = M3D::Identity() * std::pow(system_config_ptr_->imu_config_.acc_bias_noise_std, 2);
    Q_.block<3, 3>(9, 9) = M3D::Identity() * std::pow(system_config_ptr_->imu_config_.gyro_bias_noise_std, 2);
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
        kf_->State().r_il = system_config_ptr_->lidar2imu_.so3().matrix();
        kf_->State().t_il = system_config_ptr_->lidar2imu_.translation();
        // 设置bg,ba
        kf_->State().bg = imu_init_ptr_->GetMeanGyro();
        kf_->State().ba = imu_init_ptr_->GetMeanAcc() - kf_->GetState().r_wi.transpose() * V3D(0, 0, 9.8);

        // 设置初始协方差状态
        kf_->Cov().setIdentity();
        // 位置的协方差
        kf_->Cov().block<3, 3>(3, 3) = M3D::Identity() * 0.00001;
        // 外参协方差置信度较高
        kf_->Cov().block<3, 3>(6, 6) = M3D::Identity() * 0.00001;
        kf_->Cov().block<3, 3>(9, 9) = M3D::Identity() * 0.00001;
        // bg ba
        kf_->Cov().block<3, 3>(15, 15) = M3D::Identity() * 0.0001;
        kf_->Cov().block<3, 3>(18, 18) = M3D::Identity() * 0.0001;
        // get last imu data
        last_imu_ = imu_init_ptr_->last_imu_;
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

}  // namespace slam