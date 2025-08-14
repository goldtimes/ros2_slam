#include "static_imu_init.hh"

namespace slam {
bool StaticImuInit::AddMeasurements(const std::deque<IMU>& datas) {
    if (datas.empty()) {
        return false;
    }
    if (init_success_) {
        return true;
    }
    // 记录静止开始的时间
    if (init_imu_buffer_.empty()) {
        init_start_time = datas.front().timestamp_;
    }
    init_imu_buffer_.insert(init_imu_buffer_.end(), datas.begin(), datas.end());
    double init_time = init_imu_buffer_.back().timestamp_ - init_start_time;
    if (init_time > options_.init_time_seconds) {
        // 允许初始化
        try_to_init_ = true;
    }
    while (init_imu_buffer_.size() > options_.init_buffer_size) {
        init_imu_buffer_.pop_front();
    }
    return true;
};

bool StaticImuInit::TryInit() {
    if (!try_to_init_) {
        return false;
    }
    // 这里开始初始化，计算加速度的均值，角速度的均值
    // 这里学习高翔的老师的用法
    Eigen::Vector3d mean_gyro, mean_acce;
    ComputeMeanAndCovDiag(init_imu_buffer_, mean_gyro, cov_gyro_, [](const IMU& data) { return data.gyro; });
    ComputeMeanAndCovDiag(init_imu_buffer_, mean_acce, cov_acc_, [](const IMU& data) { return data.acc; });

    LOG_INFO("mean acc:{}", mean_acce.transpose());
    LOG_INFO("mean gyro:{}", mean_gyro.transpose());
    mean_acc_ = mean_acce;
    mean_gyro_ = mean_gyro;
    // 估计重力后重新计算加速度的均值和方差
    gravity_ = -mean_acce / mean_acce.norm() * options_.gravity_norm_;
    LOG_INFO("gravity:{}", gravity_.transpose());
    ComputeMeanAndCovDiag(init_imu_buffer_, mean_acce, cov_acc_,
                          [this](const IMU& data) { return data.acc + gravity_; });
    LOG_INFO("after add gravity mean acc:{}", mean_acce);
    // 检查IMU噪声
    if (cov_gyro_.norm() > options_.max_static_gyro_var) {
        LOG_ERROR("陀螺仪测量噪声太大:{}, max_static_gyro_var:{}", cov_gyro_.norm(), options_.max_static_gyro_var);
        return false;
    }

    if (cov_acc_.norm() > options_.max_static_acc_var) {
        LOG_ERROR("加速度计测量噪声太大:{}, max_static_acc_var:{}", cov_acc_.norm(), options_.max_static_acc_var);
        return false;
    }
    // 估计测量噪声和零偏
    init_bg_ = mean_gyro_;
    init_ba_ = mean_acc_;
    last_imu_ = init_imu_buffer_.back();
    // 重力对齐 FromTwoVectors 方法（两向量旋转）生成一个最短路径旋转
    // grad_schmit 方法（格拉姆 - 施密特正交化） x 轴的方向完全依赖于参考向量（e_1 或 e_2），结果受初始参考向量影响。
    // 用fastlio的方法
    init_success_ = true;
    return true;
}
}  // namespace slam