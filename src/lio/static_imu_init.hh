#pragma once
#include <deque>
#include "common/commons.hh"
#include "common/eigen_type.hh"
#include "common/sensors.hh"
#include "utils/logger.hh"

namespace slam {

class StaticImuInit {
   public:
    struct Options {
        Options() {
        }
        // 静止的时间
        double init_time_seconds = 1.0;
        // 队列长度
        int init_buffer_size = 1000;
        double max_static_gyro_var = 0.5;
        double max_static_acc_var = 0.05;
        double gravity_norm_ = 9.81;
        void print() {
            LOG_INFO("init_time_seconds:{}", init_time_seconds);
            LOG_INFO("init_buffer_size:{}", init_buffer_size);
            LOG_INFO("max_static_gyro_var:{}", max_static_gyro_var);
            LOG_INFO("max_static_acc_var:{}", max_static_acc_var);
            LOG_INFO("gravity_norm_:{}", gravity_norm_);
        }
    };
    explicit StaticImuInit(Options options = Options()) {
        LOG_INFO("StaticImuInit init");
        options_ = options;
        options_.print();
    }
    ~StaticImuInit() = default;

    bool AddMeasurements(const std::deque<IMU>& datas);

    bool TryInit();

    bool GetInitSuccess() const {
        return init_success_;
    }
    const Eigen::Vector3d GetMeanAcc() const {
        return mean_acc_;
    }
    const Eigen::Vector3d GetMeanGyro() const {
        return mean_gyro_;
    }
    const Eigen::Vector3d& GetCovGyro() const {
        return cov_gyro_;
    }
    const Eigen::Vector3d& GetCovAcc() const {
        return cov_acc_;
    }
    const Eigen::Vector3d& GetInitBa() const {
        return init_ba_;
    }
    const Eigen::Vector3d& GetinitBg() const {
        return init_bg_;
    }
    const Eigen::Vector3d& GetGravity() const {
        return gravity_;
    }
    const Eigen::Matrix3d& GetRItoG() const {
        return R_ItoG_;
    }

    void grad_schmit(const Eigen::Vector3d& gravity_inI, Eigen::Matrix3d& R_GtoI) {
        // 归一化z轴的方向
        Eigen::Vector3d z_axis = gravity_inI / gravity_inI.norm();
        Eigen::Vector3d x_axis, y_axis;
        Eigen::Vector3d e1(1, 0, 0);
        Eigen::Vector3d e2(0, 1, 0);
        double inner1 = e1.dot(z_axis) / z_axis.norm();
        double inner2 = e2.dot(z_axis) / z_axis.norm();
        if (fabs(inner1) < fabs(inner2)) {
            x_axis = z_axis.cross(e1);
            x_axis = x_axis / x_axis.norm();
            y_axis = z_axis.cross(x_axis);
            y_axis = y_axis / y_axis.norm();
        } else {
            x_axis = z_axis.cross(e2);
            x_axis = x_axis / x_axis.norm();
            y_axis = z_axis.cross(x_axis);
            y_axis = y_axis / y_axis.norm();
        }
        // Rotation from our global (where gravity is only along the z-axis) to the local one
        R_GtoI.block(0, 0, 3, 1) = x_axis;
        R_GtoI.block(0, 1, 3, 1) = y_axis;
        R_GtoI.block(0, 2, 3, 1) = z_axis;
    }

   public:
    Options options_;
    IMU last_imu_;

   private:
    std::deque<IMU> init_imu_buffer_;

    V3D mean_acc_ = V3D::Zero();
    V3D mean_gyro_ = V3D::Zero();
    bool init_success_ = false;
    bool try_to_init_ = false;
    // 静止的初始时间
    double init_start_time = 0.0;
    V3D init_ba_ = V3D::Zero();
    V3D init_bg_ = V3D::Zero();
    V3D cov_gyro_ = V3D::Zero();
    V3D cov_acc_ = V3D::Zero();
    V3D gravity_ = V3D::Zero();
    // 重力对齐后的初始姿态
    M3D R_ItoG_ = M3D::Identity();
};
}  // namespace slam