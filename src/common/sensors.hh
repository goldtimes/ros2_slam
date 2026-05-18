#pragma once
#include <iostream>
#include <memory>
#include "common/eigen_type.hh"
/**
    传感器数据
*/
namespace slam {
// IMU数据
struct IMU {
    double timestamp_ = 0.0;
    V3D acc = V3D::Zero();
    V3D gyro = V3D::Zero();

    IMU() = default;
    IMU(double timestamp, const V3D& acc, const V3D& gyro) : timestamp_(timestamp), acc(acc), gyro(gyro) {
    }
};

using IMUPtr = std::shared_ptr<IMU>;

// 轮速计数据
struct Encoder {
    double timestamp_ = 0.0;
    V3D linear_vel = V3D::Zero();
    V3D angular_vel = V3D::Zero();

    Encoder() = default;
    Encoder(double timestamp, const V3D& linear_vel, const V3D& angular_vel)
        : timestamp_(timestamp), linear_vel(linear_vel), angular_vel(angular_vel) {
    }
    // 打印
    friend std::ostream& operator<<(std::ostream& os, const Encoder& encoder) {
        os << "Encoder: timestamp = " << std::fixed << encoder.timestamp_
           << ", linear_vel = " << encoder.linear_vel.transpose()
           << ", angular_vel = " << encoder.angular_vel.transpose();
        return os;
    }
};

using EncoderPtr = std::shared_ptr<Encoder>;

// GPS数据
struct GNSS {
    double timestamp_ = 0.0;
    V3D enu_ = V3D::Zero();
    double yaw_ = 0.0;
    bool heading_valid_ = false;
    V3D pos_cov_ = V3D::Zero();
    GNSS() = default;
    GNSS(double timestamp, const V3D& enu, const V3D& pos_cov = V3D::Zero(), double yaw = 0.0,
         bool heading_valid = false)
        : timestamp_(timestamp), enu_(enu), pos_cov_(pos_cov), yaw_(yaw), heading_valid_(heading_valid) {
    }
};
}  // namespace slam