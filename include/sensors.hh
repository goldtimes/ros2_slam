#pragma once
#include <memory>
#include "eigen_type.hh"

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
};

using EncoderPtr = std::shared_ptr<Encoder>;

// GPS数据
struct GNSS {
    double timestamp_ = 0.0;
    V3D lla = V3D::Zero();

    GNSS() = default;
    GNSS(double timestamp, const V3D& lla) : timestamp_(timestamp), lla(lla) {
    }
};
}  // namespace slam