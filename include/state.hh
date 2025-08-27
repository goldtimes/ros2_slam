#pragma once
#include "eigen_type.hh"
#include "logger.hh"

namespace slam {
// 整个esikf维护的状态量
struct NavState {
    // 重力常量
    static double gravity;
    M3D r_wi = M3D::Identity();  // i->wolrd
    V3D t_wi = V3D::Zero();      // i->wolrd
    M3D r_il = M3D::Identity();  // lidar->imu的旋转矩阵
    V3D t_il = V3D::Zero();      // lidar->imu的平移向量
    V3D v = V3D::Zero();         // 速度
    V3D bg = V3D::Zero();        // imu的bias
    V3D ba = V3D::Zero();        // imu的bias
    V3D g = V3D(0, 0, -9.81);    // 重力
    void InitGravityDir(const V3D& gravity_dir) {
        g = gravity_dir.normalized() * NavState::gravity;
    }

    // 状态量的改变
    void operator+=(const V21D& delta);
    V21D operator-(const NavState& other) const;

    void Print() const;
};

// 为雷达去畸变的名义状态量
struct NominalState {
    NominalState() = default;

    // from time, R, p, v, bg, ba
    explicit NominalState(double time, const SO3& R = SO3(), const V3D& t = V3D::Zero(), const V3D& v = V3D::Zero(),
                          const V3D& bg = V3D::Zero(), const V3D& ba = V3D::Zero())
        : timestamp_(time), R_(R), p_(t), v_(v), bg_(bg), ba_(ba) {
    }

    // from pose and vel
    NominalState(double time, const SE3& pose, const V3D& vel = V3D::Zero())
        : timestamp_(time), R_(pose.so3()), p_(pose.translation()), v_(vel) {
    }

    /// 转换到Sophus
    Sophus::SE3d GetSE3() const {
        return SE3(R_, p_);
    }

    // friend std::ostream& operator<<(std::ostream& os, const NominalState& s) {
    //     os << "p: " << s.p_.transpose() << ", v: " << s.v_.transpose()
    //        << ", q: " << s.R_.().coeffs().transpose() << ", bg: " << s.bg_.transpose()
    //        << ", ba: " << s.ba_.transpose();
    //     return os;
    // }

    double timestamp_ = 0;  // 时间
    SO3 R_;                 // 旋转
    V3D p_ = V3D::Zero();   // 平移
    V3D v_ = V3D::Zero();   // 速度
    V3D bg_ = V3D::Zero();  // gyro 零偏
    V3D ba_ = V3D::Zero();  // acce 零偏
};

}  // namespace slam