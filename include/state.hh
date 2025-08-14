#pragma once
#include "eigen_type.hh"
#include "logger.hh"

namespace slam {
// 整个系统维护的状态量

#define R 0
#define T 3
#define E_R 6
#define E_T 9
#define V 12
#define BG 15
#define BA 18

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
        g = gravity_dir * NavState::gravity;
    }

    // 状态量的改变
    void operator+=(const V21D& delta);
    V21D operator-(const NavState& other) const;

    void Print() const;
};
}  // namespace slam