#pragma once

#include "common/commons.hh"
#include "common/eigen_type.hh"
#include "s2.hpp"

namespace slam {
struct NavState {
    constexpr static int dim = 23;       // s2上的重力为两维
    constexpr static int full_dim = 24;  // 实际上是24维

    using VecState = Eigen::Matrix<double, dim, 1>;
    using FullVectState = Eigen::Matrix<double, full_dim, 1>;

    NavState() = default;

    bool operator<(const NavState& other) const {
        return timestamp_ < other.timestamp_;
    }
    // 转换为状态向量
    FullVectState ToState() {
        FullVectState ret;
        ret.block<3, 1>(0, 0) = pos_;
        ret.block<3, 1>(3, 0) = rot_.log();
        ret.block<3, 1>(6, 0) = offset_R_L_I.log();
        ret.block<3, 1>(9, 0) = offset_t_L_I;
        ret.block<3, 1>(12, 0) = vel_;
        ret.block<3, 1>(15, 0) = bg_;
        ret.block<3, 1>(18, 0) = ba_;
        ret.block<3, 1>(21, 0) = grav_.vec_;
        return ret;
    }
    // 从状态向量中恢复
    void FromVectState(const FullVectState& state) {
        pos_ = state.block<3, 1>(0, 0);
        rot_ = SO3::exp(state.block<3, 1>(3, 0));
        offset_R_L_I = SO3::exp(state.block<3, 1>(6, 0));
        offset_t_L_I = state.block<3, 1>(9, 0);
        vel_ = state.block<3, 1>(12, 0);
        bg_ = state.block<3, 1>(15, 0);
        ba_ = state.block<3, 1>(18, 0);
        grav_.vec_ = state.block<3, 1>(21, 0);
    }

    inline FullVectState get_f(const V3D& gyro, const V3D& acce) const {
        FullVectState res = FullVectState::Zero();
        // 减零偏
        V3D omega = gyro - bg_;
        V3D a_inertial = rot_ * (acce - ba_);  // 加计读数-ba 并转到 世界系下
        // 用来给状态递推的
        for (int i = 0; i < 3; i++) {
            res(i) = vel_[i];                        // 存放了速度值，
            res(i + 3) = omega[i];                   // 角速度值
            res(i + 12) = a_inertial[i] + grav_[i];  // 加速度值
        }
        return res;
    }

    /// 递推
    void oplus(const FullVectState& vec, double dt) {
        timestamp_ += dt;
        // middleRows 0开始，取3行 就是imu的运动递推公式
        pos_ += vec.middleRows(0, 3) * dt;
        rot_ = rot_ * SO3::exp(vec.middleRows(3, 3) * dt);
        offset_R_L_I = offset_R_L_I * SO3::exp(vec.middleRows(6, 3) * dt);
        offset_t_L_I = offset_t_L_I + vec.middleRows(9, 3) * dt;
        vel_ += vec.middleRows(12, 3) * dt;
        bg_ += vec.middleRows(15, 3) * dt;
        ba_ += vec.middleRows(18, 3) * dt;
        grav_.oplus(vec.middleRows(21, 3) * dt);
    }

    /// 运动方程对状态的雅可比
    inline Eigen::Matrix<double, full_dim, dim> df_dx(const V3D& acce) const {
        // 24x23的雅可比矩阵
        Eigen::Matrix<double, full_dim, dim> cov = Eigen::Matrix<double, full_dim, dim>::Zero();
        cov.block<3, 3>(0, 12) = M3D::Identity();
        V3D acc = acce - ba_;
        // Vec3d omega = gyro - bg_;
        cov.block<3, 3>(12, 3) = -rot_.matrix() * SO3::hat(acc);
        cov.block<3, 3>(12, 18) = -rot_.matrix();

        Vec2d vec = Vec2d::Zero();
        Eigen::Matrix<double, 3, 2> grav_matrix = grav_.S2_Mx(vec);

        cov.block<3, 2>(12, 21) = grav_matrix;
        cov.block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
        return cov;
    }

    /// 运动方程对噪声的雅可比
    inline Eigen::Matrix<double, 24, 12> df_dw() const {
        Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
        cov.block<3, 3>(12, 3) = -rot_.matrix();
        cov.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
        cov.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
        cov.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
        return cov;
    }

    // 广义加法
    /**
     * 广义减法, this - other
     * @param result 减法结果
     * @param other 另一个状态变量
     */
    VecState boxminus(const NavState& other) {
        VecState result;
        result.block<3, 1>(0, 0) = pos_ - other.pos_;
        result.block<3, 1>(3, 0) = (other.rot_.inverse() * rot_).log();
        result.block<3, 1>(6, 0) = (other.offset_R_L_I.inverse() * offset_R_L_I).log();
        result.block<3, 1>(9, 0) = offset_t_L_I - other.offset_t_L_I;
        result.block<3, 1>(12, 0) = vel_ - other.vel_;
        result.block<3, 1>(15, 0) = bg_ - other.bg_;
        result.block<3, 1>(18, 0) = ba_ - other.ba_;

        Vec2d dg = grav_.boxminus(other.grav_);
        result.block<2, 1>(21, 0) = dg;
        return result;
    }

    /**
     * 广义加法 this = this+dx
     * @param dx 增量
     */
    NavState boxplus(const VecState& dx) {
        NavState ret;
        ret.timestamp_ = timestamp_;
        ret.pos_ = pos_ + dx.middleRows(0, 3);
        ret.rot_ = rot_ * SO3::exp(dx.middleRows(3, 3));
        ret.offset_R_L_I = offset_R_L_I * SO3::exp(dx.middleRows(6, 3));
        ret.offset_t_L_I = offset_t_L_I + dx.middleRows(9, 3);
        ret.vel_ = vel_ + dx.middleRows(12, 3);
        ret.bg_ = bg_ + dx.middleRows(15, 3);
        ret.ba_ = ba_ + dx.middleRows(18, 3);
        ret.grav_ = grav_;
        ret.grav_.boxplus(dx.middleRows(21, 2));

        return ret;
    }

    friend inline std::ostream& operator<<(std::ostream& os, const NavState& s) {
        os << std::setprecision(18) << s.pos_.transpose() << " " << s.rot_.unit_quaternion().coeffs().transpose() << " "
           << s.offset_R_L_I.unit_quaternion().coeffs().transpose() << " " << s.offset_t_L_I.transpose() << " "
           << s.vel_.transpose() << " " << s.bg_.transpose() << " " << s.ba_.transpose() << " "
           << s.grav_.vec_.transpose();
        return os;
    }

    /// 各个子变量所在维度信息
    struct MetaInfo {
        MetaInfo(int idx, int vdim, int dof) : idx_(idx), dim_(vdim), dof_(dof) {
        }
        int idx_ = 0;  // 变量所在索引
        int dim_ = 0;  // 变量维度
        int dof_ = 0;  // 自由度
    };

    static const std::vector<MetaInfo> vect_states_;  // 矢量变量的维度
    static const std::vector<MetaInfo> SO3_states_;   // SO3 变量的维度
    static const std::vector<MetaInfo> S2_states_;    // S2 变量维度

    inline SE3 GetPose() const {
        return SE3(rot_, pos_);
    }
    inline SO3 GetRot() const {
        return rot_;
    }
    inline void SetPose(const SE3& pose) {
        rot_ = pose.so3();
        pos_ = pose.translation();
    }

    inline V3D Getba() const {
        return ba_;
    }
    inline V3D Getbg() const {
        return bg_;
    }
    inline V3D GetVel() const {
        return vel_;
    }
    void SetVel(const V3D& v) {
        vel_ = v;
    }

    double timestamp_ = 0.0;           // 时间戳
    double confidence_ = 0.0;          // 定位置信度
    bool pose_is_ok_ = true;           // 定位是否正常
    bool lidar_odom_reliable_ = true;  // 激光里程计是否可靠
    bool is_parking_ = false;          // 是否在停车状态

    V3D pos_ = V3D::Zero();          // 位置
    SO3 rot_;                        // 旋转
    SO3 offset_R_L_I;                // lidar->imu旋转外参
    V3D offset_t_L_I = V3D::Zero();  // lidar->imu位置外参
    V3D vel_ = V3D::Zero();          // 速度
    V3D bg_ = V3D::Zero();           // imu bias
    V3D ba_ = V3D::Zero();           // imu bias
    S2 grav_ = S2();                 // 重力方向
};
}  // namespace slam