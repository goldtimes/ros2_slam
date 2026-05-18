#pragma once
#include <Eigen/Eigen>
namespace slam {

class PoseTrans {
   public:
    PoseTrans() {
        R = Eigen::Matrix3d::Identity();
        t = Eigen::Vector3d::Zero();
    }
    PoseTrans(const Eigen::Matrix3d &R, const Eigen::Vector3d &t) : R(R), t(t) {
    }

    const PoseTrans operator*(const PoseTrans &other) const {
        return PoseTrans(R * other.R, R * other.t + t);
    }

    const Eigen::Vector3d operator*(const Eigen::Vector3d &point_in) const {
        return R * point_in + t;
    }

    PoseTrans &operator=(const PoseTrans &other) {
        R = other.R;
        t = other.t;
        return *this;
    };

    PoseTrans(const PoseTrans &other) : R(other.R), t(other.t) {
    }

    PoseTrans inverse() const {
        return PoseTrans(R.inverse(), -R.inverse() * t);
    }

    Eigen::Matrix4d matrix() const {
        Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
        ret.block<3, 3>(0, 0) = R;
        ret.block<3, 1>(0, 3) = t;
        return ret;
    }

    const Eigen::Quaterniond eigen_q() const {
        return Eigen::Quaterniond(R);
    }

    Eigen::Vector3d RPY() const {
        Eigen::Vector3d rpy;
        rpy(1) = asin(-R(2, 0));
        double c_pitch = cos(rpy(1));
        rpy(0) = atan2(R(2, 1) / c_pitch, R(2, 2) / c_pitch);
        rpy(2) = atan2(R(1, 0) / c_pitch, R(0, 0) / c_pitch);
        return rpy;
    }

    double norm() {
        return RPY().norm() + t.norm();
    }

    friend std::ostream &operator<<(std::ostream &os, const PoseTrans &pose) {
        Eigen::Vector3d rpy = pose.RPY();
        os << "x: " << pose.t[0] << " y: " << pose.t[1] << " z: " << pose.t[2] << " roll: " << rpy[0]
           << " pitch: " << rpy[1] << " yaw: " << rpy[2];
        return os;
    }

    Eigen::Matrix<double, 3, 3> R;
    Eigen::Matrix<double, 3, 1> t;
};
}  // namespace slam