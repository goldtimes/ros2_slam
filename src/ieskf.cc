#include "ieskf.hh"

namespace slam {

int IESKF::P_ID = 0, IESKF::R_ID = 3, IESKF::ER_ID = 6, IESKF::EP_ID = 9, IESKF::V_ID = 12, IESKF::BG_ID = 15,
    IESKF::BA_ID = 18, IESKF::G_ID = 21;

M3D IESKF::Jr(const V3D& inp) {
    return Sophus::SO3d::jl(inp).transpose();
}
M3D IESKF::JrInv(const V3D& inp) {
    return Sophus::SO3d::jl_inv(inp).transpose();
}

void IESKF::Predict(const Input& inp, double dt, const M12D& Q) {
    Vector24d delta = Vector24d::Zero();
    delta.segment<3>(0) = x_.vel * dt;
    delta.segment<3>(3) = (inp.gyro - x_.bg) * dt;
    delta.segment<3>(12) = (x_.rot * (inp.acc - x_.ba) + x_.g) * dt;
    F_.setIdentity();
    F_.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity() * dt;
    F_.block<3, 3>(3, 3) = Sophus::SO3d::exp(-(inp.gyro - x_.bg) * dt).matrix();
    F_.block<3, 3>(3, 15) = -Jr((inp.gyro - x_.bg) * dt) * dt;
    F_.block<3, 3>(12, 3) = -x_.rot * Sophus::SO3d::hat(inp.acc - x_.ba) * dt;
    F_.block<3, 3>(12, 18) = -x_.rot * dt;
    F_.block<3, 2>(12, 21) = x_.getMx() * dt;
    F_.block<2, 2>(21, 21) = x_.getNx() * x_.getMx();

    G_.setZero();
    G_.block<3, 3>(3, 0) = -Jr((inp.gyro - x_.bg) * dt) * dt;
    G_.block<3, 3>(12, 3) = -x_.rot * dt;
    G_.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity() * dt;
    G_.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity() * dt;
    x_ += delta;
    P_ = F_ * P_ * F_.transpose() + G_ * Q * G_.transpose();
}

void IESKF::Update() {
    // 预测状态值
    State predict_x = x_;
    ESKFShareState shared_state;
    shared_state.iter_num = 0;
    shared_state.res = 1e10;
    Vector23d delta = Vector23d::Zero();
    // 高斯牛顿的求解,H矩阵和b矩阵
    for (size_t i = 0; i < max_iter_num_; i++) {
        // 构建点面的残差
        lidar_loss_func_(predict_x, shared_state);
        if (shared_state.valid == false) {
            break;
        }
        H_.setZero();
        b_.setZero();
        // 误差重置时的雅可比矩阵
        delta = x_ - predict_x;
        Matrix23d J = Matrix23d::Identity();
        J.block<3, 3>(3, 3) = Jr(delta.segment<3>(3));
        J.block<3, 3>(6, 6) = Jr(delta.segment<3>(6));
        J.block<2, 2>(21, 21) = x_.getNx() * predict_x.getMx(delta.segment<2>(21));
        H_ += J.transpose() * P_.inverse() * J;
        b_ += J.transpose() * P_.inverse() * delta;
        H_.block<12, 12>(0, 0) += shared_state.H_;
        b_.block<12, 1>(0, 0) += shared_state.b_;

        delta = -H_.inverse() * b_;
        // LOG_INFO("delta:{},delta_norm:{}", delta.transpose(), delta.norm());
        if (std::isnan(delta[0])) {
            break;
        }
        x_ += delta;
        shared_state.iter_num += 1;
        // if (stop_func_(delta)) {
        //     break;
        // }
        if (delta.maxCoeff() < 0.001) {
            break;
        }
    }
    // 更新协方差
    Matrix23d L = Matrix23d::Identity();
    L.block<3, 3>(3, 3) = Jr(delta.segment<3>(3));
    L.block<3, 3>(6, 6) = Jr(delta.segment<3>(6));
    L.block<2, 2>(21, 21) = x_.getNx() * predict_x.getMx(delta.segment<2>(21));
    P_ = L * H_.inverse() * L.transpose();
}
}  // namespace slam