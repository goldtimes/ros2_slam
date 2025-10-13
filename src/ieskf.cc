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
    V34D delta = V34D::Zero();
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

void IESKF::UpdateLidar() {
    // 预测状态值
    State predict_x = x_;
    ESKFShareState shared_state;
    shared_state.iter_num = 0;
    shared_state.res = 1e10;
    V33D delta = V33D::Zero();
    // 高斯牛顿的求解,H矩阵和b矩阵
    for (size_t i = 0; i < max_iter_num_; i++) {
        // 构建点面的残差
        // auto t1 = std::chrono::high_resolution_clock::now();
        lidar_loss_func_(predict_x, shared_state);
        // auto t2 = std::chrono::high_resolution_clock::now();
        // auto time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        // LOG_INFO("lidar loss time:{}", time * 1e3);
        if (shared_state.valid == false) {
            break;
        }
        H_.setZero();
        b_.setZero();
        // 误差重置时的雅可比矩阵
        delta = x_ - predict_x;
        M33D J = M33D::Identity();
        J.block<3, 3>(3, 3) = Jr(delta.segment<3>(3));
        J.block<3, 3>(6, 6) = Jr(delta.segment<3>(6));
        J.block<2, 2>(21, 21) = x_.getNx() * predict_x.getMx(delta.segment<2>(21));
        H_ += J.transpose() * P_.inverse() * J;
        b_ += J.transpose() * P_.inverse() * delta;
        H_.block<12, 12>(0, 0) += shared_state.H_;
        b_.block<12, 1>(0, 0) += shared_state.b_;

        delta = -H_.inverse() * b_;
        // LOG_INFO("delta_norm r:{}, t:{}", delta.segment<3>(0).norm(), delta.segment<3>(3).norm());
        // if (delta.segment<3>(0).norm() > 0.1 || delta.segment<3>(3).norm() > 0.1) {
        //     break;
        // }
        if (std::isnan(delta[0])) {
            break;
        }

        x_ += delta;
        shared_state.iter_num += 1;
        if (stop_func_(delta)) {
            break;
        }
        // if (delta.maxCoeff() < 0.001) {
        //     break;
        // }
    }
    // 更新协方差
    M33D L = M33D::Identity();
    L.block<3, 3>(3, 3) = Jr(delta.segment<3>(3));
    L.block<3, 3>(6, 6) = Jr(delta.segment<3>(6));
    L.block<2, 2>(21, 21) = x_.getNx() * predict_x.getMx(delta.segment<2>(21));
    P_ = L * H_.inverse() * L.transpose();
}

void IESKF::UpdateEncoder(const Encoder& encoder, const Input& input) {
    // 预测状态值
    State predict_x = x_;
    ESKFShareState shared_state;
    shared_state.iter_num = 0;
    shared_state.res = 1e10;
    V33D delta = V33D::Zero();
    // 高斯牛顿的求解,H矩阵和b矩阵
    for (size_t i = 0; i < 1; i++) {
        // 构建点面的残差
        wheel_loss_func_(encoder, input, predict_x, shared_state);
        if (shared_state.valid == false) {
            break;
        }
        H_.setZero();
        b_.setZero();
        // 误差重置时的雅可比矩阵
        delta = x_ - predict_x;
        M33D J = M33D::Identity();
        J.block<3, 3>(3, 3) = Jr(delta.segment<3>(3));
        J.block<3, 3>(6, 6) = Jr(delta.segment<3>(6));
        J.block<2, 2>(21, 21) = x_.getNx() * predict_x.getMx(delta.segment<2>(21));
        H_ += J.transpose() * P_.inverse() * J;
        b_ += J.transpose() * P_.inverse() * delta;
        H_.block<33, 33>(0, 0) += shared_state.H33_;
        b_.block<33, 1>(0, 0) += shared_state.b33_;

        delta = -H_.inverse() * b_;
        // LOG_INFO("delta:{}", delta.transpose());
        // LOG_INFO("delta_norm:{}", delta.norm());
        if (std::isnan(delta[0])) {
            break;
        }
        x_ += delta;
        shared_state.iter_num += 1;
        if (stop_func_(delta)) {
            break;
        }
        // if (delta.maxCoeff() < 0.001) {
        //     break;
        // }
    }
    // 更新协方差
    M33D L = M33D::Identity();
    L.block<3, 3>(3, 3) = Jr(delta.segment<3>(3));
    L.block<3, 3>(6, 6) = Jr(delta.segment<3>(6));
    L.block<2, 2>(21, 21) = x_.getNx() * predict_x.getMx(delta.segment<2>(21));
    P_ = L * H_.inverse() * L.transpose();
}

void IESKF::UpdateGnss(const GNSS& gnss) {
    // 预测状态值
    State predict_x = x_;
    ESKFShareState shared_state;
    shared_state.iter_num = 0;
    shared_state.res = 1e10;
    V33D delta = V33D::Zero();
    // 高斯牛顿的求解,H矩阵和b矩阵
    for (size_t i = 0; i < max_iter_num_; i++) {
        // 构建点面的残差
        gnss_loss_func_(gnss, predict_x, shared_state);
        if (shared_state.valid == false) {
            break;
        }
        H_.setZero();
        b_.setZero();
        // 误差重置时的雅可比矩阵
        delta = x_ - predict_x;
        M33D J = M33D::Identity();
        J.block<3, 3>(3, 3) = Jr(delta.segment<3>(3));
        J.block<3, 3>(6, 6) = Jr(delta.segment<3>(6));
        J.block<2, 2>(21, 21) = x_.getNx() * predict_x.getMx(delta.segment<2>(21));
        H_ += J.transpose() * P_.inverse() * J;
        b_ += J.transpose() * P_.inverse() * delta;
        H_.block<33, 33>(0, 0) += shared_state.H33_;
        b_.block<33, 1>(0, 0) += shared_state.b33_;

        delta = -H_.inverse() * b_;
        // LOG_INFO("delta:{}", delta.transpose());
        // LOG_INFO("delta_norm:{}", delta.norm());
        if (std::isnan(delta[0])) {
            break;
        }
        x_ += delta;
        shared_state.iter_num += 1;
        if (stop_func_(delta)) {
            break;
        }
        // if (delta.maxCoeff() < 0.001) {
        //     break;
        // }
    }
    // 更新协方差
    // M33D L = M33D::Identity();
    // L.block<3, 3>(3, 3) = Jr(delta.segment<3>(3));
    // L.block<3, 3>(6, 6) = Jr(delta.segment<3>(6));
    // L.block<2, 2>(21, 21) = x_.getNx() * predict_x.getMx(delta.segment<2>(21));
    // P_ = L * H_.inverse() * L.transpose();
}
}  // namespace slam