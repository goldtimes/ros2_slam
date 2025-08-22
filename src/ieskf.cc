#include "ieskf.hh"

namespace slam {
M3D IESKF::Jr(const V3D& inp) {
    return Sophus::SO3d::jl(inp).transpose();
}
M3D IESKF::JrInv(const V3D& inp) {
    return Sophus::SO3d::jl(inp).inverse().transpose();
}

void IESKF::Predict(const Input& input, double dt, const M12D& Q) {
    // 预测状态
    V21D delta = V21D::Zero();
    delta.segment<3>(0) = (input.gyro - state_.bg) * dt;                             // 旋转量
    delta.segment<3>(3) = state_.v * dt;                                             // 位移
    delta.segment<3>(12) = (state_.r_wi * (input.acc - state_.ba) + state_.g) * dt;  // 速度,g 为-9.81
    // 计算m_F矩阵
    m_F_.setIdentity();
    // delta_theta / delta_theta
    m_F_.block<3, 3>(0, 0) = Sophus::SO3d::exp(-(input.gyro - state_.bg) * dt).matrix();
    m_F_.block<3, 3>(0, 15) = -Jr((input.gyro - state_.bg) * dt) * dt;
    m_F_.block<3, 3>(3, 12) = M3D::Identity() * dt;
    m_F_.block<3, 3>(12, 0) = -state_.r_wi * Sophus::SO3d::hat(input.acc - state_.ba) * dt;
    m_F_.block<3, 3>(12, 18) = -state_.r_wi * dt;
    // 计算m_G矩阵
    m_G_.setZero();
    m_G_.block<3, 3>(0, 0) = -Jr((input.gyro - state_.bg) * dt) * dt;
    m_G_.block<3, 3>(12, 3) = -state_.r_wi * dt;
    m_G_.block<3, 3>(15, 6) = M3D::Identity() * dt;
    m_G_.block<3, 3>(18, 9) = M3D::Identity() * dt;
    // 状态更新
    state_ += delta;
    // 协方差更新
    cov_ = m_F_ * cov_ * m_F_.transpose() + m_G_ * Q * m_G_.transpose();
}

void IESKF::Update() {
    // 预测状态值
    NavState predict_x = state_;
    ESKFShareState shared_state;
    shared_state.iter_num = 0;
    shared_state.res = 1e10;
    V21D delta = V21D::Zero();
    // 高斯牛顿的求解,H矩阵和b矩阵
    M21D H = M21D::Identity();
    V21D b = V21D::Zero();

    for (size_t i = 0; i < max_iter_num_; i++) {
        // 构建点面的残差
        lidar_loss_func_(predict_x, shared_state);
        if (shared_state.valid == false) {
            break;
        }
        H.setZero();
        b.setZero();
        // 误差重置时的雅可比矩阵
        delta = state_ - predict_x;
        M21D J = M21D::Identity();
        // 投影P矩阵
        J.block<3, 3>(0, 0) = JrInv(delta.segment<3>(0));
        J.block<3, 3>(6, 6) = JrInv(delta.segment<3>(6));
        H += J.transpose() * cov_.inverse() * J;
        b += J.transpose() * cov_.inverse() * delta;
        H.block<12, 12>(0, 0) += shared_state.H_;
        b.block<12, 1>(0, 0) += shared_state.b_;

        delta = -H.inverse() * b;
        // LOG_INFO("delta:{}", delta.transpose());
        if (std::isnan(delta[0])) {
            break;
        }
        state_ += delta;
        shared_state.iter_num += 1;
        if (stop_func_(delta)) {
            break;
        }
    }
    // 更新协方差
    M21D L = M21D::Identity();
    // L.block<3, 3>(0, 0) = JrInv(delta.segment<3>(0));
    // L.block<3, 3>(6, 6) = JrInv(delta.segment<3>(6));
    L.block<3, 3>(0, 0) = Jr(delta.segment<3>(0));
    L.block<3, 3>(6, 6) = Jr(delta.segment<3>(6));
    cov_ = L * H.inverse() * L.transpose();
}
}  // namespace slam