#include "state.hh"

namespace slam {
double NavState::gravity = 9.81;

// 注意顺序，r_wi,t_wi
void NavState::operator+=(const V21D& delta) {
    r_wi *= Sophus::SO3d::exp(delta.segment<3>(0)).matrix();
    t_wi += delta.segment<3>(3);
    r_il *= Sophus::SO3d::exp(delta.segment<3>(6)).matrix();
    t_il += delta.segment<3>(9);
    v += delta.segment<3>(12);
    bg += delta.segment<3>(15);
    ba += delta.segment<3>(18);
    // 这版本没有对重力优化
}

V21D NavState::operator-(const NavState& other) const {
    V21D delta = V21D::Zero();
    delta.segment<3>(0) = Sophus::SO3d(other.r_wi.transpose() * r_wi).log();
    delta.segment<3>(3) = t_wi - other.t_wi;
    delta.segment<3>(6) = Sophus::SO3d(other.r_il.transpose() * r_il).log();
    delta.segment<3>(9) = t_il - other.t_il;
    delta.segment<3>(12) = v - other.v;
    delta.segment<3>(15) = bg - other.bg;
    delta.segment<3>(18) = ba - other.ba;
    return delta;
}

void NavState::Print() const {
    LOG_INFO("NavState: ");
    LOG_INFO("r_wi:{}", r_wi.eulerAngles(2, 1, 0).transpose());
    LOG_INFO("t_wi:{}", t_wi.transpose());
    LOG_INFO("r_il:{}", r_il.eulerAngles(2, 1, 0).transpose());
    LOG_INFO("t_il:{}", t_il.transpose());
    LOG_INFO("v:{}", v.transpose());
    LOG_INFO("bg:{}", bg.transpose());
    LOG_INFO("ba:{}", ba.transpose());
}
}  // namespace slam