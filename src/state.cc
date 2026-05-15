#include "state.hh"

namespace slam {
double State::GRAVITY = 9.81;

void State::operator+=(const V33D &delta) {
    pos += delta.segment<3>(0);
    rot *= Sophus::SO3d::exp(delta.segment<3>(3)).matrix();
    rot_ext *= Sophus::SO3d::exp(delta.segment<3>(6)).matrix();
    pos_ext += delta.segment<3>(9);
    vel += delta.segment<3>(12);
    bg += delta.segment<3>(15);
    ba += delta.segment<3>(18);
    g = Sophus::SO3d::exp(getBx() * delta.segment<2>(21)).matrix() * g;
    rot_R_IE *= Sophus::SO3d::exp(delta.segment<3>(23)).matrix();
    pos_t_IE += delta.segment<3>(26);
    wheel_scale += delta.segment<1>(29);
    rot_R_IG *= Sophus::SO3d::exp(delta.segment<3>(30)).matrix();
}

void State::operator+=(const V34D &delta) {
    pos += delta.segment<3>(0);
    rot *= Sophus::SO3d::exp(delta.segment<3>(3)).matrix();
    rot_ext *= Sophus::SO3d::exp(delta.segment<3>(6)).matrix();
    pos_ext += delta.segment<3>(9);
    vel += delta.segment<3>(12);
    bg += delta.segment<3>(15);
    ba += delta.segment<3>(18);
    g = Sophus::SO3d::exp(delta.segment<3>(21)).matrix() * g;
    rot_R_IE *= Sophus::SO3d::exp(delta.segment<3>(24)).matrix();
    pos_t_IE += delta.segment<3>(27);
    wheel_scale += delta.segment<1>(30);
    rot_R_IG *= Sophus::SO3d::exp(delta.segment<3>(31)).matrix();
}

V33D State::operator-(const State &other) {
    V33D delta = V33D::Zero();
    delta.segment<3>(0) = pos - other.pos;
    delta.segment<3>(3) = Sophus::SO3d(other.rot.transpose() * rot).log();

    delta.segment<3>(6) = Sophus::SO3d(other.rot_ext.transpose() * rot_ext).log();
    delta.segment<3>(9) = pos_ext - other.pos_ext;
    delta.segment<3>(12) = vel - other.vel;
    delta.segment<3>(15) = bg - other.bg;
    delta.segment<3>(18) = ba - other.ba;

    double v_sin = (Sophus::SO3d::hat(g) * other.g).norm();
    double v_cos = g.transpose() * other.g;
    double theta = std::atan2(v_sin, v_cos);
    Eigen::Vector2d res;
    if (v_sin < 1e-11) {
        if (std::fabs(theta) > 1e-11) {
            res << 3.1415926, 0;
        } else {
            res << 0, 0;
        }
    } else {
        res = theta / v_sin * other.getBx().transpose() * Sophus::SO3d::hat(other.g) * g;
    }
    delta.segment<2>(21) = res;
    delta.segment<3>(23) = Sophus::SO3d(other.rot_R_IE.transpose() * rot_R_IE).log();
    delta.segment<3>(26) = pos_t_IE - other.pos_t_IE;
    delta.segment<1>(29) = wheel_scale - other.wheel_scale;
    delta.segment<3>(30) = Sophus::SO3d(other.rot_R_IG.transpose() * rot_R_IG).log();
    return delta;
}

Matrix3x2d State::getBx() const {
    Matrix3x2d res;
    res << -g[1], -g[2], GRAVITY - g[1] * g[1] / (GRAVITY + g[0]), -g[2] * g[1] / (GRAVITY + g[0]),
        -g[2] * g[1] / (GRAVITY + g[0]), GRAVITY - g[2] * g[2] / (GRAVITY + g[0]);
    res /= GRAVITY;
    return res;
}

Matrix3x2d State::getMx() const {
    return -Sophus::SO3d::hat(g) * getBx();
}

Matrix3x2d State::getMx(const Eigen::Vector2d &res) const {
    Matrix3x2d bx = getBx();
    Eigen::Vector3d bu = bx * res;
    return -Sophus::SO3d::exp(bu).matrix() * Sophus::SO3d::hat(g) * Sophus::SO3d::jl(bu).transpose() * bx;
}

Matrix2x3d State::getNx() const {
    return 1 / GRAVITY / GRAVITY * getBx().transpose() * Sophus::SO3d::hat(g);
}

void State::Print() const {
    LOG_INFO("State: ");
    LOG_INFO("r_wi:{}", rot.eulerAngles(2, 1, 0).transpose());
    LOG_INFO("t_wi:{}", pos.transpose());
    LOG_INFO("r_il:{}", rot_ext.eulerAngles(2, 1, 0).transpose());
    LOG_INFO("t_il:{}", pos_ext.transpose());
    LOG_INFO("v:{}", vel.transpose());
    LOG_INFO("bg:{}", bg.transpose());
    LOG_INFO("ba:{}", ba.transpose());
    LOG_INFO("g:{}", g.transpose());
}
}  // namespace slam