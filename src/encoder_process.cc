#include "encoder_process.hh"
#include "system_config.hh"

namespace slam {
EncoderProcessor::EncoderProcessor(const std::shared_ptr<SystemConfig>& config_ptr) : config_ptr_(config_ptr) {
    is_static_ = true;
    wheel_cov_ = config_ptr->encoder_config_.wheel_cov;
    nhc_y_ = config_ptr->encoder_config_.nhc_y;
    nhc_z_ = config_ptr->encoder_config_.nhc_z;
}

void EncoderProcessor::AddEncoder(const std::deque<Encoder>& encoders) {
    if (encoders.empty()) {
        return;
    }
    encoder_queue_.insert(encoder_queue_.end(), encoders.begin(), encoders.end());
    // 确保数据不要太多
    while (encoders.back().timestamp_ - encoders.front().timestamp_ > 1.0) {
        encoder_queue_.pop_front();
    }
}

bool EncoderProcessor::Propagation(PoseTrans& delta_pose, const double lidar_begin_time, const double lidar_end_time) {
    if (encoder_queue_.empty()) {
        return false;
    }
    // 检测是否有数据在lidar时间范围内
    // if (lidar_begin_time < encoder_queue_.front().timestamp_) {
    //     LOG_ERROR("EncoderProcessor::Propagation: lidar_begin_time < encoder_queue_.front().timestamp_");
    //     return false;
    // }
    // if (lidar_end_time > encoder_queue_.back().timestamp_) {
    //     LOG_ERROR("EncoderProcessor::Propagation: lidar_end_time > encoder_queue_.back().timestamp_");
    //     return false;
    // }

    std::deque<Encoder> selected_data;
    int start_idx = -1, end_idx = -1;
    for (int i = 0; i < encoder_queue_.size(); i++) {
        Encoder data = encoder_queue_[i];
        if (data.timestamp_ >= lidar_begin_time && data.timestamp_ <= lidar_end_time) {
            if (selected_data.empty()) {
                start_idx = i;
            }
            // std::cout << "data: " << data << std::endl;
            selected_data.push_back(data);
            end_idx = i;
        }
    }
    // 对第一个数据的encoder数据进行插值处理
    if (start_idx >= 1) {
        Encoder inter = interplate(encoder_queue_[start_idx - 1], encoder_queue_[start_idx], lidar_begin_time);
        selected_data.push_front(inter);
    }
    // 对最后一个数据的encoder数据进行插值处理
    if (end_idx < encoder_queue_.size() - 1) {
        Encoder inter = interplate(encoder_queue_[end_idx], encoder_queue_[end_idx + 1], lidar_end_time);
        selected_data.push_back(inter);
    }
    if (selected_data.size() <= 2) {
        return false;
    }
    // LOG_INFO("EncoderProcessor::Propagation: selected_data.size() = {}", selected_data.size());
    // 积分
    PoseTrans ret_pose;
    for (int i = 1; i < selected_data.size(); ++i) {
        double dt = selected_data[i].timestamp_ - selected_data[i - 1].timestamp_;
        double w = (selected_data[i].angular_vel.z() + selected_data[i - 1].angular_vel.z()) / 2.0;
        double v_x = (selected_data[i].linear_vel.x() + selected_data[i - 1].linear_vel.x()) / 2.0;
        double v_y = (selected_data[i].linear_vel.y() + selected_data[i - 1].linear_vel.y()) / 2.0;
        // LOG_INFO("EncoderProcessor::Propagation: dt = {}, w = {}, v_x = {}, v_y = {}", dt, w, v_x, v_y);
        M3D rot = M3D(Eigen::AngleAxisd(w * dt, V3D::UnitZ()));
        V3D trans = V3D(v_x * dt, v_y * dt, 0);
        PoseTrans delta(rot, trans);
        ret_pose = ret_pose * delta;
    }
    delta_pose = ret_pose;
    return true;
}

Encoder EncoderProcessor::interplate(Encoder& start, Encoder& end, const double& tms) {
    double v_x_s = start.linear_vel.x();
    double v_y_s = start.linear_vel.y();
    double w_z_s = start.angular_vel.z();
    double v_x_e = end.linear_vel.x();
    double v_y_e = end.linear_vel.y();
    double w_z_e = end.angular_vel.z();

    double dt = end.timestamp_ - start.timestamp_;
    double ratio = (tms - start.timestamp_) / dt;  // ratio 越大， tms越靠近 end

    double inter_v = v_x_e * ratio + v_x_s * (1 - ratio);
    double inter_y = v_y_e * ratio + v_y_s * (1 - ratio);
    double inter_w = w_z_e * ratio + w_z_s * (1 - ratio);

    Encoder inter;
    inter.timestamp_ = tms;
    inter.linear_vel.x() = inter_v;
    inter.linear_vel.y() = inter_y;
    inter.angular_vel.z() = inter_w;
    return inter;
}

void EncoderProcessor::UpdateEncoder(const Encoder& encoder, const Input& input, State& x,
                                     ESKFShareState& share_state) {
    // LOG_INFO("wheel vel:{}", encoder.linear_vel.transpose());
    // LOG_INFO("input vel:{}", input.gyro.transpose());
    V3D z = V3D::Zero();
    Eigen::Matrix<double, 3, 33> H = Eigen::Matrix<double, 3, 33>::Zero();
    M3D R = M3D::Zero();
    // 计算残差
    M3D angv_crossmat;
    V3D gyro(input.gyro.x() - x.bg(0), input.gyro.y() - x.bg(1), input.gyro.z() - x.bg(2));  // 输入的角速度信息
    angv_crossmat << SKEW_SYM_MATRX(gyro);
    V3D wheel_vel = encoder.linear_vel;
    const double wheel_scale = 1.0;
    // 将imu的速度装换到wheel系
    PoseTrans T_IE = T_EI_.inverse();
    M3D R_wheelToImu = T_IE.R;
    V3D t_WheelToImu = T_IE.t;
    V3D current_vel = R_wheelToImu.transpose() * (x.rot.transpose() * x.vel + angv_crossmat * t_WheelToImu);
    // LOG_INFO("current_vel:{}", current_vel.transpose());
    V3D res = wheel_vel * wheel_scale - current_vel;
    // LOG_INFO("res:{}", res.transpose());
    // 求雅可比矩阵
    Eigen::Matrix<double, 3, 33> J;
    J.setZero();
    M3D rot_crossmat;
    V3D vel_in_body = x.rot.transpose() * x.vel;
    rot_crossmat << SKEW_SYM_MATRX(vel_in_body);
    // 残差对旋转的雅可比矩阵
    J.block<3, 3>(0, 3) = -R_wheelToImu.transpose() * rot_crossmat;
    // 残差对速度的雅可比矩阵
    J.block<3, 3>(0, 12) = -R_wheelToImu.transpose() * x.rot.transpose();
    // 对bg的雅可比矩阵
    M3D bg_crossmat;
    bg_crossmat << SKEW_SYM_MATRX(t_WheelToImu);
    J.block<3, 3>(0, 15) = -R_wheelToImu.transpose() * bg_crossmat;
    // 外参标定
    // scale 标定
    // 协方差
    M3D tmp_mat = -R_wheelToImu.transpose() * bg_crossmat;
    M3D cov_mat = M3D::Identity();
    cov_mat(0, 0) = wheel_cov_;
    if (gyro.norm() > 0.3) {
        cov_mat(1, 1) = wheel_vel(0) * gyro.norm();
    } else {
        cov_mat(1, 1) = nhc_y_;
    }
    cov_mat(2, 2) = nhc_z_;
    cov_mat = cov_mat + tmp_mat * tmp_mat.transpose() * 0.1;
    M3D info_mat;             // 最终的动态信息矩阵
    const double eps = 1e-6;  // 正则化参数，避免协方差矩阵奇异（逆不存在）
    // 先对协方差矩阵添加微小对角扰动，确保可逆
    M3D cov_reg = cov_mat + eps * M3D::Identity();
    info_mat = cov_reg.inverse();  // 信息矩阵 = 正则化协方差矩阵的逆
    share_state.valid = true;
    share_state.H33_.setZero();
    share_state.b33_.setZero();
    share_state.H33_ = J.transpose() * info_mat * J;
    share_state.b33_ = J.transpose() * info_mat * res;
    // LOG_INFO("iter:{},res:{}", share_state.iter_num, res.transpose());
}

}  // namespace slam