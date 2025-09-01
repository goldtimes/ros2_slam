#include "encoder_process.hh"
#include "system_config.hh"

namespace slam {
EncoderProcessor::EncoderProcessor(const std::shared_ptr<SystemConfig>& config_ptr) : config_ptr_(config_ptr) {
    is_static_ = true;
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
    LOG_INFO("EncoderProcessor::Propagation: selected_data.size() = {}", selected_data.size());
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

}  // namespace slam