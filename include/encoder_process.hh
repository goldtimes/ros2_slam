#pragma once

#include "commons.hh"
#include "eigen_type.hh"
#include "state.hh"

namespace slam {

class SystemConfig;

class EncoderProcessor {
   public:
    EncoderProcessor(const std::shared_ptr<SystemConfig>& config_ptr);
    ~EncoderProcessor() = default;
    // 添加轮速计数据
    void AddEncoder(const std::deque<Encoder>& encoders);

    // 计算一帧雷达之间的轮速计位移
    bool Propagation(PoseTrans& delta_pose, const double begin_time, const double end_time);

    Encoder interplate(Encoder& start, Encoder& end, const double& tms);

    void UpdateEncoder(const Encoder& encoder, const Input& input, State& x, ESKFShareState& share_state);

    void SetTransformWheelToImu(const PoseTrans& T_EI) {
        T_EI_ = T_EI;
    }

   private:
    //    配置
    std::shared_ptr<SystemConfig> config_ptr_;
    bool is_static_;
    //    存放数据
    std::deque<Encoder> encoder_queue_;
    PoseTrans T_EI_;
    double wheel_cov_;
    double nhc_y_;
    double nhc_z_;
};
}  // namespace slam