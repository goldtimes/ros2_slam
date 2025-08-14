#pragma once
#include <functional>
#include "commons.hh"
#include "logger.hh"
#include "state.hh"

namespace slam {

using lidar_loss_func = std::function<void(NavState&, ESKFShareState&)>;
using stop_func = std::function<bool(const V21D& dx)>;

// 迭代卡尔曼滤波器
class IESKF {
   public:
    IESKF() {
        LOG_INFO("IESKF init");
    }
    ~IESKF() = default;

    M3D Jr(const V3D& inp);
    M3D JrInv(const V3D& inp);

    void SetMaxIterNum(size_t max_iter_num) {
        max_iter_num_ = max_iter_num;
    }

    void SetLidarLossFunc(lidar_loss_func lidar_loss_func) {
        lidar_loss_func_ = lidar_loss_func;
    }

    void SetStopFunc(stop_func stop_func) {
        stop_func_ = stop_func;
    }

    void Predict(const Input& input, double dt, const M12D& Q);

    void Update();

    const NavState& GetState() const {
        return state_;
    }

    const M21D& GetCov() const {
        return cov_;
    }

    NavState& State() {
        return state_;
    }

    M21D& Cov() {
        return cov_;
    }

    

   private:
    // 状态量
    NavState state_;
    // 协方差
    M21D cov_ = M21D::Zero();
    // 最大迭代次数
    size_t max_iter_num_ = 10;
    // 损失函数
    lidar_loss_func lidar_loss_func_;
    // 停止函数
    stop_func stop_func_;
    // 预测矩阵
    M21D m_F_ = M21D::Zero();
    // 输入矩阵
    Eigen::Matrix<double, 21, 12> m_G_ = Eigen::Matrix<double, 21, 12>::Zero();
    double current_time_;
};
}  // namespace slam
