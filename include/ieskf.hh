#pragma once
#include "commons.hh"
#include "logger.hh"
#include "state.hh"

namespace slam {

using lidar_loss_func = std::function<void(State&, ESKFShareState&)>;
using stop_func = std::function<bool(const Vector23d& dx)>;

// 迭代卡尔曼滤波器
class IESKF {
   public:
    static int P_ID, R_ID, ER_ID, EP_ID, V_ID, BG_ID, BA_ID, G_ID;
    IESKF() {
        LOG_INFO("IESKF init");
    }
    ~IESKF() = default;

    M3D Jr(const V3D& inp);
    M3D JrInv(const V3D& inp);

    void SetMaxIterNum(size_t max_iter_num) {
        max_iter_num_ = max_iter_num;
    }

    void SetLidarLossFunc(lidar_loss_func loss_func) {
        lidar_loss_func_ = loss_func;
    }

    void SetStopFunc(stop_func func) {
        stop_func_ = func;
    }

    void Predict(const Input& input, double dt, const M12D& Q);

    void Update();

    const State& GetState() const {
        return x_;
    }

    const Matrix23d& GetCov() const {
        return P_;
    }

    State& x() {
        return x_;
    }

    Matrix23d& Cov() {
        return P_;
    }

   private:
    // 状态量
    State x_;
    // 协方差
    Matrix23d P_ = Matrix23d::Zero();
    // 最大迭代次数
    size_t max_iter_num_ = 10;
    // 损失函数
    lidar_loss_func lidar_loss_func_;
    // 停止函数
    stop_func stop_func_;
    // 预测矩阵
    Matrix23d F_ = Matrix23d::Zero();
    // 输入矩阵
    Eigen::Matrix<double, 23, 12> G_ = Eigen::Matrix<double, 23, 12>::Zero();

    Matrix23d H_;
    Vector23d b_;
    double current_time_;
};
}  // namespace slam
