/*
 * @Author: lihang lihang@kilox.cn
 * @Date: 2025-08-29 14:15:00
 * @LastEditors: lihang lihang@kilox.cn
 * @LastEditTime: 2025-09-05 14:02:41
 * @FilePath: /fast_lvio_ws/src/open_slam/include/ieskf.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include "commons.hh"
#include "logger.hh"
#include "state.hh"

namespace slam {

using lidar_loss_func = std::function<void(State&, ESKFShareState&)>;
using wheel_loss_func = std::function<void(const Encoder&, const Input&, State&, ESKFShareState&)>;
using gnss_loss_func = std::function<void(const GNSS&, State&, ESKFShareState&)>;
using stop_func = std::function<bool(const V33D& dx)>;

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

    void SetWheelLossFunc(wheel_loss_func loss_func) {
        wheel_loss_func_ = loss_func;
    }

    void SetGnssLossFunc(gnss_loss_func loss_func) {
        gnss_loss_func_ = loss_func;
    }

    void SetStopFunc(stop_func func) {
        stop_func_ = func;
    }

    void Predict(const Input& input, double dt, const M12D& Q);

    void UpdateLidar();

    void UpdateEncoder(const Encoder& encoder, const Input& input);

    void UpdateGnss(const GNSS& gnss);

    const State& GetState() const {
        return x_;
    }

    const M33D& GetCov() const {
        return P_;
    }

    State& x() {
        return x_;
    }

    M33D& Cov() {
        return P_;
    }

   private:
    // 状态量
    State x_;
    // 协方差
    M33D P_ = M33D::Zero();
    // 最大迭代次数
    size_t max_iter_num_ = 10;
    // 激光损失函数
    lidar_loss_func lidar_loss_func_;
    // 轮速计损失函数
    wheel_loss_func wheel_loss_func_;
    // gnss损失函数
    gnss_loss_func gnss_loss_func_;
    // 停止函数
    stop_func stop_func_;
    // 预测矩阵
    M33D F_ = M33D::Zero();
    // 输入矩阵
    Eigen::Matrix<double, 33, 12> G_ = Eigen::Matrix<double, 33, 12>::Zero();

    M33D H_;
    V33D b_;
    double current_time_;
};
}  // namespace slam
