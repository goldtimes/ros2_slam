#pragma once

#include "eigen_type.hh"
#include "nav_state.hh"

namespace slam {

/**
 * LIO 中的ESKF重写 来自高博的吐槽
 *
 * MTK
 * 为了实现状态量的自由组合，搞了一套宏函数来实现自定状态变量各维度的索引，然而实际当中状态变量基本是固定的，并不希望这样子用。。
 * 而且宏函数无法调试，人类也看不到展开之后的宏长啥样
 * 重写的版本只使用固定的NavState，内部逻辑也在NavState里面定义，不做无谓的拓展了 （并没有谁去拓展那玩意）
 * n = 23, m = 24
 */
class ESKF {
   public:
    static constexpr int process_noise_dim_ = 12;                                            // 过程噪声维度
    static constexpr int state_dim_ = NavState::dim;                                         // 状态维度
    using StateVecType = NavState::VecState;                                                 // 状态向量类型
    using CovType = Eigen::Matrix<double, state_dim_, state_dim_>;                           // 协方差矩阵 23x23
    using ProcessNoiseType = Eigen::Matrix<double, process_noise_dim_, process_noise_dim_>;  // 12x12
    /// ESKF 观测类型
    enum class ObsType {
        LIDAR,                  // 开源版本只有Lidar
        WHEEL_SPEED,            // 单独的轮速观测
        WHEEL_SPEED_AND_LIDAR,  // 轮速+Lidar
        ACC_AS_GRAVITY,         // 重力作为加计观测量
        GPS,                    // GPS/RTk 六自由度位姿
        BIAS,
    };

    explicit ESKF(const NavState& x = NavState(), const CovType& P = CovType::Identity(), bool use_aa = true)
        : x_(x), P_(P), use_aa_(use_aa) {
    }

    ~ESKF() {
    }
    // 自定义观测模型，无论那种模型的观测，都需要满足这个结构体的格式
    struct CustomObservationModel {
        bool valid_ = true;
        bool converge_ = true;

        Eigen::Matrix<double, Eigen::Dynamic, 1> residual_;          // residual: z-Hx
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h_x_;  // dr/dx, H阵
        Eigen::Matrix<double, Eigen::Dynamic, 1> s_;                 // ?
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R_;    // 观测噪声矩阵

        double lidar_residual_mean_ = 0;
        double lidar_residual_max_ = 0;
    };

    // 观测函数
    using CustomObsFunction = std::function<void(NavState& s, CustomObservationModel& obs)>;

    struct Options {
        CustomObsFunction lidar_obs_func_;           // 雷达观测函数
        CustomObsFunction wheelspeed_obs_func_;      // 轮速观测函数
        CustomObsFunction acc_as_gravity_obs_func_;  // 加计观测函数
        CustomObsFunction gps_obs_func_;
        CustomObsFunction bias_obs_func_;
        int max_iterations_ = 4;
        StateVecType epsi_;    // 收敛条件
        bool use_aa_ = false;  // use anderson accleration
    };

    void Init(Options options) {
        lidar_obs_func_ = options.lidar_obs_func_;
        wheelspeed_obs_func_ = options.wheelspeed_obs_func_;
        acc_as_gravity_obs_func_ = options.acc_as_gravity_obs_func_;
        gps_obs_func_ = options.gps_obs_func_;
        bias_obs_func_ = options.bias_obs_func_;
        maximum_iter_ = options.max_iterations_;
        limit_ = options.epsi_;
        use_aa_ = options.use_aa_;
    }

    // 预测
    void Predict(const double& dt, const ProcessNoiseType& Q, const V3D& gyro, const V3D& acce);

    // 更新
    void Update(ObsType obs, const double& R);

    // accessors
    const NavState& GetX() const {
        return x_;
    }
    const CovType& GetP() const {
        return P_;
    }
    const double& GetStamp() const {
        return stamp_;
    }

    void ChangeX(const NavState& state) {
        x_ = state;
    }
    void ChangeP(const CovType& P) {
        P_ = P;
    }
    void ChangeStamp(const double& stamp) {
        stamp_ = stamp;
    }

    void SetUseAA(bool use_aa) {
        use_aa_ = use_aa;
    }
    void SetTime(double timestamp) {
        x_.timestamp_ = timestamp;
    }

    /// 迭代次数
    int GetIterations() const {
        return iterations_;
    }
    /// 最终平均观测误差
    double GetFinalRes() const {
        return final_res_;
    }

   private:
    double stamp_ = 0.0;

    NavState x_;
    CovType P_ = CovType::Identity();
    CovType F_x1_ = CovType::Identity();
    CovType L_ = CovType ::Identity();

    CustomObservationModel custom_obs_model_;
    CustomObsFunction lidar_obs_func_;
    CustomObsFunction wheelspeed_obs_func_;
    CustomObsFunction acc_as_gravity_obs_func_;
    CustomObsFunction gps_obs_func_;
    CustomObsFunction bias_obs_func_;

    int maximum_iter_ = 0;  // 最大迭代次数
    StateVecType limit_;

    int iterations_ = 0;
    double final_res_ = 0.0;

    /// anderson acceleration?
    bool use_aa_ = false;
    // AndersonAcceleration<double, state_dim_, 10> aa_;
};
}  // namespace slam