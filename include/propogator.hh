#pragma once
#include <memory>
#include "commons.hh"
#include "logger.hh"

// 状态的前向传播和后向传播
namespace slam {

class IESKF;
class StaticImuInit;
class SystemConfig;
struct NominalState;

struct Pose {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc;
    V3D gyro;
    M3D rot;
    V3D pos;
    V3D vel;
    Pose();
    Pose(double t, V3D a, V3D g, V3D v, V3D p, Eigen::Matrix3d r) : offset(t), acc(a), gyro(g), vel(v), pos(p), rot(r) {
    }
    double offset;
};

class Propogator {
   public:
    Propogator(std::shared_ptr<SystemConfig> config_, std::shared_ptr<IESKF> kf);
    ~Propogator();

    bool Initialize(MeasureGroup& meas);

    bool GetInitSuccess() {
        return init_success_;
    }

    // 状态传播
    void PropogateState(MeasureGroup& meas);

    void UndistortLidar(MeasureGroup& meas, PointCloudPtr& cloud_out);
    NominalState GetNominalState() const;

   private:
    // 静态初始化器
    std::shared_ptr<StaticImuInit> imu_init_ptr_;
    // 系统配置
    std::shared_ptr<SystemConfig> system_config_ptr_;
    // 状态估计器
    std::shared_ptr<IESKF> kf_;

    bool init_success_ = false;

    IMU last_imu_;

    V3D cov_acc_;
    V3D cov_gyro_;
    V3D cov_bias_acc_;
    V3D cov_bias_gyro_;
    M12D Q_;

    std::deque<IMU> imu_init_buffer_;
    double current_imu_time_;
    double last_propagate_time_;
    std::deque<NominalState> imu_states_;
    std::deque<IMU> imu_caches_;

    std::deque<Pose> imu_pose_cache_;
    V3D last_acc_;
    V3D last_gyro_;

    PoseTrans T_IL_;
};
}  // namespace slam