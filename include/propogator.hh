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

class Propogator {
   public:
    Propogator(std::shared_ptr<SystemConfig> config_, std::shared_ptr<IESKF> kf);
    ~Propogator();

    bool Initialize(MeasureGroup& meas);

    bool GetInitSuccess() {
        return init_success_;
    }

    // 状态传播
    void PropogateAndUndistort(MeasureGroup& meas, PointCloudPtr& out_cloud);

    void UndistortLidar(const PointCloudPtr& cloud_in, PointCloudPtr& cloud_out);
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

    double current_imu_time_;
    double last_propagate_time_;
    std::deque<NominalState> imu_states_;
    std::deque<IMU> imu_caches_;

    PoseTrans T_IL_;
};
}  // namespace slam