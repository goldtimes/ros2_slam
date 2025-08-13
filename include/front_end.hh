#pragma once

#include "commons.hh"

namespace slam {
class System;
class StaticImuInit;

enum class FrontEndStatus {
    IMU_INIT,  // 静态初始化
    MAP_INIT,  // 里程计初始化
    MAPPING,   // lidar align
    LOST,      // 里程计漂移
};
// 前端里程计
class FrontEnd {
   public:
    FrontEnd(System* system);
    ~FrontEnd();

    void Run();

    FrontEndStatus getFrontEndStatus() {
        return front_end_status_;
    }

   private:
    bool GetMeasureGroup(MeasureGroup& measures);

   private:
    System* system_ = nullptr;
    MeasureGroup measure_group_;
    FrontEndStatus front_end_status_ = FrontEndStatus::IMU_INIT;
    bool opt_wheel_ = true;
    bool opt_gnss_ = true;
    bool lidar_pushed_ = false;
    double lidar_mean_scantime_ = 0.0;
    int scan_count_ = 0;
    bool use_encoder_ = false;
    bool use_gnss_ = false;

    std::shared_ptr<StaticImuInit> static_imu_init_ptr_;
};
}  // namespace slam
