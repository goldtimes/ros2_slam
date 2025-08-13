#pragma once

#include "commons.hh"

namespace slam {
class System;
// 前端里程计
class FrontEnd {
   public:
    FrontEnd(System* system);
    ~FrontEnd();

    void Run();

   private:
    bool GetMeasureGroup(MeasureGroup& measures);

   private:
    System* system_ = nullptr;
    MeasureGroup measure_group_;

    bool opt_wheel_ = true;
    bool opt_gnss_ = true;
    bool lidar_pushed_ = false;
    double lidar_mean_scantime_ = 0.0;
    int scan_count_ = 0;
    bool use_encoder_ = false;
    bool use_gnss_ = false;
};
}  // namespace slam
