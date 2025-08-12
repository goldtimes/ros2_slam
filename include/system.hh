#pragma once

#include <deque>
#include "commons.hh"
#include "logger.hh"
#include "sensors.hh"

namespace slam {

// 系统参数
class SystemConfig;
// 传感器处理类
class SensorManager;
class System {
   public:
    System() = delete;
    explicit System(const std::string& config_path);
    ~System();

    void Run();

    const std::shared_ptr<SystemConfig> GetSystemConfig() const {
        return system_config_ptr_;
    }

    void AddIMU(const IMU& imu);
    void AddEncoder(const Encoder& encoder);
    // void AddLidar(const Lidar& lidar);
    void AddGNSS(const GNSS& gnss);

   private:
    void InitConfigParams();

   private:
    std::string config_path_;
    std::shared_ptr<SystemConfig> system_config_ptr_;

    std::deque<IMU> imu_queue_;
    std::deque<Encoder> encoder_queue_;
    // std::deque<Lidar> lidar_queue_;
    std::deque<GNSS> gnss_queue_;

};
}  // namespace slam