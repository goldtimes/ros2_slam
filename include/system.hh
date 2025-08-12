#pragma once

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

    void run();

    const std::shared_ptr<SystemConfig> GetSystemConfig() const {
        return system_config_ptr_;
    }

   private:
    void InitConfigParams();

   private:
    std::string config_path_;
    std::shared_ptr<SystemConfig> system_config_ptr_;
};
}  // namespace slam