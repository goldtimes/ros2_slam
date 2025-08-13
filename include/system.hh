#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include "commons.hh"
#include "lidar_process.hh"
#include "logger.hh"
#include "sensors.hh"

namespace slam {

// 系统参数
class SystemConfig;

// lidar process
class LidarProcess;

class FrontEnd;

class System {
   public:
    System() = delete;
    explicit System(const std::string& config_path);
    ~System();

    void Run();

    const std::shared_ptr<SystemConfig> GetSystemConfig() const {
        return system_config_ptr_;
    }
    const std::shared_ptr<LidarProcess> GetLidarProcess() const {
        return lidar_process_ptr_;
    }

    void AddIMU(const IMU& imu);
    void AddEncoder(const Encoder& encoder);
    void AddLidar(const PointCloudPtr& cloud, const double cloud_time);
    void AddGNSS(const GNSS& gnss);
    // 重置系统
    void reset();

   public:
    std::condition_variable m_buff_cv_;
    std::mutex m_buf_mutex_;
    std::deque<IMU> imu_queue_;              // imu数据
    std::deque<Encoder> encoder_queue_;      // 编码器数据
    std::deque<PointCloudPtr> lidar_queue_;  // lidar数据
    std::deque<double> lidar_time_queue_;    // 雷达帧首数据
    std::deque<GNSS> gnss_queue_;            // gnss数据

   private:
    void InitConfigParams();

   private:
    std::string config_path_;
    std::shared_ptr<SystemConfig> system_config_ptr_;

    std::shared_ptr<LidarProcess> lidar_process_ptr_;
    // 前端类和前端线程
    FrontEnd* front_end_ptr_ = nullptr;
    std::thread* front_end_thread_ptr_ = nullptr;
};
}  // namespace slam