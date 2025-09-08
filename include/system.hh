#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include "commons.hh"
#include "lidar_process.hh"
#include "lidar_register/voxel_map.hh"
#include "logger.hh"
#include "sensors.hh"
#include "state.hh"

namespace slam {

// 系统参数
class SystemConfig;

// lidar process
class LidarProcess;
class FrontEnd;
class Localizer;
struct MetaInfo;

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

    bool IsSystemInit() const {
        return system_init_.load();
    }

    void SetSystemInit(bool init) {
        system_init_.store(init);
    }

    const std::shared_ptr<Localizer> GetLocalizer();
    void SetInitPose(const PoseTrans& init_pose, int level = 0, const std::string& map_id = "");
    void SetMetaInfo(const std::map<std::string, std::vector<std::shared_ptr<MetaInfo>>>& meta_maps);

    const PoseTrans GetTransformEncodeToWorld() const;

    const State GetCurentNavState() const;

    const double GetSystemTime() const;

    const PoseTrans GetTLidarToImu() const;

    const PoseTrans GetLidarToBaselink() const;

    const PoseTrans GetImuToBaselink() const;
    // lidar坐标系原始数据
    const PointCloudPtr GetCloudInLidarLink() const;

    // robot_link坐标系点云
    const PointCloudPtr GetCloudInRobotLink() const;

    // odom坐标系点云
    const PointCloudPtr GetCloudInOdomLink() const;

    // std::shared_ptr<VoxelMap> GetVoxelMap() const {
    //     return front_end_ptr_->GetVoxelMap();
    // }
    const M33D GetCov() const;
    const M3D GetGnssHeading() const;

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

    std::shared_ptr<Localizer> localizer_ptr_;

    std::atomic<bool> system_init_;

    // 坐标信息
    PoseTrans T_IL;
    PoseTrans T_BL;
    PoseTrans T_BI;
};
}  // namespace slam