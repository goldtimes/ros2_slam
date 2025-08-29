#pragma once

#include "commons.hh"
#include "state.hh"

namespace slam {

class System;
class Propogator;
class IESKF;
class LidarRegister;

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

    const double GetCurentTime() const {
        return measure_group_.lidar_beg_time;
    }

    State GetCurentNavState();

    // lidar坐标系原始数据
    const PointCloudPtr GetCloudInLidarLink() const;

    // robot_link坐标系点云
    const PointCloudPtr GetCloudInRobotLink() const;

    // odom坐标系点云
    const PointCloudPtr GetCloudInOdomLink() const;

    // std::shared_ptr<VoxelMap> GetVoxelMap() const {
    //     return lidar_register_ptr_;
    // }

    const Matrix23d GetCov() const;

    bool InitializeImu(std::deque<IMU>& imus);

   private:
    bool GetMeasureGroup(MeasureGroup& measures);
    void AllocateMemory();

   private:
    System* system_;
    MeasureGroup measure_group_;
    FrontEndStatus front_end_status_ = FrontEndStatus::IMU_INIT;
    bool opt_wheel_ = true;
    bool opt_gnss_ = true;
    bool lidar_pushed_ = false;
    double lidar_mean_scantime_ = 0.0;
    int scan_count_ = 0;
    bool use_encoder_ = false;
    bool use_gnss_ = false;

    std::shared_ptr<Propogator> propogator_ptr_;
    std::shared_ptr<IESKF> kf_ptr_;
    State nav_state_;

    // 坐标信息
    // lidar到imu的外参
    PoseTrans T_IL;
    // 激光雷达到robot的外参
    PoseTrans T_BL;
    // imu到robot的外参
    PoseTrans T_BI;
    // IMU到轮速计到外参，轮速计的中心暂时可以理解机器人的中心，如果后轮驱动的车型那就不一样，我们这里是差速模型
    PoseTrans T_EI;
    // 轮速计变换到world坐标系下，主要是朝向
    PoseTrans T_WE;

    std::shared_ptr<LidarRegister> lidar_register_ptr_;
    PointCloudPtr undistort_cloud_lidar_;
    PointCloudPtr undistort_cloud_robot_;
    PointCloudPtr undistort_cloud_odom_;
};
}  // namespace slam
