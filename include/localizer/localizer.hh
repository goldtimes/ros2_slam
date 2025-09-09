/*
 * @Author: lihang lihang@kilox.cn
 * @Date: 2025-09-08 13:41:59
 * @LastEditors: lihang lihang@kilox.cn
 * @LastEditTime: 2025-09-09 13:52:22
 * @FilePath: /fast_lvio_ws/src/open_slam/include/localizer/locallizer.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

// gtsam相关
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <memory>
#include "eigen_type.hh"
#include "logger.hh"
#include "pointcloud_utils.hh"
#include "pose_trans.hh"

namespace slam {

class SystemConfig;

// 定义定位中的状态量
enum class LOCAL_STATE {
    NOT_INIT,      // 未初始化
    INITING,       // 初始化中
    INIT_FAILED,   // 初始化失败
    INITED,        // 定位成功
    LOST,          // 定位丢失
    RELOCALIZING,  // 重定位中
};

// 存储位姿和分数
struct ScorePose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double score = 0.0;
    PoseTrans pose;
    ScorePose(const PoseTrans& p, double s) : pose(p), score(s) {
    }
    bool operator<(const ScorePose& other) const {
        return score < other.score;
    }
};

// 图元信息
struct MetaInfo {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    double load_time;
    double expired_time;
    PointCloudXYZI::Ptr map_pcd;
    std::string name;
    bool is_active = false;
    bool is_old = false;
    double x, y;
    int level;
    PoseTrans T;
    std::string identity;
};

class Localizer {
   public:
    Localizer(const std::shared_ptr<SystemConfig>& system_config_ptr);
    ~Localizer();

    // 获得当前的地图信息
    MetaInfo GetCurrMetaInfo() const {
        return curr_meta_info_;
    }
    // 设置地图信息
    void SetMetaMaps(const std::map<std::string, std::vector<std::shared_ptr<MetaInfo>>>& maps);
    // 设置单张的全局地图
    void SetMaps(const std::string& pcd_path);
    void SetInitPose(const PoseTrans& init_pose, int level = 0, const std::string& map_id = "");

    void SetTrajCloud(const PointCloudPtr& traj_cloud);

    // void SetLidarCloud(const PointCloudXYZI::Ptr& lidar_cloud);

   private:
    bool LoadMapByPose(const PoseTrans& init_pose = PoseTrans());

    // 地图更新线程
    void MapUpdate() noexcept;
    // 地图注册线程
    void MapRegister() noexcept;

   private:
    std::shared_ptr<SystemConfig> system_config_ptr_;
    LOCAL_STATE local_state_ = LOCAL_STATE::NOT_INIT;
    // 存储地图id和地图的图元列表
    std::map<std::string, std::vector<std::shared_ptr<MetaInfo>>> ids_metamap_map_;

    // 开启两个线程，一个更新加载的地图，一个做后台的配准
    bool use_meta_maps_ = false;
    std::shared_ptr<std::thread> map_update_thread_;
    std::shared_ptr<std::thread> map_register_thread_;
    pcl::VoxelGrid<PointType> global_map_filter_;

    // 全局地图
    PointCloudPtr global_map_;
    // 全局地图的kd树
    PointTree::Ptr global_map_tree_;

    // 轨迹点云
    PointCloudPtr traj_cloud_;
    // 轨迹点云的kd树
    PointTree::Ptr traj_cloud_tree_;

    bool traj_cloud_loaded_ = false;

    bool map_loaded_ = false;

    MetaInfo curr_meta_info_;

    // gtsam 相关
    gtsam::NonlinearFactorGraph gtSAMgraph_;
    gtsam::Values initialEstimate_;
    gtsam::Values optimizedEstimate_;
    gtsam::ISAM2 isam_;
    gtsam::Values isamCurrentEstimate_;

    // 待优化的里程计位姿
    std::vector<PoseTrans> keyframe_poses_;

    bool get_init_pose_ = false;
    bool get_curr_lidar_ = false;
    PoseTrans init_guess_pose_;
};
}  // namespace slam