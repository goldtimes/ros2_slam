/*
 * @Author: lihang lihang@kilox.cn
 * @Date: 2025-09-08 13:41:48
 * @LastEditors: lihang lihang@kilox.cn
 * @LastEditTime: 2025-09-08 19:41:12
 * @FilePath: /fast_lvio_ws/src/open_slam/src/localizer/localizer.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "localizer/localizer.hh"
#include <boost/filesystem.hpp>

namespace slam {
Localizer::Localizer(const std::shared_ptr<SystemConfig>& system_config_ptr) : system_config_ptr_(system_config_ptr) {
    // use_meta_maps_ = system_config_ptr_->use_meta_maps;
    if (use_meta_maps_) {
        map_update_thread_ = std::make_shared<std::thread>(&Localizer::MapUpdate, this);
    }
    map_register_thread_ = std::make_shared<std::thread>(&Localizer::MapRegister, this);

    global_map_filter_.setLeafSize(0.3, 0.3, 0.3);
    // 分配空间
    global_map_.reset(new PointCloudXYZI());
    global_map_tree_.reset(new PointXYZITree());

    local_state_ = LOCAL_STATE::NOT_INIT;
}
Localizer::~Localizer() {
    if (use_meta_maps_) {
        if (map_update_thread_->joinable()) {
            map_update_thread_->join();
        }
    }
    if (map_register_thread_->joinable()) {
        map_register_thread_->join();
    }
}

// 设置地图信息
void Localizer::SetMetaMaps(const std::map<std::string, std::vector<std::shared_ptr<MetaInfo>>>& maps) {
    ids_metamap_map_ = maps;
}

void Localizer::SetInitPose(const PoseTrans& init_pose, int level, const std::string& map_id) {
}

// 设置单张的全局地图
void Localizer::SetMaps(const std::string& pcd_path) {
    // 加载全局地图
    if (boost::filesystem::exists(pcd_path)) {
        pcl::io::loadPCDFile(pcd_path, *global_map_);
    }
    // 构建kd树
    global_map_tree_->setInputCloud(global_map_);
    map_loaded_ = true;
}

void Localizer::SetTrajCloud(const PointCloudXYZI::Ptr& traj_cloud) {
    // 分配空间
    traj_cloud_.reset(new PointCloudXYZI());
    traj_cloud_tree_.reset(new PointXYZITree());
    traj_cloud_ = traj_cloud;
    traj_cloud_tree_->setInputCloud(traj_cloud_);
    traj_cloud_loaded_ = true;
}

void Localizer::MapUpdate() noexcept {
}
void Localizer::MapRegister() noexcept {
    LOG_INFO("MapRegister Run");
    // 等待局部地图更新了
    while (true) {
    }
}
}  // namespace slam