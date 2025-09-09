/*
 * @Author: lihang lihang@kilox.cn
 * @Date: 2025-09-08 13:41:48
 * @LastEditors: lihang lihang@kilox.cn
 * @LastEditTime: 2025-09-09 17:36:53
 * @FilePath: /fast_lvio_ws/src/open_slam/src/localizer/localizer.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "localizer/localizer.hh"
#include <boost/filesystem.hpp>
#include <chrono>
#include "system_config.hh"

namespace slam {
Localizer::Localizer(const std::shared_ptr<SystemConfig>& system_config_ptr) : system_config_ptr_(system_config_ptr) {
    use_meta_maps_ = system_config_ptr_->localizer_config_.use_meta_maps;
    if (use_meta_maps_) {
        map_update_thread_ = std::make_shared<std::thread>(&Localizer::MapUpdate, this);
    }
    map_dir_ = system_config_ptr->localizer_config_.local_map_dir;
    map_register_thread_ = std::make_shared<std::thread>(&Localizer::MapRegister, this);

    global_map_filter_.setLeafSize(system_config_ptr->localizer_config_.global_map_filter_size,
                                   system_config_ptr->localizer_config_.global_map_filter_size,
                                   system_config_ptr->localizer_config_.global_map_filter_size);

    local_state_ = LOCAL_STATE::MAP_NOT_LOAD;
    AllocateMemory();
}

void Localizer::AllocateMemory() {
    // 分配空间
    curr_lidar_cloud_.reset(new PointCloudType());
    curr_submap_cloud_.reset(new PointCloudType());
    global_map_.reset(new PointCloudType());
    global_map_tree_.reset(new PointTree());
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

void Localizer::SetLidarCloud(const PointCloudPtr& lidar_cloud, const PoseTrans& T_LtoO) {
    curr_lidar_cloud_ = lidar_cloud;
    get_new_lidar_ = true;
    T_LtoO_ = T_LtoO;
    if (local_state_ == LOCAL_STATE::INITED) {
        update_map_ = true;
        guess_pose_ = T_OtoM_ * T_LtoO;
    }
}

void Localizer::SetSubmapCloud(const PointCloudPtr& submap_cloud, const PoseTrans& T_LtoO) {
    curr_submap_cloud_ = submap_cloud;
    get_new_submap_ = true;
    update_T_LtoO_ = T_LtoO;
}

void Localizer::SetInitPose(const PoseTrans& init_pose, int level, const std::string& map_id) {
    get_init_pose_ = true;
    // 设置机器人的初始位置
    guess_pose_ = init_pose;
    // 通知地图更新线程更新地图
    update_map_ = true;
    // 记录当前的地图信息
    curr_meta_info_.level = level;
    curr_map_ = std::make_pair(map_id, curr_meta_info_);
    LOG_INFO("map_identity:{}", map_id);
    LOG_INFO("init posisition:{}", guess_pose_.t.transpose());
    LOG_INFO("init orientation:{}", guess_pose_.RPY().transpose());
    // 修改初始值的高度
    if (traj_cloud_loaded_) {
        PointType init_pt;
        init_pt.x = guess_pose_.t(0);
        init_pt.y = guess_pose_.t(1);
        init_pt.z = guess_pose_.t(2);
        // 查找最近的点
        std::vector<int> indices;
        std::vector<float> distances;
        traj_cloud_tree_->nearestKSearch(init_pt, 1, indices, distances);
        if (indices.size() > 0) {
            guess_pose_.t(2) = traj_cloud_->points[indices[0]].z;
        }
        if (guess_pose_.t(2) < 0) {
            guess_pose_.t(2) = 0;
        }
        if (guess_pose_.t(2) > 2.0) {
            guess_pose_.t(2) /= 2;
        }
        LOG_INFO("change predict height to {}", guess_pose_.t.transpose());
    }
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

void Localizer::SetTrajCloud(const PointCloudPtr& traj_cloud) {
    // 分配空间
    traj_cloud_.reset(new PointCloudType());
    traj_cloud_tree_.reset(new PointTree());
    traj_cloud_ = traj_cloud;
    traj_cloud_tree_->setInputCloud(traj_cloud_);
    traj_cloud_loaded_ = true;
}

void Localizer::MapUpdate() noexcept {
    LOG_INFO("[LOC] MapUpdate Run");
    ros::Rate rate(20);
    while (ros::ok()) {
        if (update_map_) {
            update_map_ = false;
        }
        LoadMapByPose(guess_pose_);
        rate.sleep();
    }
}
void Localizer::MapRegister() noexcept {
    LOG_INFO("MapRegister Run");
    // 等待局部地图更新了
    ros::Rate rate(20);
    while (ros::ok()) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        switch (local_state_) {
            // 需要在外面设置 NOT_INIT状态
            case LOCAL_STATE::NOT_INIT:
                LOG_INFO("[LOC] not init");
                if (get_init_pose_ && get_new_lidar_) {
                    StartInitialization();
                    get_init_pose_ = false;
                    get_new_lidar_ = false;
                }
                break;
            case LOCAL_STATE::INITING:  // cast 不会创建作用域
                CheckInitializationStatus();
                break;
            case LOCAL_STATE::INITED:
                break;
            case LOCAL_STATE::INIT_FAILED:
                break;
            case LOCAL_STATE::LOST:
                break;
            case LOCAL_STATE::RELOCALIZING:
                break;
            case LOCAL_STATE::MAP_NOT_LOAD:
                LOG_INFO("[LOC] map not load");
                break;
        }
        rate.sleep();
    }
}

void Localizer::StartInitialization() {
    // 初始化配准
    if (is_initializing_) {
        // 如果正在初始化
        CancelInit();
        // 等待初始化线程结束
        if (init_future_worker_.valid()) {
            init_future_worker_.wait();
        }
    }
    // 重新开始用新的位姿初始化
    cancel_init_ = false;
    is_initializing_ = true;
    local_state_ = LOCAL_STATE::INITING;
    // 启动异步初始化任务
    init_future_worker_ = std::async(std::launch::async, &Localizer::InitSearch, this);
}

void Localizer::CheckInitializationStatus() {
    if (!is_initializing_ || !init_future_worker_.valid()) {
        return;
    }
    // 再一次收到了初始化的位置和点云
    if (get_init_pose_ && get_new_lidar_) {
        local_state_ = LOCAL_STATE::NOT_INIT;
        return;
    }
    // 检查初始化的线程是否计算完成,不阻塞当前线程 ready,timeout,deferred三种状态
    std::future_status status = init_future_worker_.wait_for(std::chrono::milliseconds(0));
    if (status == std::future_status::ready) {
        // 初始化完成
        bool success = init_future_worker_.get();
        is_initializing_ = false;
        if (success) {
            local_state_ = LOCAL_STATE::INITED;
        } else {
            local_state_ = LOCAL_STATE::INIT_FAILED;
        }
    }
}
// 取消初始化
void Localizer::CancelInit() {
    if (is_initializing_) {
        cancel_init_ = true;
    }
}

bool Localizer::InitSearch() {
    bool success = false;
    // 需要检查cancel_init_标志位
    return success;
}

bool Localizer::LoadMapByPose(const PoseTrans& T_BtoM) {
    Eigen::Vector2d pos = T_BtoM.t.head<2>();
    // 找不到当前的地图
    if (ids_metamap_map_.find(curr_map_.first) == ids_metamap_map_.end()) {
        LOG_ERROR("map {} not found", curr_map_.first);
        return false;
    }
    // 将当前图元附近的加载进来
    bool update = false;
    double min_dis = 1e5;
    int new_load_cnt = 0;
    for (auto& meta_info : ids_metamap_map_[curr_map_.first]) {
        // 如果距离过远
        double dis_to_robot = dis_a_b(meta_info->x, meta_info->y, pos[0], pos[1]);
        // 减小地图大小的加载
        if (dis_to_robot > 50) continue;
        if (dis_to_robot <= min_dis && dis_to_robot <= 25 * sqrt(2)) {
            min_dis = dis_to_robot;
            curr_meta_info_.name = meta_info->name;
            // last_map_identity = meta_info->identity;
            // cur_map_identity = meta_info->identity;
        }

        // 如果已经加载了
        if (meta_info->is_active) continue;
        // 如果图元里面pcd文件不存在
        std::stringstream ss;
        ss << map_dir_ << "/" << meta_info->identity << "/" << meta_info->name << "/data.pcd";
        if (!boost::filesystem::exists(ss.str())) {
            LOG_INFO("file {} not exist", ss.str());
            continue;
        }

        // 从文件加载
        pcl::io::loadPCDFile<PointType>(ss.str(), *meta_info->map_pcd);
        meta_info->is_active = true, meta_info->is_old = false;
        meta_info->load_time = boost::posix_time::microsec_clock::local_time();
        LOG_INFO("load {}, point size {} \n", ss.str().c_str(), meta_info->map_pcd->size());
        update = true;
        new_load_cnt++;
    }
    LOG_INFO("load new meta cnt {}", new_load_cnt);
    if (new_load_cnt == 0) {
        return false;
    }

    auto cur_tms = boost::posix_time::microsec_clock::local_time();
    // 将过期的图元进行移除
    for (auto& meta_info : ids_metamap_map_[curr_map_.first]) {
        // 忽略未加载的
        if (!meta_info->is_active) continue;

        if (dis_a_b(meta_info->x, meta_info->y, pos[0], pos[1]) > 50) {
            // 超过距离的设置为old
            if (!meta_info->is_old) {
                // 如果初次为old则设置时间
                meta_info->expired_time = boost::posix_time::microsec_clock::local_time();
                meta_info->is_old = true;
            }
            // 如果过期时间超过15s则移除
            if (meta_info->is_old && (cur_tms - meta_info->expired_time).total_milliseconds() > 15 * 1000) {
                LOG_INFO("remove grid [{}] \n", meta_info->name.c_str());
                meta_info->is_active = false;
                meta_info->map_pcd.reset(new PointCloudType);
                update = true;
            }
        }
    }

    // 如果发生了更新, 则从新更新地图
    if (update) {
        PointCloudPtr trans_map(new PointCloudType);
        for (auto& meta_info : ids_metamap_map_[curr_map_.first]) {
            if (!meta_info->is_active) continue;
            if (meta_info->map_pcd->empty()) continue;

            LOG_INFO(YELLOW "insert [{}] of point size {} \n", meta_info->name.c_str(), meta_info->map_pcd->size());
            *trans_map += *TransformLidar(meta_info->map_pcd, meta_info->T.R, meta_info->T.t);
            global_map_filter_.setInputCloud(trans_map);
            global_map_filter_.filter(*global_map_);
        }
        if (global_map_->empty()) {
            return false;
        }
        LOG_INFO(YELLOW "global map point size {} \n" RESET, global_map_->size());
        std::lock_guard<std::mutex> lock(state_mutex_);
        local_state_ = LOCAL_STATE::NOT_INIT;
    }
    return true;
}
}  // namespace slam