/*
 * @Author: lihang lihang@kilox.cn
 * @Date: 2025-09-08 13:41:48
 * @LastEditors: lihang lihang@kilox.cn
 * @LastEditTime: 2025-09-12 16:20:40
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
    curr_lidar_cloud_.reset(new PointCloudXYZI());
    curr_submap_cloud_.reset(new PointCloudXYZI());
    global_map_.reset(new PointCloudXYZI());
    global_map_tree_.reset(new PointTree());

    gicp_matcher_.reset(new GICP());
    gicp_matcher_->setMaximumIterations(100);
    gicp_matcher_->setTransformationEpsilon(1e-6);
    gicp_matcher_->setEuclideanFitnessEpsilon(1e-6);
    gicp_matcher_->setRANSACIterations(1);
    // new_lidar_map.init_icp->setSearchMethodTarget(new_lidar_map.map_tree, true);
    // new_lidar_map.init_icp->setInputTarget(new_lidar_map.global_map);
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
    LOG_INFO("receive meta maps");
    ids_metamap_map_ = maps;
    std::lock_guard<std::mutex> lock(state_mutex_);
    local_state_ = LOCAL_STATE::NOT_INIT;
    LOG_INFO("local_state_:{}", local_state_);
    map_loaded_ = true;
}

void Localizer::SetLidarCloud(const PointCloudXYZIPtr& lidar_cloud, const PoseTrans& T_RtoO) {
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    curr_lidar_cloud_ = lidar_cloud;
    get_new_lidar_ = true;
    T_RtoO_ = T_RtoO;
    if (local_state_ == LOCAL_STATE::INITED) {
        update_map_ = true;
        update_T_RtoM_ = T_OtoM_ * T_RtoO_;
    }
}

void Localizer::SetSubmapCloud(const PointCloudXYZIPtr& submap_cloud, const PoseTrans& T_LtoO) {
    curr_submap_cloud_ = submap_cloud;
    get_new_submap_ = true;
    update_T_RtoO_ = T_LtoO;
}

void Localizer::SetInitPose(const PoseTrans& init_RtoM, int level, const std::string& map_id) {
    if (ids_metamap_map_.empty()) {
        LOG_ERROR(YELLOW "The map has not been loaded yet");
        return;
    }
    get_init_pose_ = true;
    // 设置机器人的初始位置
    init_T_RtoM_ = init_RtoM;
    // 通知地图更新线程更新地图
    update_map_ = true;
    // 记录当前的地图信息
    curr_meta_info_.level = level;
    curr_map_ = std::make_pair(map_id, curr_meta_info_);
    LOG_INFO("map_identity:{}", map_id);
    LOG_INFO("init posisition:{}", init_T_RtoM_.t.transpose());
    LOG_INFO("init orientation:{}", init_T_RtoM_.RPY().transpose());
    // 修改初始值的高度
    if (traj_cloud_loaded_) {
        PointXYZI init_pt;
        init_pt.x = init_T_RtoM_.t(0);
        init_pt.y = init_T_RtoM_.t(1);
        init_pt.z = init_T_RtoM_.t(2);
        // 查找最近的点
        std::vector<int> indices;
        std::vector<float> distances;
        traj_cloud_tree_->nearestKSearch(init_pt, 1, indices, distances);
        if (indices.size() > 0) {
            init_T_RtoM_.t(2) = traj_cloud_->points[indices[0]].z;
        }
        if (init_T_RtoM_.t(2) < 0) {
            init_T_RtoM_.t(2) = 0;
        }
        if (init_T_RtoM_.t(2) > 2.0) {
            init_T_RtoM_.t(2) /= 2;
        }
        LOG_INFO("change predict height to {}", init_T_RtoM_.t.transpose());
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

    std::lock_guard<std::mutex> lock(state_mutex_);
    local_state_ = LOCAL_STATE::NOT_INIT;
    map_loaded_ = true;
}

void Localizer::SetTrajCloud(const PointCloudXYZIPtr& traj_cloud) {
    // 分配空间
    traj_cloud_.reset(new PointCloudXYZI());
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
            LoadMapByPose(update_T_RtoM_);
        }
        rate.sleep();
    }
    LOG_INFO("[LOC] MapUpdate Stop");
}
void Localizer::MapRegister() noexcept {
    LOG_INFO("MapRegister Run");
    // 等待局部地图更新了
    ros::Rate rate(50);
    while (ros::ok()) {
        // 1. 先获取当前状态（仅在获取状态时加锁）
        LOCAL_STATE current_state;
        bool has_init_pose;
        bool has_new_lidar;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state = local_state_;
            has_init_pose = get_init_pose_;
            has_new_lidar = get_new_lidar_;
        }  // 锁在此处释放，避免后续操作持有锁
        switch (current_state) {
            // 需要在外面设置 NOT_INIT状态
            case LOCAL_STATE::NOT_INIT:
                LOG_INFO("[LOC] not init , get_init_pose_:{}, get_new_lidar:{}", get_init_pose_, get_new_lidar_);
                if (has_init_pose && has_new_lidar) {
                    // 3. 仅在修改共享变量时加锁
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        get_init_pose_ = false;
                        get_new_lidar_ = false;
                    }
                    // 启动初始化（耗时操作，无锁）
                    StartInitialization();
                }
                break;
            case LOCAL_STATE::INITING:  // cast 不会创建作用域
                CheckInitializationStatus();
                break;
            case LOCAL_STATE::INITED:
                UpdateSearch();
                break;
            case LOCAL_STATE::INIT_FAILED:
                break;
            case LOCAL_STATE::LOST:
                break;
            case LOCAL_STATE::RELOCALIZING:
                break;
            case LOCAL_STATE::MAP_NOT_LOAD:
                // LOG_INFO("[LOC] map not load");
                break;
        }
        rate.sleep();
    }
    LOG_INFO("MapRegister Stop");
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
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        local_state_ = LOCAL_STATE::INITING;
    }
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
    int num_trans = system_config_ptr_->localizer_config_.num_trans;
    int num_rot = system_config_ptr_->localizer_config_.num_rot;
    double delta_trans = system_config_ptr_->localizer_config_.delta_trans;
    double delta_rot = system_config_ptr_->localizer_config_.delta_rot;
    std::vector<PoseTrans> search_poses =
        GeneratorSearchGrids(init_T_RtoM_, num_trans, num_rot, delta_trans, delta_rot);
    double min_score = 1e4;
    PoseTrans best_guess_pose;
    PointCloudXYZIPtr curr_lidar_cloud;
    {
        // 这里放到了两个线程中了，如果不加锁，curr_lidar_cloud会被修改，并且需要进行深拷贝
        std::lock_guard<std::mutex> lock(lidar_mutex_);
        curr_lidar_cloud.reset(new PointCloudXYZI(*curr_lidar_cloud_));
    }
    for (size_t i = 0; i < search_poses.size(); i++) {
        if (cancel_init_) {
            break;
        }
        double score = CalculateP2PScore(search_poses[i], curr_lidar_cloud, global_map_tree_,
                                         system_config_ptr_->localizer_config_.icp_dist_thresh);
        if (score < min_score) {
            min_score = score;
            best_guess_pose = search_poses[i];
        }
    }
    LOG_INFO("After Search, min_score:{}", min_score);
    LOG_INFO("best_guess_pose Position:{}, Rotation:{}", best_guess_pose.t.transpose(),
             best_guess_pose.RPY().transpose());

    // 更新num_trans以及其他参数
    delta_rot = 0.1;
    delta_trans = 0.1;
    search_poses.clear();
    search_poses = GeneratorSearchGrids(best_guess_pose, num_trans, num_rot, delta_trans, delta_rot);
    LOG_INFO("After Search, search_poses size:{}", search_poses.size());
    int fail_cnt = 0;
    double init_icp_score = system_config_ptr_->localizer_config_.init_icp_score;
    for (size_t i = 0; i < search_poses.size(); i++) {
        if (cancel_init_) {
            break;
        }
        PointCloudXYZIPtr trans_source_cloud = TransformLidar(curr_lidar_cloud, search_poses[i].R, search_poses[i].t);

        PoseTrans incre_pose;
        double score;
        score = GicpAlign(trans_source_cloud, global_map_, global_map_tree_, incre_pose,
                          system_config_ptr_->localizer_config_.update_search_dist_thresh,
                          system_config_ptr_->localizer_config_.match_score_thresh);

        LOG_INFO("try times {}, score:{}", i, score);
        if (score > 0 && score < init_icp_score) {
            LOG_INFO("update score");
            init_icp_score = score;
            // 更新配准后机器位姿
            init_Result_ = incre_pose * search_poses[i];
            success = true;

            if (score < system_config_ptr_->localizer_config_.init_icp_score / 2) {
                success = true;
                init_Result_ = search_poses[i];
                break;
            }
        } else if (score > init_icp_score) {
            fail_cnt++;
        }
        if (fail_cnt > search_poses.size() / 2) {
            LOG_INFO("try many times, can't get a good match score");
            return false;
        }
    }
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
        pcl::io::loadPCDFile<PointXYZI>(ss.str(), *meta_info->map_pcd);
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
                meta_info->map_pcd.reset(new PointCloudXYZI);
                update = true;
            }
        }
    }

    // 如果发生了更新, 则从新更新地图
    if (update) {
        PointCloudXYZIPtr trans_map(new PointCloudXYZI);
        for (auto& meta_info : ids_metamap_map_[curr_map_.first]) {
            if (!meta_info->is_active) continue;
            if (meta_info->map_pcd->empty()) continue;

            LOG_INFO(YELLOW "insert [{}] of point size {} \n", meta_info->name.c_str(), meta_info->map_pcd->size());
            *trans_map += *TransformLidar(meta_info->map_pcd, meta_info->T.R, meta_info->T.t);
            global_map_filter_.setInputCloud(trans_map);
            global_map_filter_.filter(*global_map_);
            global_map_update_ = true;
            global_map_tree_->setInputCloud(global_map_);
        }
        if (global_map_->empty()) {
            return false;
        }
        LOG_INFO(YELLOW "global map point size {} \n" RESET, global_map_->size());
    } else {
        global_map_update_ = false;
    }

    return true;
}

std::vector<PoseTrans> Localizer::GeneratorSearchGrids(const PoseTrans& init_pose, int num_trans, int num_rot,
                                                       double delta_trans, double delta_rot) {
    // 假设平移步长为0.1,旋转步长为0.1
    // num_trans_x[-3,3],num_trans_y[-3,3],num_rot:[-3,3] -3,-2,-1,0,1,2,3,搜索的次数比较多了
    std::vector<PoseTrans> search_poses;
    int total_try = (num_trans * 2 + 1) * (num_trans * 2 + 1) * (num_rot * 2 + 1);
    LOG_INFO("[Search Near Pose] min_trans:{:03.3f}, max_trans:{:03.3f}", -num_trans * delta_trans,
             num_trans * delta_trans);
    LOG_INFO("[Search Near Pose] min_yaw:{:03.3f},max_yaw:{:03.3f}", num_rot * delta_rot, num_rot * delta_rot);
    for (int dx = -num_trans; dx <= num_trans; ++dx) {
        for (int dy = -num_trans; dy <= num_trans; ++dy) {
            Eigen::Vector3d trans_offset = Eigen::Vector3d(dx * delta_trans, dy * delta_trans, 0.0);
            // 只在yaw角做搜索
            for (int dr = -num_rot; dr <= num_rot; ++dr) {
                Eigen::Matrix3d rot_offset =
                    Eigen::AngleAxisd(dr * delta_rot, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                PoseTrans pose_offset(rot_offset, trans_offset);
                PoseTrans search_pose = init_pose * pose_offset;
                search_poses.push_back(search_pose);
            }
        }
    }
    return search_poses;
}

double Localizer::CalculateP2PScore(const PoseTrans& pose, const PointCloudXYZIPtr& input_cloud,
                                    const PointTree::Ptr& targer_tree, double dist_thresh) {
    if (input_cloud->points.size() < 100) {
        LOG_ERROR("input cloud size is too small < 100");
        return -1;
    }
    // LOG_INFO("input cloud size {}", input_cloud->size());
    PointCloudXYZIPtr trans_cloud(new PointCloudXYZI);
    TransformCloud(input_cloud, trans_cloud, pose.R, pose.t);

    std::vector<int> nn_indices;
    std::vector<float> nn_dists;
    int effect_num = 0;
    double fitness_score = 0;
    for (size_t i = 0; i < trans_cloud->points.size(); ++i) {
        if (targer_tree->nearestKSearch(trans_cloud->points[i], 1, nn_indices, nn_dists) > 0) {
            if (!nn_indices.empty() && nn_dists[0] <= dist_thresh) {
                fitness_score += nn_dists[0];
                effect_num++;
            }
        }
    }
    if (effect_num > 0) {
        double score = fitness_score / effect_num + (1 - 1.0 * effect_num / trans_cloud->size()) * 0.5;
        LOG_INFO("match_points:{}, dist_sum:{}, p2p_score:{}", effect_num, fitness_score, score);
        return score;
    }
    return -1;
}

void Localizer::UpdateSearch() {
    PointCloudXYZIPtr cloud_in_map = TransformLidar(curr_submap_cloud_, T_OtoM_.R, T_OtoM_.t);
    PoseTrans incre_pose;
    double score = GicpAlign(cloud_in_map, global_map_, global_map_tree_, incre_pose,
                             system_config_ptr_->localizer_config_.update_search_dist_thresh,
                             system_config_ptr_->localizer_config_.match_score_thresh);
    LOG_INFO(BLUE
             "=======> update source {} icp score {:03.3f} current in match icp "
             "init, incre_trans {:03.3f} " RESET,
             cloud_in_map->size(), score, incre_pose.norm());
    PoseTrans best_OtoM = incre_pose * T_OtoM_;
    if (score > system_config_ptr_->localizer_config_.match_score_thresh) {
        LOG_INFO("updated failed:{}", score);
        std::lock_guard<std::mutex> lock(state_mutex_);
        local_state_ = LOCAL_STATE::LOST;
    }
}

double Localizer::GicpAlign(const PointCloudXYZIPtr& trans_cloud, const PointCloudXYZIPtr& target_cloud,
                            const PointTree::Ptr& target_tree, PoseTrans& incre_pose, double update_dist_thresh,
                            double match_score_thresh) {
    double score = -1;
    if (trans_cloud->points.size() < 100) {
        LOG_ERROR("input cloud size is too small < 100");
        return -1;
    }
    if (target_cloud->points.size() < 100) {
        LOG_ERROR("target cloud size is too small < 100");
        return -1;
    }
    PointCloudXYZIPtr aligned_cloud(new PointCloudXYZI);
    gicp_matcher_->setSearchMethodTarget(target_tree, true);
    gicp_matcher_->setInputTarget(target_cloud);
    gicp_matcher_->setInputSource(trans_cloud);
    gicp_matcher_->align(*aligned_cloud);
    Eigen::Affine3f transform;
    transform = gicp_matcher_->getFinalTransformation();
    PoseTrans final_tf(transform.linear().cast<double>(), transform.translation().cast<double>());
    score = CalculateP2PScore(final_tf, trans_cloud, target_tree, update_dist_thresh);
    LOG_INFO("optimize ----> icp match source {} target {} score {} \n", trans_cloud->size(), target_cloud->size(),
             score);
    if (gicp_matcher_->hasConverged() && score > 0 && score <= match_score_thresh) {
        incre_pose = final_tf;
    }
    return score;
}
}  // namespace slam