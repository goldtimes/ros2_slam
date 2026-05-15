#include "system.hh"
#include "front_end.hh"
#include "localizer/localizer.hh"
#include "state.hh"
#include "system_config.hh"

namespace slam {
System::System(const std::string& config_path) : config_path_(config_path) {
    InitConfigParams();
    lidar_process_ptr_ = std::make_shared<LidarProcess>(
        system_config_ptr_->lidar_config_.lidar_type, system_config_ptr_->lidar_config_.use_livox_driver,
        system_config_ptr_->lidar_config_.lidar_min_range, system_config_ptr_->lidar_config_.lidar_max_range,
        system_config_ptr_->lidar_config_.point_filter_num, system_config_ptr_->frontend_config_.keep_angle_ranges,
        system_config_ptr_->frontend_config_.remove_ranges);
    front_end_ptr_ = new FrontEnd(this);
    T_IL = system_config_ptr_->lidar2imu_;
    T_BL = system_config_ptr_->lidar2robot_;
    auto T_LI = T_IL.inverse();
    // T_BI = T_BL * T_LI;
    T_BI = (T_BL * T_LI);
    // 开启前端的线程
    front_end_thread_ptr_ = new std::thread(&FrontEnd::Run, front_end_ptr_);
    system_init_.store(false);

    // 定位程序
    localizer_ptr_ = std::make_shared<Localizer>(system_config_ptr_);
}

void System::InitConfigParams() {
    LOG_INFO("config_path: {}", config_path_);
    system_config_ptr_ = std::make_shared<SystemConfig>();
    if (!system_config_ptr_->LoadAndPrintConfig(config_path_)) {
        LOG_ERROR("Load config file failed, Program will exit!");
        std::exit(1);
    }
    LOG_INFO("System init done!");
}

void System::AddIMU(const IMU& imu) {
    std::lock_guard<std::mutex> lock(m_buf_mutex_);
    imu_queue_.push_back(imu);
    // 保持3s内的imu数据
    while (!imu_queue_.empty()) {
        if (imu_queue_.back().timestamp_ - imu_queue_.front().timestamp_ > 1.0) {
            imu_queue_.pop_front();
        } else {
            break;
        }
    }
    // LOG_INFO("imu size: {}", imu_queue_.size());
    m_buff_cv_.notify_one();
}
void System::AddEncoder(const Encoder& encoder) {
    std::lock_guard<std::mutex> lock(m_buf_mutex_);
    encoder_queue_.push_back(encoder);
    // 保持1s内的encoder数据
    while (!encoder_queue_.empty()) {
        if (encoder_queue_.back().timestamp_ - encoder_queue_.front().timestamp_ > 1.0) {
            encoder_queue_.pop_front();
        } else {
            break;
        }
    }
}
void System::AddLidar(const PointCloudPtr& cloud, const double cloud_time) {
    std::lock_guard<std::mutex> lock(m_buf_mutex_);
    lidar_queue_.push_back(cloud);
    lidar_time_queue_.push_back(cloud_time);
    // 保持1s内的lidar数据
    while (!lidar_time_queue_.empty()) {
        if (lidar_time_queue_.back() - lidar_time_queue_.front() > 1.0) {
            lidar_queue_.pop_front();
            lidar_time_queue_.pop_front();
        } else {
            break;
        }
    }
    // m_buff_cv_.notify_one();
}
void System::AddGNSS(const GNSS& gnss) {
    std::lock_guard<std::mutex> lock(m_buf_mutex_);
    gnss_queue_.push_back(gnss);
    // 保持1s内的gnss数据
    while (!gnss_queue_.empty()) {
        if (gnss_queue_.back().timestamp_ - gnss_queue_.front().timestamp_ > 1.0) {
            gnss_queue_.pop_front();
        } else {
            break;
        }
    }
}

void System::SetInitPose(const PoseTrans& init_pose, int level, const std::string& map_id) {
    localizer_ptr_->SetInitPose(init_pose, level, map_id);
}
void System::SetMetaInfo(const std::map<std::string, std::vector<std::shared_ptr<MetaInfo>>>& meta_maps) {
    localizer_ptr_->SetMetaMaps(meta_maps);
}

const double System::GetSystemTime() const {
    return front_end_ptr_->GetCurentTime();
}

const std::shared_ptr<Localizer> System::GetLocalizer() {
    return localizer_ptr_;
}

const PoseTrans System::GetTLidarToImu() const {
    return T_IL;
}

const PoseTrans System::GetLidarToBaselink() const {
    return T_BL;
}

const PoseTrans System::GetImuToBaselink() const {
    return T_BI;
}

const State System::GetCurentNavState() const {
    return front_end_ptr_->GetCurentNavState();
}

const M33D System::GetCov() const {
    return front_end_ptr_->GetCov();
}

// lidar坐标系原始数据
const PointCloudXYZIPtr System::GetCloudInLidarLink() const {
    return front_end_ptr_->GetCloudInLidarLink();
}

// robot_link坐标系点云
const PointCloudXYZIPtr System::GetCloudInRobotLink() const {
    return front_end_ptr_->GetCloudInRobotLink();
}

// odom坐标系点云
const PointCloudXYZIPtr System::GetCloudInOdomLink() const {
    return front_end_ptr_->GetCloudInOdomLink();
}

const PoseTrans System::GetTransformEncodeToWorld() const {
    return front_end_ptr_->GetTransformEncodeToWorld();
}

const M3D System::GetGnssHeading() const {
    return front_end_ptr_->GetGnssHeading();
}

const PointCloudXYZIPtr System::GetSubmap() const {
    return front_end_ptr_->GetSubmap();
}

// 重置系统
void System::reset() {
    std::lock_guard<std::mutex> lock(m_buf_mutex_);
    imu_queue_.clear();
    encoder_queue_.clear();
    lidar_queue_.clear();
    lidar_time_queue_.clear();
    gnss_queue_.clear();
}

System::~System() {
    if (front_end_thread_ptr_->joinable()) {
        front_end_thread_ptr_->join();
    }
    if (front_end_ptr_ != nullptr) {
        delete front_end_ptr_;
        front_end_ptr_ = nullptr;
    }
}
}  // namespace slam