#pragma once
#include "common/commons.hh"
#include "common/eigen_type.hh"
#include "common/logger.hh"
// #include "loop_closure/loop_closure_detector.hh"
#include <yaml-cpp/yaml.h>
#include <string>
namespace slam {

struct LidarConfig {
    std::string lidar_topic;
    int use_livox_driver;
    int point_filter_num;
    double lidar_min_range;
    double lidar_max_range;
    std::string lidar_type;
    bool is_tms_head;
    double lidar_noise_std;
    int use_multi_lidar;
    std::string lidar_left_topic;
    std::string lidar_right_topic;
    // 重载print函数
    void print() const {
        LOG_INFO(BLUE "LidarConfig:" RESET);
        LOG_INFO("  lidar_topic: {}", lidar_topic);
        LOG_INFO("  is_tms_head: {}", is_tms_head);
        LOG_INFO("  use_livox_driver: {}", use_livox_driver);
        LOG_INFO("  point_filter_num: {}", point_filter_num);
        LOG_INFO("  lidar_min_range: {:03.3f}", lidar_min_range);
        LOG_INFO("  lidar_max_range: {:03.3f}", lidar_max_range);
        LOG_INFO("  lidar_type: {}", lidar_type);
        LOG_INFO("  lidar_noise_std: {:03.3f}", lidar_noise_std);
        LOG_INFO("  use_multi_lidar: {}", use_multi_lidar);
        if (use_multi_lidar > 1) {
            LOG_INFO("  lidar_left_topic: {}", lidar_left_topic);
            LOG_INFO("  lidar_right_topic: {}", lidar_right_topic);
        }
    }
};
struct IMUConfig {
    std::string imu_topic;
    double imu_scale;
    double acc_noise_std;
    double acc_bias_noise_std;
    double gyro_noise_std;
    double gyro_bias_noise_std;
    // 重载print函数
    void print() const {
        LOG_INFO(BLUE "IMUConfig:" RESET);
        LOG_INFO("  imu_topic: {}", imu_topic);
        LOG_INFO("  imu_scale: {:03.3f}", imu_scale);
        LOG_INFO("  acc_noise_std: {:03.3f}", acc_noise_std);
        LOG_INFO("  acc_bias_noise_std: {:03.3f}", acc_bias_noise_std);
        LOG_INFO("  gyro_noise_std: {:03.3f}", gyro_noise_std);
        LOG_INFO("  gyro_bias_noise_std: {:03.3f}", gyro_bias_noise_std);
    }
};
struct EncoderConfig {
    std::string encoder_topic;
    double encoder_position_noise_std;
    double encoder_rotation_noise_std;
    double wheel_cov;
    double nhc_y;
    double nhc_z;
    // 重载print函数
    void print() const {
        LOG_INFO(BLUE "EncoderConfig:" RESET);
        LOG_INFO("  encoder_topic: {}", encoder_topic);
        LOG_INFO("  encoder_position_noise_std: {:03.3f}", encoder_position_noise_std);
        LOG_INFO("  encoder_rotation_noise_std: {:03.3f}", encoder_rotation_noise_std);
        LOG_INFO("  wheel_cov: {:03.3f}", wheel_cov);
        LOG_INFO("  nhc_y: {:03.3f}", nhc_y);
        LOG_INFO("  nhc_z: {:03.3f}", nhc_z);
    }
};
struct GNSSConfig {
    std::string gnss_topic;
    double gnss_position_noise_std;
    double gnss_rotation_noise_std;
    bool has_orientation;
    // 重载print函数
    void print() const {
        LOG_INFO(BLUE "GNSSConfig:" RESET);
        LOG_INFO("  gnss_topic: {}", gnss_topic);
        LOG_INFO("  gnss_position_noise_std: {:03.3f}", gnss_position_noise_std);
        LOG_INFO("  gnss_rotation_noise_std: {:03.3f}", gnss_rotation_noise_std);
        LOG_INFO("  has_orientation: {}", has_orientation);
    }
};

struct P2PlaneConfig {
    int keyframe_num;
    double p2plane_thresh;
    double keyframe_distance;
    double keyframe_angle_distance;
    bool use_angle_keyframe;
    double downsample;
    double map_resolution;
    int cube_len;
    int det_range;
    double move_thresh;
    // 重载print函数
    void print() const {
        LOG_INFO(BLUE "P2PlaneConfig:" RESET);
        LOG_INFO("  keyframe_num: {}", keyframe_num);
        LOG_INFO("  p2plane_thresh: {:03.3f}", p2plane_thresh);
        LOG_INFO("  keyframe_distance: {:03.3f}", keyframe_distance);
        LOG_INFO("  keyframe_angle_distance: {:03.3f}", keyframe_angle_distance);
        LOG_INFO("  use_angle_keyframe: {}", use_angle_keyframe);
        LOG_INFO("  downsample: {:03.3f}", downsample);
        LOG_INFO("  map_resolution: {:03.3f}", map_resolution);
        LOG_INFO("  cube_len: {}", cube_len);
        LOG_INFO("  det_range: {}", det_range);
        LOG_INFO("  move_thresh: {:03.3f}", move_thresh);
    }
};
struct VoxelConfig {
    double voxle_size;
    int max_layer;
    std::vector<int> layer_point_size;
    double plannar_threshold;
    int max_points_size;
    int max_cov_points_size;
    bool update_omp;
    bool pub_voxel_map;
    int pub_max_voxel_layer;
    bool pub_point_cloud;
    bool dense_map_enable;
    int pub_point_cloud_skip;
    double ranging_cov;
    double angle_cov;
    int max_capacity;
    int sigma_num;
    bool updatemap_omp;
    // 重载print函数
    void print() const {
        LOG_INFO(BLUE "VoxelConfig:" RESET);
        LOG_INFO("  voxle_size: {:03.3f}", voxle_size);
        LOG_INFO("  max_layer: {}", max_layer);
        // LOG_INFO("  layer_point_size: [{},{}]", layer_point_size[0],
        // layer_point_size[1], layer_point_size[2],
        //          layer_point_size[3], layer_point_size[4]);
        LOG_INFO("  plannar_threshold: {:03.3f}", plannar_threshold);
        LOG_INFO("  max_points_size: {}", max_points_size);
        LOG_INFO("  max_cov_points_size: {}", max_cov_points_size);
        LOG_INFO("  max_capacity: {}", max_capacity);
        LOG_INFO("  update_omp: {}", update_omp);
        LOG_INFO("  pub_voxel_map: {}", pub_voxel_map);
        LOG_INFO("  pub_max_voxel_layer: {}", pub_max_voxel_layer);
        LOG_INFO("  pub_point_cloud: {}", pub_point_cloud);
        LOG_INFO("  dense_map_enable: {}", dense_map_enable);
        LOG_INFO("  pub_point_cloud_skip: {}", pub_point_cloud_skip);
        LOG_INFO("  ranging_cov: {:03.3f}", ranging_cov);
        LOG_INFO("  angle_cov: {:03.3f}", angle_cov);
        LOG_INFO("  sigma_num: {}", sigma_num);
        LOG_INFO("  updatemap_omp: {}", updatemap_omp);
    }
};
struct NDTConfig {
    double voxel_size;
    bool near_search;
    int max_capacity;
    int min_effective_pts;
    int min_pts_in_voxel;
    int max_pts_in_voxel;
    double res_outlier_thresh;
    double eps;
    // 重载print函数
    void print() const {
        LOG_INFO(BLUE "NDTConfig:" RESET);
        LOG_INFO("  voxle_size: {:03.3f}", voxel_size);
        LOG_INFO("  near_search: {}", near_search);
        LOG_INFO("  max_capacity: {}", max_capacity);
        LOG_INFO("  min_effective_pts: {}", min_effective_pts);
        LOG_INFO("  min_pts_in_voxel: {}", min_pts_in_voxel);
        LOG_INFO("  max_pts_in_voxel: {}", max_pts_in_voxel);
        LOG_INFO("  res_outlier_thresh: {:03.3f}", res_outlier_thresh);
        LOG_INFO("  eps: {:03.3f}", eps);
    }
};

struct FasterlioConfig {
    double ivox_grid_resolution;
    double planner_threshold;
    int ivox_nearby_type;  // 6, 18, 26
    // 重载print函数
    void print() const {
        LOG_INFO(BLUE "FasterlioConfig:" RESET);
        LOG_INFO("  ivox_grid_resolution: {:03.3f}", ivox_grid_resolution);
        LOG_INFO("  planner_threshold: {:03.3f}", planner_threshold);
        LOG_INFO("  ivox_nearby_type: {}", ivox_nearby_type);
    }
};

struct FrontendConfig {
    std::vector<double> keep_angle_ranges;
    std::vector<double> remove_ranges;
    bool calib_lidar2imu;
    int max_iteration;
    // 关键帧参数
    int keyframe_size;
    double keyframe_distance;
    double keyframe_angle_distance;
    bool use_angle_keyframe;
    bool save_map;
    int pcd_save_interval;
    std::string map_dir;
    // 回环检测参数
    //   LoopClosureConfig loop_closure_config;
    void print() const {
        LOG_INFO(BLUE "FrontendConfig:" RESET);
        LOG_INFO("  calib_lidar2imu: {}", calib_lidar2imu);
        LOG_INFO("  max_iteration: {}", max_iteration);
        LOG_INFO("  keyframe_size: {}", keyframe_size);
        LOG_INFO("  keyframe_distance: {:03.3f}", keyframe_distance);
        LOG_INFO("  keyframe_angle_distance: {:03.3f}", keyframe_angle_distance);
        LOG_INFO("  use_angle_keyframe: {}", use_angle_keyframe);
        LOG_INFO("  save_map: {}", save_map);
        LOG_INFO("  pcd_save_interval: {}", pcd_save_interval);
        LOG_INFO("  map_dir: {}", map_dir);
        // loop_closure_config.print();
    }
    P2PlaneConfig p2plane_config;
    VoxelConfig voxel_config;
    NDTConfig ndt_config;
    FasterlioConfig fasterlio_config;
};

struct LocalizerConfig {
    bool use_meta_maps;
    double global_map_filter_size;
    std::string local_map_dir;
    std::string map_identity;  // 默认叶子地图 identity(图元分组), 可空
    std::string register_method;
    double point_to_plane;
    double dist_to_robot;
    int num_trans;
    int num_rot;
    double delta_trans;
    double delta_rot;
    double init_icp_score;
    double icp_dist_thresh;
    double update_search_dist_thresh;
    double match_score_thresh;
    bool use_ceres;

    void print() const {
        LOG_INFO(BLUE "LocalizerConfig:" RESET);
        LOG_INFO("  use_meta_maps: {}", use_meta_maps);
        LOG_INFO("  global_map_filter_size: {:03.3f}", global_map_filter_size);
        LOG_INFO("  local_map_dir: {}", local_map_dir);
        LOG_INFO("  map_identity: {}", map_identity);
        LOG_INFO("  register_method: {}", register_method);
        LOG_INFO("  point_to_plane: {:03.3f}", point_to_plane);
        LOG_INFO("  dist_to_robot: {:03.3f}", dist_to_robot);
        LOG_INFO("  num_trans: {}", num_trans);
        LOG_INFO("  num_rot: {}", num_rot);
        LOG_INFO("  delta_trans: {:03.3f}", delta_trans);
        LOG_INFO("  delta_rot: {:03.3f}", delta_rot);
        LOG_INFO("  init_icp_score: {:03.3f}", init_icp_score);
        LOG_INFO("  icp_dist_thresh: {:03.3f}", icp_dist_thresh);
        LOG_INFO("  update_search_dist_thresh: {:03.3f}", update_search_dist_thresh);
        LOG_INFO("  match_score_thresh: {:03.3f}", match_score_thresh);
        LOG_INFO("  use_ceres: {}", use_ceres);
    }
};

class SystemConfig {
   public:
    SystemConfig() = default;
    ~SystemConfig() = default;

    bool LoadAndPrintConfig(const std::string &config_path);

    PoseTrans LoadTransformAndPrint(const YAML::Node &node, const std::string &name);

   public:
    LidarConfig lidar_config_;
    IMUConfig imu_config_;
    EncoderConfig encoder_config_;
    GNSSConfig gnss_config_;
    FrontendConfig frontend_config_;
    LocalizerConfig localizer_config_;

    double GRAVIRT_;

    bool use_p2plane_;
    bool use_voxel_;
    bool use_ndt_;
    bool use_fasterlio_;
    bool use_superlio_;

    bool has_encoder_;
    bool has_gnss_;
    bool has_camera_;

    PoseTrans lidar2imu_;
    PoseTrans imu2encoder_;
    PoseTrans lidar2robot_;
    PoseTrans Rlidar2imu_;
    PoseTrans Llidar2imu_;
    PoseTrans gnss2imu_;
};
}  // namespace slam