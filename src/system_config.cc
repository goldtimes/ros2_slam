#include "system_config.hh"
#include "logger.hh"

namespace slam {
bool SystemConfig::LoadAndPrintConfig(const std::string& config_path) {
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        // 加载雷达相关的配置
        lidar_config_.lidar_topic = config["lidar"]["lidar_topic"].as<std::string>();
        lidar_config_.lidar_type = config["lidar"]["lidar_type"].as<std::string>();
        lidar_config_.is_tms_head = config["lidar"]["is_tms_head"].as<bool>();
        lidar_config_.use_livox_driver = config["lidar"]["use_livox_driver"].as<int>();
        lidar_config_.point_filter_num = config["lidar"]["point_filter_num"].as<int>();
        lidar_config_.lidar_min_range = config["lidar"]["lidar_min_range"].as<double>();
        lidar_config_.lidar_max_range = config["lidar"]["lidar_max_range"].as<double>();
        lidar_config_.lidar_position_noise_std = config["lidar"]["lidar_position_noise_std"].as<double>();
        lidar_config_.lidar_rotation_noise_std = config["lidar"]["lidar_rotation_noise_std"].as<double>();
        lidar_config_.lidar_noise_std = config["lidar"]["lidar_noise_std"].as<double>();
        lidar_config_.print();
        // 加载IMU相关的配置
        imu_config_.imu_topic = config["imu"]["imu_topic"].as<std::string>();
        imu_config_.imu_scale = config["imu"]["imu_scale"].as<double>();
        imu_config_.acc_noise_std = config["imu"]["acc_noise_std"].as<double>();
        imu_config_.gyro_noise_std = config["imu"]["gyro_noise_std"].as<double>();
        imu_config_.acc_bias_noise_std = config["imu"]["acc_bias_noise_std"].as<double>();
        imu_config_.gyro_bias_noise_std = config["imu"]["gyro_bias_noise_std"].as<double>();
        imu_config_.print();
        // 加载编码器相关的配置
        encoder_config_.encoder_topic = config["encoder"]["encoder_topic"].as<std::string>();
        encoder_config_.encoder_position_noise_std = config["encoder"]["encoder_position_noise_std"].as<double>();
        encoder_config_.encoder_rotation_noise_std = config["encoder"]["encoder_rotation_noise_std"].as<double>();
        encoder_config_.print();

        // 加载GNSS相关的配置
        gnss_config_.gnss_topic = config["gnss"]["gnss_topic"].as<std::string>();
        gnss_config_.gnss_position_noise_std = config["gnss"]["gnss_position_noise_std"].as<double>();
        gnss_config_.gnss_rotation_noise_std = config["gnss"]["gnss_rotation_noise_std"].as<double>();
        gnss_config_.has_orientation = config["gnss"]["has_orientation"].as<bool>();
        gnss_config_.print();

        // 通用配置
        has_camera_ = config["has_camera"].as<bool>();
        has_encoder_ = config["has_encoder"].as<bool>();
        has_gnss_ = config["has_gnss"].as<bool>();

        // 加载其他通用配置
        use_p2plane_ = config["use_p2plane"].as<bool>();
        use_voxel_ = config["use_voxel"].as<bool>();
        use_ndt_ = config["use_ndt"].as<bool>();
        GRAVIRT_ = config["gravity"].as<double>();
        // 打印通用配置
        LOG_INFO("has_camera: {}", has_camera_);
        LOG_INFO("has_encoder: {}", has_encoder_);
        LOG_INFO("has_gnss: {}", has_gnss_);
        LOG_INFO("use_p2plane: {}", use_p2plane_);
        LOG_INFO("use_voxel: {}", use_voxel_);
        LOG_INFO("use_ndt: {}", use_ndt_);
        LOG_INFO("GRAVIRT: {}", GRAVIRT_);
        // 加载外参文件
        auto lidar2robot_vec = config["T_lidar2robot"].as<std::vector<double>>();
        Eigen::Matrix4d T_lidar2robot =
            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(lidar2robot_vec.data());
        auto lidar2imu_vec = config["T_lidar2imu"].as<std::vector<double>>();
        Eigen::Matrix4d T_lidar2imu = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(lidar2imu_vec.data());
        auto imu2enc_vec = config["T_imu2encoder"].as<std::vector<double>>();
        Eigen::Matrix4d T_imu2enc = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(imu2enc_vec.data());

        // 先转换成四元素的目的是防止旋转矩阵不是正交的
        lidar2imu_ = PoseTrans(Eigen::Quaterniond(T_lidar2imu.block<3, 3>(0, 0)).toRotationMatrix(),
                               T_lidar2imu.block<3, 1>(0, 3));
        imu2encoder_ =
            PoseTrans(Eigen::Quaterniond(T_imu2enc.block<3, 3>(0, 0)).toRotationMatrix(), T_imu2enc.block<3, 1>(0, 3));
        lidar2robot_ = PoseTrans(Eigen::Quaterniond(T_lidar2robot.block<3, 3>(0, 0)).toRotationMatrix(),
                                 T_lidar2robot.block<3, 1>(0, 3));
        print_matrix(T_lidar2imu, std::string("T_lidar2imu"));
        print_matrix(T_lidar2robot, std::string("T_lidar2robot"));
        print_matrix(T_imu2enc, std::string("T_imu2enc"));

        frontend_config_.keep_angle_ranges = config["front_end"]["keep_angle_ranges"].as<std::vector<double>>();
        frontend_config_.remove_ranges = config["front_end"]["remove_ranges"].as<std::vector<double>>();
        frontend_config_.calib_lidar2imu = config["front_end"]["calib_lidar2imu"].as<bool>();
        frontend_config_.max_iteration = config["front_end"]["max_iteration"].as<int>();
        if (use_p2plane_) {
            frontend_config_.p2plane_config.p2plane_thresh =
                config["front_end"]["use_p2plane"]["p2plane_thresh"].as<double>();
            frontend_config_.p2plane_config.keyframe_num =
                config["front_end"]["use_p2plane"]["keyframe_size"].as<int>();
            frontend_config_.p2plane_config.keyframe_distance =
                config["front_end"]["use_p2plane"]["keyframe_distance"].as<double>();
            frontend_config_.p2plane_config.keyframe_angle_distance =
                config["front_end"]["use_p2plane"]["keyframe_angle_distance"].as<double>();
            frontend_config_.p2plane_config.use_angle_keyframe =
                config["front_end"]["use_p2plane"]["use_angle_keyframe"].as<bool>();
            frontend_config_.p2plane_config.downsample = config["front_end"]["use_p2plane"]["downsample"].as<double>();
            frontend_config_.p2plane_config.map_resolution =
                config["front_end"]["use_p2plane"]["map_resolution"].as<double>();
            frontend_config_.p2plane_config.cube_len = config["front_end"]["use_p2plane"]["cube_len"].as<int>();
            frontend_config_.p2plane_config.det_range = config["front_end"]["use_p2plane"]["det_range"].as<int>();
            frontend_config_.p2plane_config.move_thresh =
                config["front_end"]["use_p2plane"]["move_thresh"].as<double>();
            frontend_config_.p2plane_config.print();
        }
        if (use_voxel_) {
            frontend_config_.voxel_config.voxle_size = config["front_end"]["use_voxel_map"]["voxle_size"].as<double>();
            frontend_config_.voxel_config.max_layer = config["front_end"]["use_voxel_map"]["max_layer"].as<int>();
            frontend_config_.voxel_config.layer_point_size =
                config["front_end"]["use_voxel_map"]["layer_point_size"].as<std::vector<int>>();
            frontend_config_.voxel_config.plannar_threshold =
                config["front_end"]["use_voxel_map"]["plannar_threshold"].as<double>();
            frontend_config_.voxel_config.max_points_size =
                config["front_end"]["use_voxel_map"]["max_points_size"].as<int>();
            frontend_config_.voxel_config.max_cov_points_size =
                config["front_end"]["use_voxel_map"]["max_cov_points_size"].as<int>();
            frontend_config_.voxel_config.max_capacity = config["front_end"]["use_voxel_map"]["max_capacity"].as<int>();
            frontend_config_.voxel_config.update_omp = config["front_end"]["use_voxel_map"]["update_omp"].as<bool>();
            frontend_config_.voxel_config.pub_voxel_map =
                config["front_end"]["use_voxel_map"]["pub_voxel_map"].as<bool>();
            frontend_config_.voxel_config.pub_max_voxel_layer =
                config["front_end"]["use_voxel_map"]["pub_max_voxel_layer"].as<int>();
            frontend_config_.voxel_config.pub_point_cloud =
                config["front_end"]["use_voxel_map"]["pub_point_cloud"].as<bool>();
            frontend_config_.voxel_config.dense_map_enable =
                config["front_end"]["use_voxel_map"]["dense_map_enable"].as<bool>();
            frontend_config_.voxel_config.pub_point_cloud_skip =
                config["front_end"]["use_voxel_map"]["pub_point_cloud_skip"].as<int>();
            frontend_config_.voxel_config.ranging_cov =
                config["front_end"]["use_voxel_map"]["ranging_cov"].as<double>();
            frontend_config_.voxel_config.angle_cov = config["front_end"]["use_voxel_map"]["angle_cov"].as<double>();
            frontend_config_.voxel_config.sigma_num = config["front_end"]["use_voxel_map"]["sigma_num"].as<int>();
            // frontend_config_.voxel_config.updatemap_omp =
            //     config["front_end"]["use_voxel_map"]["updatemap_omp"].as<bool>();
            frontend_config_.voxel_config.print();
        }
        if (use_ndt_) {
            frontend_config_.ndt_config.voxel_size = config["front_end"]["use_ndt"]["voxel_size"].as<double>();
            frontend_config_.ndt_config.near_search = config["front_end"]["use_ndt"]["near_search"].as<bool>();
            frontend_config_.ndt_config.max_capacity = config["front_end"]["use_ndt"]["max_capacity"].as<int>();
            frontend_config_.ndt_config.min_effective_pts =
                config["front_end"]["use_ndt"]["min_effective_pts"].as<int>();
            frontend_config_.ndt_config.min_pts_in_voxel = config["front_end"]["use_ndt"]["min_pts_in_voxel"].as<int>();
            frontend_config_.ndt_config.max_pts_in_voxel = config["front_end"]["use_ndt"]["max_pts_in_voxel"].as<int>();
            frontend_config_.ndt_config.res_outlier_thresh =
                config["front_end"]["use_ndt"]["res_outlier_thresh"].as<double>();
            frontend_config_.ndt_config.eps = config["front_end"]["use_ndt"]["eps"].as<double>();
            frontend_config_.ndt_config.print();
        }
    } catch (const YAML::BadFile& e) {
        LOG_ERROR("config file not found: {}", config_path);
        return false;
    } catch (const YAML::Exception& e) {
        LOG_ERROR("config file parser error: {}", e.what());
        return false;
    }
    return true;
}
}  // namespace slam