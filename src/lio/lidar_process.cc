#include "lidar_process.hh"
// #include <execution> 并行处理会出问题

namespace slam {
LidarProcess::LidarProcess(const std::string &lidar_type, int use_livox_driver,
                           double min_range, double max_range,
                           int point_filter_num,
                           std::vector<double> keep_angle_ranges,
                           std::vector<double> remove_ranges)
    : lidar_type_(lidar_type), use_livox_driver_(use_livox_driver),
      min_range_(min_range), max_range_(max_range),
      point_filter_num_(point_filter_num),
      keep_angle_ranges_(keep_angle_ranges), remove_ranges_(remove_ranges) {
  LOG_INFO("Lidar Process Init: ");
  LOG_INFO("  lidar type: {}, use_livox_driver: {}, min_range: {}, max_range: "
           "{}, point_filter_num: {}",
           lidar_type_, use_livox_driver_, min_range_, max_range_,
           point_filter_num_);
  if (lidar_type_ == "mid360") {
    // LOG_INFO("lidar type mid360");
    lidar_mode_ = LIDAR_MODE::MID360;
  } else if (lidar_type_ == "avia") {
    lidar_mode_ = LIDAR_MODE::AVIA;
  } else if (lidar_type_ == "lslidar") {
    lidar_mode_ = LIDAR_MODE::LSLIDAR;
  } else if (lidar_type_ == "rs16") {
    lidar_mode_ = LIDAR_MODE::RS16;
  } else if (lidar_type_ == "airy") {
    lidar_mode_ = LIDAR_MODE::AIRY;
  } else if (lidar_type_ == "vanjee") {
    lidar_mode_ = LIDAR_MODE::VANJEE;
  } else if (lidar_type == "velodyne16") {
    // LOG_INFO("lidar type velodyne16");
    lidar_mode_ = LIDAR_MODE::VELODYNE16;
  } else if (lidar_type_ == "velodyne32") {
    lidar_mode_ = LIDAR_MODE::VELODYNE32;
  } else if (lidar_type_ == "ouster64") {
    lidar_mode_ = LIDAR_MODE::OUSTER64;
  } else {
    lidar_mode_ = LIDAR_MODE::MID360;
    LOG_INFO("lidar type {} not support, use mid360 instead", lidar_type_);
  }
  // 保存一对一对的角度
  if (!keep_angle_ranges.empty()) {
    for (int i = 0; i < keep_angle_ranges.size(); i += 2) {
      LOG_INFO("keep angle pair:{}->{}", keep_angle_ranges[i],
               keep_angle_ranges[i + 1]);
      keep_angles.emplace_back(keep_angle_ranges[i], keep_angle_ranges[i + 1]);
    }
  }
  remove_lidar_front_ = remove_ranges_[0];
  remove_lidar_back_ = remove_ranges_[1];
  remove_lidar_left_ = remove_ranges_[2];
  remove_lidar_right_ = remove_ranges_[3];
  LOG_INFO("remove lidar range:{}, {}, {}, {}", remove_ranges_[0],
           remove_ranges_[1], remove_ranges_[2], remove_ranges_[3]);
}

// 处理ros标准的雷达消息
bool LidarProcess::Process(const PointCloud2MsgConstPtr &cloud_msg,
                           PointCloudPtr &out_cloud) {
  switch (lidar_mode_) {
  case LIDAR_MODE::MID360:
    // LOG_INFO("mid360_process");
    return mid360_process(cloud_msg, out_cloud);
  case LIDAR_MODE::AVIA:
    return avia_process(cloud_msg, out_cloud);
  case LIDAR_MODE::LSLIDAR:
    return ls16_process(cloud_msg, out_cloud);
  case LIDAR_MODE::RS16:
    return rs16_process(cloud_msg, out_cloud);
  case LIDAR_MODE::AIRY:
    return airy_process(cloud_msg, out_cloud);
  case LIDAR_MODE::VANJEE:
    return vanjee_process(cloud_msg, out_cloud);
  case LIDAR_MODE::VELODYNE16:
    return velodyne16_process(cloud_msg, out_cloud);
  case LIDAR_MODE::VELODYNE32:
    return velodyne32_process(cloud_msg, out_cloud);
  case LIDAR_MODE::OUSTER64:
    return ouster64_process(cloud_msg, out_cloud);
  default:
    return false;
  }
  return false;
}

#if ROS_AVAILABLE == 1
// 处理livox driver1雷达消息(仅 ROS1)
bool LidarProcess::Process(
    const LivoxMsg1ConstPtr &cloud_msg,
    PointCloudPtr &out_cloud) {
  if (lidar_mode_ == LIDAR_MODE::MID360) {
    return mid360_process(cloud_msg, out_cloud);
  } else if (lidar_mode_ == LIDAR_MODE::AVIA) {
    return avia_process(cloud_msg, out_cloud);
  } else {
    LOG_ERROR("unsupport lidar mode {} for livox driver1", lidar_mode_);
    return false;
  }
}
#endif

// 处理livox driver2雷达消息
bool LidarProcess::Process(
    const LivoxMsg2ConstPtr &cloud_msg,
    PointCloudPtr &out_cloud) {
  if (lidar_mode_ == LIDAR_MODE::MID360) {
    return mid360_process(cloud_msg, out_cloud);
  } else if (lidar_mode_ == LIDAR_MODE::AVIA) {
    return avia_process(cloud_msg, out_cloud);
  } else {
    LOG_ERROR("unsupport lidar mode {} for livox driver2", static_cast<int>(lidar_mode_));
    return false;
  }
  return true;
}

bool LidarProcess::mid360_process(
    const PointCloud2MsgConstPtr &cloud_msg,
    PointCloudPtr &out_cloud) {
  // LOG_INFO("mid360_process");
  // 创建点云
  pcl::PointCloud<slam::LivoxMid360PointXYZITLT>::Ptr cloud(
      new pcl::PointCloud<slam::LivoxMid360PointXYZITLT>);
  // 转换点云
  pcl::fromROSMsg(*cloud_msg, *cloud);
  int points_num = cloud->points.size();
  // LOG_INFO("point size:{}", points_num);
  // 角度过滤点云，距离过滤点云，以及降采样
  PointCloudPtr filtered_cloud(new PointCloudType);
  filtered_cloud->reserve(points_num);
  int valid_num = 0;
  for (int i = 0; i < points_num; ++i) {
    valid_num++;
    if (valid_num % point_filter_num_ != 0) {
      continue;
    }
    auto livox_point = cloud->points[i];
    // 过滤nan点
    if (std::isnan(livox_point.x) || std::isnan(livox_point.y) ||
        std::isnan(livox_point.z)) {
      continue;
    }

    double dist = std::sqrt(livox_point.x * livox_point.x +
                            livox_point.y * livox_point.y +
                            livox_point.z * livox_point.z);
    // if (dist < min_range_ || dist > max_range_) {
    //     return;
    // }
    // 过滤范围点云
    if (livox_point.x < remove_lidar_front_ &&
        livox_point.x > remove_lidar_back_ &&
        livox_point.y < remove_lidar_left_ &&
        livox_point.y > remove_lidar_right_) {
      continue;
    }
    // 角度过滤
    double point_angle = std::atan2(livox_point.y, livox_point.x);
    for (const auto &angle_range : keep_angles) {
      const double start_rad =
          normalizedAngle(angle_range.first) * M_PI / 180.0;
      const double end_rad = normalizedAngle(angle_range.second) * M_PI / 180.0;
      bool keep_point = false;
      if (start_rad <= end_rad) {
        // -135°-135°
        keep_point = (point_angle >= start_rad && point_angle <= end_rad);
      } else {
        // case (e.g., 135° to -135°)
        keep_point = (point_angle >= start_rad || point_angle <= end_rad);
      }
      if (keep_point) {
        PointType pt;
        pt.x = livox_point.x;
        pt.y = livox_point.y;
        pt.z = livox_point.z;
        pt.intensity = livox_point.intensity;
        // ns -> s
        pt.time = livox_point.timestamp / 1e9;
        filtered_cloud->push_back(pt);
      }
    }
  };
  out_cloud = filtered_cloud;
  // LOG_INFO("after filter point size:{}", out_cloud->size());
  return true;
}
bool LidarProcess::avia_process(
    const PointCloud2MsgConstPtr &cloud_msg,
    PointCloudPtr &out_cloud) {
  return false;
}
bool LidarProcess::ls16_process(
    const PointCloud2MsgConstPtr &cloud_msg,
    PointCloudPtr &out_cloud) {
  return false;
}
bool LidarProcess::rs16_process(
    const PointCloud2MsgConstPtr &cloud_msg,
    PointCloudPtr &out_cloud) {
  // LOG_INFO("rs16_process");
  // 创建点云
  pcl::PointCloud<slam::RsPointXYZIRT>::Ptr cloud(
      new pcl::PointCloud<slam::RsPointXYZIRT>);
  // 转换点云
  pcl::fromROSMsg(*cloud_msg, *cloud);
  int points_num = cloud->points.size();
  // LOG_INFO("time:{}", cloud_msg->header.stamp.toSec());
  // 角度过滤点云，距离过滤点云，以及降采样
  PointCloudPtr filtered_cloud(new PointCloudType);
  filtered_cloud->reserve(points_num);
  int valid_num = 0;
  for (int i = 0; i < points_num; ++i) {
    valid_num++;
    if (valid_num % point_filter_num_ != 0) {
      continue;
    }
    auto point = cloud->points[i];
    // 过滤nan点
    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
      continue;
    }

    double dist =
        std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (dist < min_range_ || dist > max_range_) {
      continue;
    }
    // 过滤范围点云
    if (point.x < remove_lidar_front_ && point.x > remove_lidar_back_ &&
        point.y < remove_lidar_left_ && point.y > remove_lidar_right_) {
      continue;
    }
    // 角度过滤
    double point_angle = std::atan2(point.y, point.x);
    for (const auto &angle_range : keep_angles) {
      const double start_rad =
          normalizedAngle(angle_range.first) * M_PI / 180.0;
      const double end_rad = normalizedAngle(angle_range.second) * M_PI / 180.0;
      bool keep_point = false;
      if (start_rad <= end_rad) {
        // -135°-135°
        keep_point = (point_angle >= start_rad && point_angle <= end_rad);
      } else {
        // case (e.g., 135° to -135°)
        keep_point = (point_angle >= start_rad || point_angle <= end_rad);
      }
      if (keep_point) {
        PointType pt;
        pt.x = point.x;
        pt.y = point.y;
        pt.z = point.z;
        pt.intensity = point.intensity;
        // ns -> s
        pt.time = point.timestamp;
        // std::cout << std::fixed << "pt time:" << pt.time << std::endl;

        filtered_cloud->push_back(pt);
      }
    }
  };
  out_cloud = filtered_cloud;
  // LOG_INFO("after filter point size:{}", out_cloud->size());
  return true;

  return false;
}
bool LidarProcess::airy_process(
    const PointCloud2MsgConstPtr &cloud_msg,
    PointCloudPtr &out_cloud) {
  return false;
}
bool LidarProcess::vanjee_process(
    const PointCloud2MsgConstPtr &cloud_msg,
    PointCloudPtr &out_cloud) {
  return false;
}
bool LidarProcess::velodyne16_process(
    const PointCloud2MsgConstPtr &cloud_msg,
    PointCloudPtr &out_cloud) {
  LOG_INFO("velodyne16_process");
  // 创建点云
  pcl::PointCloud<slam::VelodynePointXYZIRT>::Ptr cloud(
      new pcl::PointCloud<slam::VelodynePointXYZIRT>);
  // 转换点云
  pcl::fromROSMsg(*cloud_msg, *cloud);
  // int points_num = cloud->points.size();
  double cloud_start_time = slam::StampToSec(cloud_msg->header.stamp);
  LOG_INFO("cloud_start_time:{}", cloud_start_time);

  int plsize = cloud->points.size();
  const int MAX_LINE_NUM = 16;
  bool is_first[MAX_LINE_NUM];
  double yaw_fp[MAX_LINE_NUM] = {0};     // yaw of first scan point
  double omega_l = 3.61;                 // scan angular velocity
  float yaw_last[MAX_LINE_NUM] = {0.0};  // yaw of last scan point
  float time_last[MAX_LINE_NUM] = {0.0}; // last offset time
  bool given_offset_time = false;
  if (cloud->points[plsize - 1].time > 0) {
    given_offset_time = true;
    // LOG_INFO("given_offset_time");
  } else {
    given_offset_time = false;
    memset(is_first, true, sizeof(is_first));
    double yaw_first = atan2(cloud->points[0].y, cloud->points[0].x) * 57.29578;
    double yaw_end = yaw_first;
    int layer_first = cloud->points[0].ring;
    for (uint i = plsize - 1; i > 0; i--) {
      if (cloud->points[i].ring == layer_first) {
        yaw_end = atan2(cloud->points[i].y, cloud->points[i].x) * 57.29578;
        break;
      }
    }
  }

  for (int i = 0; i < plsize; i++) {
    PointType added_pt;
    // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

    // added_pt.normal_x = 0;
    // added_pt.normal_y = 0;
    // added_pt.normal_z = 0;
    added_pt.x = cloud->points[i].x;
    added_pt.y = cloud->points[i].y;
    added_pt.z = cloud->points[i].z;
    added_pt.intensity = cloud->points[i].intensity;
    added_pt.time = cloud_start_time + cloud->points[i].time;
    // LOG_INFO("time:{}", added_pt.time);
    if (!given_offset_time) {
      int layer = cloud->points[i].ring;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

      if (is_first[layer]) {
        // printf("layer: %d; is first: %d", layer, is_first[layer]);
        yaw_fp[layer] = yaw_angle;
        is_first[layer] = false;
        added_pt.time = 0.0;
        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.time;
        continue;
      }

      // compute offset time
      if (yaw_angle <= yaw_fp[layer]) {
        added_pt.time = (yaw_fp[layer] - yaw_angle) / omega_l;
      } else {
        added_pt.time = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
      }

      if (added_pt.time < time_last[layer])
        added_pt.time += 360.0 / omega_l;

      // added_pt.curvature = pl_orig.points[i].t;

      yaw_last[layer] = yaw_angle;
      time_last[layer] = added_pt.time;
    }
    if (i == 0)
      LOG_INFO("time:{}", added_pt.time);

    // if(i==(plsize-1))  printf("index: %d layer: %d, yaw: %lf, offset-time:
    // %lf, condition: %d\n", i, layer, yaw_angle, added_pt.curvature, prints);
    if (i % point_filter_num_ == 0) {
      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y +
              added_pt.z * added_pt.z >
          min_range_ * min_range_) {
        out_cloud->push_back(added_pt);
      }
    }
  }
  LOG_INFO("end time:{}", out_cloud->points.back().time);
  return true;
}
bool LidarProcess::velodyne32_process(
    const PointCloud2MsgConstPtr &cloud_msg,
    PointCloudPtr &out_cloud) {
  return false;
}
bool LidarProcess::ouster64_process(
    const PointCloud2MsgConstPtr &cloud_msg,
    PointCloudPtr &out_cloud) {
  return false;
}
#if ROS_AVAILABLE == 1
// mid360的livox driver1消息处理(仅 ROS1)
bool LidarProcess::mid360_process(
    const LivoxMsg1ConstPtr &cloud_msg,
    PointCloudPtr &out_cloud) {
  return false;
}
#endif
// mid360的livox driver2消息处理
bool LidarProcess::mid360_process(
    const LivoxMsg2ConstPtr &cloud_msg,
    PointCloudPtr &out_cloud) {
  return false;
}
#if ROS_AVAILABLE == 1
// avia的livox driver1消息处理(仅 ROS1)
bool LidarProcess::avia_process(
    const LivoxMsg1ConstPtr &cloud_msg,
    PointCloudPtr &out_cloud) {
  return false;
}
#endif
bool LidarProcess::avia_process(
    const LivoxMsg2ConstPtr &msg,
    PointCloudPtr &out_cloud) {
  // LOG_INFO("avia_process");

  int point_num = msg->point_num;
  out_cloud->clear();
  out_cloud->reserve(point_num / point_filter_num_ + 1);
  uint valid_num = 0;
  double time_start = slam::StampToSec(msg->header.stamp);

  for (uint i = 0; i < point_num; i++) {
    if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10 ||
                                      (msg->points[i].tag & 0x30) == 0x00)) {
      if ((valid_num++) % point_filter_num_ != 0)
        continue;
      PointType p;
      p.x = msg->points[i].x;
      p.y = msg->points[i].y;
      p.z = msg->points[i].z;
      p.intensity = msg->points[i].reflectivity;
      p.time = time_start + msg->points[i].offset_time / 1e9; // 纳秒->毫秒
      double sq_range = p.x * p.x + p.y * p.y + p.z * p.z;
      if (sq_range > (min_range_ * min_range_) &&
          sq_range < max_range_ * max_range_) {
        out_cloud->push_back(p);
      }
    }
  }
  // int points_num = msg->point_num;
  // // 角度过滤点云，距离过滤点云，以及降采样
  // PointCloudPtr filtered_cloud(new PointCloudType);
  // filtered_cloud->reserve(points_num);
  // int valid_num = 0;
  // double time_start = msg->header.stamp.toSec();
  // // std::cout << std::fixed << "time_start: " << time_start << std::endl;
  // for (int i = 0; i < points_num; ++i) {
  //     if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10
  //     || (msg->points[i].tag & 0x30) == 0x00)) {
  //         valid_num++;
  //         if (valid_num % point_filter_num_ != 0) {
  //             continue;
  //         }
  //         float x = msg->points[i].x;
  //         float y = msg->points[i].y;
  //         float z = msg->points[i].z;
  //         float dist2 = x * x + y * y + z * z;
  //         if (dist2 < min_range_ * min_range_ || dist2 > max_range_ *
  //         max_range_) {
  //             continue;
  //         }
  //         PointType pt;
  //         pt.x = x;
  //         pt.y = y;
  //         pt.z = z;
  //         pt.intensity = msg->points[i].reflectivity;
  //         //
  //         每个点的时间戳是相对于第一个点的时间戳的偏移量，需要加上第一个点的时间戳
  //         pt.time = time_start + msg->points[i].offset_time / 1e9;
  //         // std::cout << std::fixed << "time_start: " << pt.time <<
  //         std::endl;

  //         pt.ring = msg->points[i].line;
  //         filtered_cloud->push_back(pt);
  //     }
  // }
  // out_cloud = filtered_cloud;
  // LOG_INFO("after filter point size:{}", out_cloud->size());
  return true;
}

} // namespace slam