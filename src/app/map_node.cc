#include "common/logger.hh"
#include "ros/ros1_manager.hh"
#include "system/system.hh"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "openslam_node");
  ros::NodeHandle nh("~");
  SpdLogger logger;
  std::string config_path;
  std::string slam_mode;
  nh.param<std::string>("config_path", config_path, "");
  nh.param<std::string>("slam_mode", slam_mode, "mapping");

  std::shared_ptr<slam::System> system_ptr =
      std::make_shared<slam::System>(config_path, slam_mode);
  std::shared_ptr<slam::ROS1Manager> ros1_manager_ptr =
      std::make_shared<slam::ROS1Manager>(nh, system_ptr);

  ros::Rate rate(1000);
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}