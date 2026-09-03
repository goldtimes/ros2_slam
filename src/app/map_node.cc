#include "common/logger.hh"
#include "ros/ros_common.hh"
#include "system/system.hh"

#if ROS_AVAILABLE == 1
#include <ros/ros.h>
#include "ros/ros1_manager.hh"
#elif ROS_AVAILABLE == 2
#include <rclcpp/executor.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "ros/ros2_manager.hh"
#endif

#if ROS_AVAILABLE == 1
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
#elif ROS_AVAILABLE == 2
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  SpdLogger logger;

  rclcpp::NodeOptions options;
  // 允许未声明的参数(通过 launch / --ros-args 透传)
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("slam_node", options);

  std::string config_path;
  std::string slam_mode;
  node->get_parameter_or("config_path", config_path, std::string(""));
  node->get_parameter_or("slam_mode", slam_mode, std::string("mapping"));

  auto system_ptr = std::make_shared<slam::System>(config_path, slam_mode);
  auto ros2_manager_ptr = std::make_shared<slam::ROS2Manager>(node, system_ptr);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
#endif