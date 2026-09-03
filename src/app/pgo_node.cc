#include "PGO/PosegraphOptimization.hh"

#if ROS_AVAILABLE == 2
#include <rclcpp/executor.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#endif

#if ROS_AVAILABLE == 1
int main(int argc, char **argv) {
  ros::init(argc, argv, "pgo_node");
  ros::NodeHandle nh("~");
  SpdLogger::Config logger_config;
  logger_config.name = "pgo_node";
  SpdLogger logger(logger_config);

  slam::PosegraphOptimization pgo(nh);
  ros::spin();
  return 0;
}
#elif ROS_AVAILABLE == 2
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  SpdLogger::Config logger_config;
  logger_config.name = "pgo_node";
  SpdLogger logger(logger_config);

  rclcpp::NodeOptions options;
  // 允许未声明的参数(通过 launch / --ros-args 透传)
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("pgo_node", options);

  slam::PosegraphOptimization pgo(node);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
#endif