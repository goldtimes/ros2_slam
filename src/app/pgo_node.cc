#include "PGO/PosegraphOptimization.hh"

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