
#include <ros/ros.h>
#include "ros/ros1_manager.hh"
#include "system.hh"
#include "utils/logger.hh"

int main(int argc, char **argv) {
    ros::init(argc, argv, "lio_slam");
    ros::NodeHandle nh("~");
    SpdLogger logger;
    std::string config_path;
    nh.param<std::string>("config_path", config_path, "");
    std::shared_ptr<slam::System> system_ptr = std::make_shared<slam::System>(config_path);
    std::shared_ptr<slam::ROS1Manager> ros1_manager_ptr = std::make_shared<slam::ROS1Manager>(nh, system_ptr);

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}