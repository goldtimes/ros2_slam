#include <ros/ros.h>
#include "logger.hh"
#include "system.hh"

int main(int argc, char **argv) {
    ros::init(argc, argv, "openslam_node");
    ros::NodeHandle nh("~");
    SpdLogger logger;
    std::shared_ptr<slam::System> system_ptr = std::make_shared<slam::System>(nh);
    // system_ptr->run();
    ros::spin();
    return 0;
}