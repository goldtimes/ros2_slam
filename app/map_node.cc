/*
 * @Author: lihang lihang@kilox.cn
 * @Date: 2025-08-11 17:48:59
 * @LastEditors: lihang lihang@kilox.cn
 * @LastEditTime: 2025-09-10 16:39:58
 * @FilePath: /fast_lvio_ws/src/open_slam/app/map_node.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <ros/ros.h>
#include "logger.hh"
#include "ros/ros1_manager.hh"
#include "system.hh"

int main(int argc, char **argv) {
    ros::init(argc, argv, "openslam_node");
    ros::NodeHandle nh("~");
    SpdLogger logger;
    std::string config_path;
    nh.param<std::string>("config_path", config_path, "");
    std::shared_ptr<slam::System> system_ptr = std::make_shared<slam::System>(config_path);
    std::shared_ptr<slam::ROS1Manager> ros1_manager_ptr = std::make_shared<slam::ROS1Manager>(nh, system_ptr);

    ros::Rate rate(1000);
    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}