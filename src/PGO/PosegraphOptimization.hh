#pragma once
#include <ros/ros.h>
#include <thread>

namespace slam {
class PosegraphOptimization {
   public:
   private:
    ros::NodeHandle nh_;
    // 地图可视化线程
    std::thread map_visualization_thread_;
};
}  // namespace slam