#pragma once
#include "logger.hh"

namespace slam {
enum class SLAM_MODE {
    MAPPING,         // 建图模式
    LOCALIZATION,    // 定位模式
    RELOCALIZATION,  // 重定位模式
    NONE,            // 无模式
    TRACKING,        // 里程计跟踪模式
};

template <typename T>
inline void print_matrix(const T& mat, std::string name) {
    // clang-format off
    LOG_INFO(REDPURPLE
             "{}:\n"
             "-[{:1.3f},{:1.3f},{:1.3f},{:1.3f}]\n"
             "-[{:1.3f},{:1.3f},{:1.3f},{:1.3f}]\n"
             "-[{:1.3f},{:1.3f},{:1.3f},{:1.3f}]\n"
             "-[{:1.3f},{:1.3f},{:1.3f},{:1.3f}]\n" RESET,
             name, 
             mat(0, 0), mat(0, 1), mat(0, 2), mat(0, 3), 
             mat(1, 0), mat(1, 1), mat(1, 2), mat(1, 3), 
             mat(2, 0), mat(2, 1), mat(2, 2), mat(2, 3), 
             mat(3, 0), mat(3, 1), mat(3, 2), mat(3, 3));
    // clang-format on
}

}  // namespace slam