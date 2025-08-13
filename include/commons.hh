#pragma once
#include <deque>
#include "logger.hh"
#include "pointcloud_utils.hh"
#include "sensors.hh"

namespace slam {
enum class SLAM_MODE {
    MAPPING,         // 建图模式
    LOCALIZATION,    // 定位模式
    RELOCALIZATION,  // 重定位模式
    NONE,            // 无模式
    TRACKING,        // 里程计跟踪模式
};

struct MeasureGroup {
    PointCloudPtr curent_cloud;
    double lidar_beg_time;
    double lidar_end_time;
    std::deque<IMU> imus;
    std::deque<Encoder> encoders;
    std::deque<GNSS> gnsss;
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

template <typename FuncT>
inline void evaluate_and_call(FuncT&& func, const std::string& func_name = "", bool print = false) {
    double total_time = 0;

    auto t1 = std::chrono::high_resolution_clock::now();
    func();
    auto t2 = std::chrono::high_resolution_clock::now();
    // second
    total_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    if (print) {
        LOG_INFO("方法{},调用时间:{} ", func_name, total_time);
    }
}

template <typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C& collects, D& mean, D& cov_diag, Getter&& getter) {
    size_t len = collects.size();
    // 计算均值
    mean = std::accumulate(collects.begin(), collects.end(), D::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) /
           len;
    // 计算协方差对角线
    cov_diag = std::accumulate(collects.begin(), collects.end(), D::Zero().eval(),
                               [&getter, &mean](const D& sum, const auto& data) {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) /
               (len - 1);
}

template <typename C, int dim, typename Getter>
void ComputeMeanAndCov(const C& collects, Eigen::Matrix<double, dim, 1>& mean, Eigen::Matrix<double, dim, dim>& cov,
                       Getter&& getter) {
    using D = Eigen::Matrix<double, dim, 1>;
    using E = Eigen::Matrix<double, dim, dim>;
    size_t len = collects.size();
    // 计算均值
    // clang-format off

    mean = std::accumulate(collects.begin(), collects.end(), D::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    // 计算协方差对角线
    cov = std::accumulate(collects.begin(), collects.end(), E::Zero().eval(),
                               [&getter, &mean](const E& sum, const auto& data) -> E{
                                auto value = getter(data).eval();
                                D v = value - mean;
                                return sum + v * v.transpose();
                               }) / (len - 1);
    // clang-format on
}

}  // namespace slam