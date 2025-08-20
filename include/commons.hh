#pragma once
#include <deque>
#include "eigen_type.hh"
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

// 滤波器的输入
struct Input {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Input() = default;
    Input(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, double dt) : acc(acc), gyro(gyro), dt(dt) {
    }
    // 中值积分
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    double dt;
};

// 迭代是否合理的
struct ESKFShareState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    M12D H_;  // Hessian矩阵，这里可以保持和state的维度是一致，不过是构造了很多0矩阵
    V12D b_;  // b矩阵
    double res;
    bool valid = false;
    size_t iter_num = 0;
    void print() {
        LOG_INFO("iter_num: {}, res: {}", iter_num, res);
    }
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

/**
 * pose插值算法
 * @tparam T  数据类型 NavState
 * @tparam C 容器类型 std::deque<NavState>
 * @tparam FT 获取时间函数
 * @tparam FP 获取pose函数
 * @param query_time 查询时间
 * @param data 容器数据
 * @param take_time_func 获取时间函数
 * @param take_pose_func 获取pose函数
 * @param result 插值后的pose
 * @param best_match 最佳匹配
 * @param time_th 时间阈值
 */
template <typename T, typename C, typename FT, typename FP>
inline bool InterpolatePose(double query_time, C&& data, FT&& take_time_func, FP&& take_pose_func, SE3& result,
                            T& best_match, float time_th = 0.5) {
    if (data.empty()) {
        LOG_ERROR("can't interpolate pose, data is empty");
        return false;
    }
    // 容器中最后一个数据
    double last_time = take_time_func(*data.rbegin());
    // 查询时间 > imus的pose
    if (query_time > last_time) {
        if (query_time < (last_time + time_th)) {
            // 可接受的位姿
            result = take_pose_func(*data.rbegin());
            best_match = *data.rbegin();  // NormalState
            return true;
        }
        return false;
    }

    auto match_iter = data.begin();
    // --imu_pose_1--query_time---imu_pose_2---
    for (auto iter = data.begin(); iter != data.end(); ++iter) {
        auto next_it = iter;
        next_it++;
        if (take_time_func(*iter) < query_time && take_time_func(*next_it) >= query_time) {
            match_iter = iter;
            break;
        }
    }
    // 接下来要做插值
    auto match_iter_next = match_iter;
    match_iter_next++;

    double dt = take_time_func(*match_iter_next) - take_time_func(*match_iter);
    // 插值的系数
    double s = (query_time - take_time_func(*match_iter)) / dt;
    if (std::fabs(dt) < 1e-6) {
        best_match = *match_iter;
        result = take_pose_func(*match_iter);
        return true;
    }

    SE3 pose_first = take_pose_func(*match_iter);
    SE3 pose_second = take_pose_func(*match_iter_next);

    result = SE3(pose_first.unit_quaternion().slerp(s, pose_second.unit_quaternion()),
                 pose_first.translation() * (1 - s) + pose_second.translation() * s);
    best_match = s < 0.5 ? *match_iter : *match_iter_next;
    return true;
}

}  // namespace slam