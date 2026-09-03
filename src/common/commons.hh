#pragma once
#include "common/eigen_type.hh"
#include "logger.hh"
#include "pose_trans.hh"
#include "sensors.hh"
#include "utils/pointcloud_utils.hh"
#include <boost/math/tools/precision.hpp>
#include <deque>
#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0
namespace slam {
enum class SLAM_MODE {
  MAPPING,        // 建图模式
  LOCALIZATION,   // 定位模式
  RELOCALIZATION, // 重定位模式
  NONE,           // 无模式
  ODOMETER,       // 里程计跟踪模式
};

enum class UPDATE_SENSOR {
  LIDAR,
  ENCODER,
  GNSS,
};

struct MeasureGroup {
  PointCloudPtr curent_cloud;
  double lidar_beg_time;
  double lidar_end_time;
  std::deque<IMU> imus;
  std::deque<Encoder> encoders;
  std::deque<GNSS> gnsss;
  MeasureGroup() {
    curent_cloud.reset(new PointCloudType);
    imus.clear();
    encoders.clear();
    gnsss.clear();
  }
};

// 滤波器的输入
struct Input {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Input() = default;
  Input(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro, double dt)
      : acc(acc), gyro(gyro), dt(dt) {}
  // 中值积分
  Eigen::Vector3d acc;
  Eigen::Vector3d gyro;
  double dt;
};

// 迭代是否合理的
struct ESKFShareState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // lidar的观测，主要对位姿和外参的雅可比矩阵
  M12D
      H_; // Hessian矩阵，这里可以保持和state的维度是一致，不过是构造了很多0矩阵
  V12D b_; // b矩阵
  // 轮速计和gnss的观测海森矩阵
  M33D H33_;
  V33D b33_;
  double res;
  bool valid = false;
  size_t iter_num = 0;
  void print() { LOG_INFO("iter_num: {}, res: {}", iter_num, res); }
};

template <typename T> inline void print_matrix(const T &mat, std::string name) {
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
inline void evaluate_and_call(FuncT &&func, const std::string &func_name = "",
                              bool print = false) {
  double total_time = 0;

  auto t1 = std::chrono::high_resolution_clock::now();
  func();
  auto t2 = std::chrono::high_resolution_clock::now();
  // second
  total_time =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
          .count();
  if (print) {
    LOG_INFO("方法{},调用时间:{} ", func_name, total_time);
  }
}

template <typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C &collects, D &mean, D &cov_diag,
                           Getter &&getter) {
  size_t len = collects.size();
  // 计算均值
  mean = std::accumulate(collects.begin(), collects.end(), D::Zero().eval(),
                         [&getter](const D &sum, const auto &data) -> D {
                           return sum + getter(data);
                         }) /
         len;
  // 计算协方差对角线
  cov_diag =
      std::accumulate(collects.begin(), collects.end(), D::Zero().eval(),
                      [&getter, &mean](const D &sum, const auto &data) {
                        return sum + (getter(data) - mean).cwiseAbs2().eval();
                      }) /
      (len - 1);
}

template <typename C, int dim, typename Getter>
void ComputeMeanAndCov(const C &collects, Eigen::Matrix<double, dim, 1> &mean,
                       Eigen::Matrix<double, dim, dim> &cov, Getter &&getter) {
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
 * 高斯分布合并
 * @tparam S    scalar type
 * @tparam D    dimension
 * @param hist_m        历史点数
 * @param curr_n        当前点数
 * @param hist_mean     历史均值
 * @param hist_var      历史方差
 * @param curr_mean     当前均值
 * @param curr_var      当前方差
 * @param new_mean      新的均值
 * @param new_var       新的方差
 */
template <typename S, int D>
void UpdateMeanAndCov(int hist_m, int curr_n,
                      const Eigen::Matrix<S, D, 1> &hist_mean,
                      const Eigen::Matrix<S, D, D> &hist_var,
                      const Eigen::Matrix<S, D, 1> &curr_mean,
                      const Eigen::Matrix<S, D, D> &curr_var,
                      Eigen::Matrix<S, D, 1> &new_mean,
                      Eigen::Matrix<S, D, D> &new_var) {
  assert(hist_m > 0);
  assert(curr_n > 0);
  new_mean = (hist_m * hist_mean + curr_n * curr_mean) / (hist_m + curr_n);
  new_var =
      (hist_m * (hist_var + (hist_mean - new_mean) *
                                (hist_mean - new_mean).template transpose()) +
       curr_n * (curr_var + (curr_mean - new_mean) *
                                (curr_mean - new_mean).template transpose())) /
      (hist_m + curr_n);
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
inline bool InterpolatePose(double query_time, C &&data, FT &&take_time_func,
                            FP &&take_pose_func, SE3 &result, T &best_match,
                            float time_th = 0.5) {
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
      best_match = *data.rbegin(); // NormalState
      return true;
    }
    return false;
  }

  auto match_iter = data.begin();
  // --imu_pose_1--query_time---imu_pose_2---
  for (auto iter = data.begin(); iter != data.end(); ++iter) {
    auto next_it = iter;
    next_it++;
    if (take_time_func(*iter) < query_time &&
        take_time_func(*next_it) >= query_time) {
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

  result =
      SE3(pose_first.unit_quaternion().slerp(s, pose_second.unit_quaternion()),
          pose_first.translation() * (1 - s) + pose_second.translation() * s);
  best_match = s < 0.5 ? *match_iter : *match_iter_next;
  return true;
}

/// 矢量哈希
template <int N> struct hash_vec {
  inline size_t operator()(const Eigen::Matrix<int, N, 1> &v) const;
};

template <>
inline size_t hash_vec<2>::operator()(const Eigen::Matrix<int, 2, 1> &v) const {
  return size_t(((v[0] * 73856093) ^ (v[1] * 471943)) % 10000000);
}

template <>
inline size_t hash_vec<3>::operator()(const Eigen::Matrix<int, 3, 1> &v) const {
  return size_t(((v[0] * 73856093) ^ (v[1] * 471943) ^ (v[2] * 83492791)) %
                10000000);
}

/// 矢量比较
template <int N> struct less_vec {
  inline bool operator()(const Eigen::Matrix<int, N, 1> &v1,
                         const Eigen::Matrix<int, N, 1> &v2) const;
};

// 实现2D和3D的比较
template <>
inline bool less_vec<2>::operator()(const Eigen::Matrix<int, 2, 1> &v1,
                                    const Eigen::Matrix<int, 2, 1> &v2) const {
  return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
}

template <>
inline bool less_vec<3>::operator()(const Eigen::Matrix<int, 3, 1> &v1,
                                    const Eigen::Matrix<int, 3, 1> &v2) const {
  return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]) ||
         (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] < v2[2]);
}

template <class scalar>
inline std::pair<scalar, scalar> cos_sinc_sqrt(const scalar &x2) {
  using std::cos;
  using std::sin;
  using std::sqrt;
  static scalar const taylor_0_bound = boost::math::tools::epsilon<scalar>();
  static scalar const taylor_2_bound = sqrt(taylor_0_bound);
  static scalar const taylor_n_bound = sqrt(taylor_2_bound);

  assert(x2 >= 0 && "argument must be non-negative");

  // FIXME check if bigger bounds are possible
  if (x2 >= taylor_n_bound) {
    // slow fall-back solution
    scalar x = sqrt(x2);
    return std::make_pair(cos(x), sin(x) / x); // x is greater than 0.
  }

  // FIXME Replace by Horner-Scheme (4 instead of 5 FLOP/term, numerically more
  // stable, theoretically cos and sinc can be calculated in parallel using SSE2
  // mulpd/addpd)
  // TODO Find optimal coefficients using Remez algorithm
  static scalar const inv[] = {1 / 3., 1 / 4., 1 / 5., 1 / 6.,
                               1 / 7., 1 / 8., 1 / 9.};
  scalar cosi = 1., sinc = 1;
  scalar term = -1 / 2. * x2;
  for (int i = 0; i < 3; ++i) {
    cosi += term;
    term *= inv[2 * i];
    sinc += term;
    term *= -inv[2 * i + 1] * x2;
  }

  return std::make_pair(cosi, sinc);
}

inline SO3 exp(const V3D &vec, const double &scale = 1) {
  double norm2 = vec.squaredNorm();
  std::pair<double, double> cos_sinc = cos_sinc_sqrt(scale * scale * norm2);
  double mult = cos_sinc.second * scale;
  V3D result = mult * vec;
  return SO3(Quatd(cos_sinc.first, result[0], result[1], result[2]));
}

/**
 * SO3 Jl()/JacobianL()
 * @param v
 * @return
 */
inline Eigen::Matrix<double, 3, 3> A_matrix(const V3D &v) {
  Eigen::Matrix<double, 3, 3> res;
  double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
  double norm = std::sqrt(squaredNorm);
  if (norm < 1e-5) {
    res = Eigen::Matrix<double, 3, 3>::Identity();
  } else {
    res = Eigen::Matrix<double, 3, 3>::Identity() +
          (1 - std::cos(norm)) / squaredNorm * SO3::hat(v) +
          (1 - std::sin(norm) / norm) / squaredNorm * SO3::hat(v) * SO3::hat(v);
  }
  return res;
}

} // namespace slam