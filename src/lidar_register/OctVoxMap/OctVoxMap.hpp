#pragma once

#include <array>
#include "common/eigen_type.hh"
#include "tsl/robin_map.h"

namespace slam {
// kNN算法，查找点的最近k个点

template <int K, typename Point>
class KNNHeap {
   public:
    uint8_t count_ = 0;
    uint8_t worst_idx_ = 0;
    // 存储找到最近的几个点
    std::array<Point, K> points_;
    // 存储找到最近的几个点的距离索引
    float dist2_[K];
    float max_dist2_ = 0.0f;  // 最大距离的平方，
    KNNHeap() : max_dist2_(0.0f), worst_idx_(0), count_(0) {
        memset(dist2_, 0, sizeof(dist2_));
    }
    // 尝试插入点,判断这个点是否需要插入，并且距离是否大于最大距离
    inline void try_insert(const Point& point, float dist2) {
        const bool not_full = points_.size() < K;
        const bool should_insert = not_full || dist2 < max_dist2_;
        // 如果需要插入
        if (should_insert) {
            // 插入下标
            uint8_t insert_idx = not_full ? count_ : worst_idx_;
            points_[insert_idx] = point;
            // 如果是满了情况下，这里就更新了最大距离，需要在后面更新最大距离的下标和最大距离
            dist2_[insert_idx] = dist2;
            if (not_full) {
                count_++;
                // 如果距离大于最大距离，就更新最大距离，并且记录这个下标，为什么这里不sort一下？
                if (dist2 > max_dist2_) {
                    max_dist2_ = dist2;
                    worst_idx_ = insert_idx;
                }
            } else {
                // 如果满了,切dist2 < 最大距离，我们需要插入，并且更新最大距离
                float d0 = dist2_[0], d1 = dist2_[1], d2 = dist2_[2], d3 = dist2_[3], d4 = dist2_[4];
                // 比较0,1，取距离较大的那个
                uint8_t idx01 = d0 > d1 ? 0 : 1;
                float max01 = d0 > d1 ? d0 : d1;
                // 比较2,3，取距离较大的那个
                uint8_t idx23 = d2 > d3 ? 2 : 3;
                float max23 = d2 > d3 ? d2 : d3;
                // 比较0,1,2,3，取距离较大的那个
                uint8_t idx0123 = max01 > max23 ? idx01 : idx23;
                float max0123 = max01 > max23 ? max01 : max23;
                // 最大的值
                worst_ = max0123 > d4 ? idx0123 : 4;
                max_dist2_ = max0123 > d4 ? max0123 : d4;
            }
        }
    }
};
// 定义单个voxel体素的结构体,我们采用模板类,方便后续扩展
// 我们限制了体素最多只能存8个点
// 然后进行滤波
template <typename Point>
class OctVox {
   public:
    static constexpr uint8_t UNINIT_MASK = 0x00;
    static constexpr uint8_t MAX_POINTS_PER_SUBVOXEL = 20;
    static constexpr double DISTANCE_THRESHOLD_SQ = 0.1 * 0.1;  // 0.01cm

    // 构造函数
    OctVox() = default;
    // 构造函数
    OctVox(const Point& point, uint8_t idx) {
        count_.fill(UNINIT_MASK);
        points_[idx] = point;
        count_[idx] = 1;
    }
    ~OctVox() = default;

    // 往体素中添加点，最多的观测次数为MAX_POINTS_PER_SUBVOXEL
    void AddPoint(const Point& point, uint8_t idx) {
        uint8_t& count = count_[idx];
        Point& stored_point = points_[idx];
        if (count == UNINIT_MASK) {
            // 为空
            stored_point = point;
            count = 1;
            return;
        }
        if (count >= MAX_POINTS_PER_SUBVOXEL) {
            // 已满
            return;
        }
        // 距离过大的点,不添加
        if ((pt - stored_point).squaredNorm() > DISTANCE_THRESHOLD_SQ) {
            return;
        }
        // 动态更新均值
        stored_point = (stored_point * count_ + point) / (count + 1);
        count++;
    }
    // 获取点
    bool getPoint(const uint8_t idx, Point& point) const {
        if (count_[idx] == UNINIT_MASK) {
            return false;
        }
        point = points_[idx];
        return true;
    }

    std::array<uint8_t, 8> count_;  // 计数
    std::array<Point, 8> points_;   // array 栈内存，固定大小，就是一个数组
};

/** @brief 体素构成的八叉树地图
 * 存储所有体素
 * 体素的下标
 */
template <typename Point, typename Scalar>
class OctVoxMap {
   public:
    using Ptr = std::shared_ptr<OctVoxMap>;                              // 指针
    using OctVoxType = OctVox<Point>;                                    // 体素类型
    using KEY = Eigen::Vector3i;                                         // 体素的下标
    using Points = std::vector<Point, Eigen::aligned_allocator<Point>>;  // 点的集合
    using KNNHeaptType = KNNHeap<5, Points>;                             // KNN堆类型

    // 配置结构体
    struct Options {
        // 分辨率
        float resolution = 0.5;
        // 容量
        std::size_t capacity = 1e6;
        Options(float _resolution, std::size_t _capacity) {
            resolution = _resolution;
            capacity = _capacity;
        }
    };

    // 构造函数
    OctVoxMap() = default;
    // 构造函数
    OctVoxMap(const Point& point, uint8_t idx) {
    }
    ~OctVoxMap() = default;
    struct HASH_VEC {
        std::size_t operator()(const KEY& v) const {
            size_t h = static_cast<size_t>(v[0]);
            h ^= v[1] * 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= v[2] * 0x85ebca6b + (h << 6) + (h >> 2);
            return h;
        }
    };
    // TODO hash 方式
   private:
    float resolution_ = 0.5;
    float inv_resolution_ = 1.0;
    float sub_resolution_ = 0.25;
    float inv_sub_resolution_ = 4.0;
    std::size_t capacity_ = 1e6;

    bool reset_map_ = false;
    int reset_map_count_ = 0;

    using DATA_LIST = std::list<std::pair<KEY, OctVoxType>>;
    using DATE_ITER = DATA_LIST::iterator;
    // TODO 所有体素的存储 map类型，存储的是下标索引和迭代器的位置，这里将迭代器的位置进行hash散列
    tsl::robin_map<KEY, DATE_ITER, HASH_VEC> grids_;
    // 存储voxel的list,list的每个元素是<下标，体素>
    DATA_LIST data_;
};

}  // namespace slam