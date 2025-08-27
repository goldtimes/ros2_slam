#pragma once

#include <cstdint>
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>
#include "eigen_type.hh"
#define HASH_P 116101
#define MAX_N 10000000000

namespace slam {

// 体素地图的键
class VoxelKey {
   public:
    VoxelKey(int64_t x, int64_t y, int64_t z) : x_(x), y_(y), z_(z) {
    }
    ~VoxelKey() = default;

    bool operator==(const VoxelKey& other) const {
        return x_ == other.x_ && y_ == other.y_ && z_ == other.z_;
    }

    struct Hasher {
        int64_t operator()(const VoxelKey& key) const {
            return ((((key.z_) * HASH_P) % MAX_N + (key.y_)) * HASH_P) % MAX_N + (key.x_);
        }
    };
    int64_t x_;
    int64_t y_;
    int64_t z_;
};

struct PointWithCov {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D point;
    M3D cov;
};

struct Plane {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D center;
    V3D normal;
    V3D x_normal;
    V3D y_normal;
    M3D covariance;
    V3D eigens;
    Eigen::Matrix<double, 6, 6> plane_cov;
    bool is_valid;
    int points_size;
};

struct ResidualData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D plane_center;
    V3D plane_norm;
    Eigen::Matrix<double, 6, 6> plane_cov;
    Eigen::Matrix3d pcov;
    Eigen::Matrix3d cov;
    Eigen::Vector3d point_lidar;
    Eigen::Vector3d point_world;
    bool is_valid = false;
    bool from_near = false;
    int current_layer = 0;
    double sigma_num = 3.0;
    double residual = 0.0;
};

class OctoTree {
   public:
    OctoTree(int _max_layer, int _layer, std::vector<int> _update_size_threshes, int _max_point_thresh,
             double _plane_thresh);
    ~OctoTree() = default;

    void Insert(const std::vector<PointWithCov>& points);
    void InitialTree();
    void BuildPlane(const std::vector<PointWithCov>& points);

    void SplitTree();
    int SubIndex(const PointWithCov& pv, int* xyz);

   public:
    double quater_length;
    V3D center;
    Plane plane;
    int layer;
    int max_layer;
    bool is_leave;
    bool is_initialized;
    bool update_enable;
    std::vector<std::shared_ptr<OctoTree>> leaves;
    std::vector<PointWithCov> tmp_points;
    std::vector<int> update_size_threshes;
    double plane_thresh;
    int max_point_thresh;
    int update_size_thresh;
    int update_size_thresh_for_new;
    int all_point_num;
    int new_point_num;
};

// 体素地图中的value，也就是八叉树
struct VoxelValue {
    std::list<VoxelKey>::iterator it;
    std::shared_ptr<OctoTree> tree;
};

// 插入体素或者更新体素
enum class SubVoxelType {
    INSERT,
    UPDATE,
};

struct VoxelGrid {
    SubVoxelType type;
    std::list<VoxelKey>::iterator it;
    std::vector<PointWithCov> points;
};

using FeatMap = std::unordered_map<VoxelKey, VoxelValue, VoxelKey::Hasher>;
using SubMap = std::unordered_map<VoxelKey, VoxelGrid, VoxelKey::Hasher>;

class VoxelMap {
   public:
    VoxelMap(double voxel_size, int max_layer, const std::vector<int>& update_size_threshes, int max_point_thresh,
             double plane_thresh, int capacity);
    ~VoxelMap() = default;
    void Insert(const std::vector<PointWithCov>& points);
    void Pack(const std::vector<PointWithCov>& points);
    VoxelKey Index(const V3D& point);
    void BuildResidual(ResidualData& info, std::shared_ptr<OctoTree> oct_tree);

   public:
    FeatMap feat_map;
    SubMap sub_map;
    std::list<VoxelKey> cache;
    double voxel_size;
    int max_layer;
    std::vector<int> update_size_threshes;
    int max_point_thresh;
    double plane_thresh;
    int capacity;
};

}  // namespace slam