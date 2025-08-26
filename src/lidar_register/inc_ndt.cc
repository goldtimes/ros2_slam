#include "lidar_register/inc_ndt.hh"
#include <set>

namespace slam {
IncNdt::IncNdt(double voxel_size, bool near_search, int max_capacity, int min_effective_pts, int min_pts_in_voxel,
               int max_pts_in_voxel, double res_outlier_thresh, double eps)
    : voxel_size_(voxel_size),
      near_search_(near_search),
      max_capacity_(max_capacity),
      min_effective_pts_(min_effective_pts),
      min_pts_in_voxel_(min_pts_in_voxel),
      max_pts_in_voxel_(max_pts_in_voxel),
      res_outlier_thresh_(res_outlier_thresh),
      eps_(eps) {
    inv_voxel_size_ = 1.0 / voxel_size_;
    GenerateNearbyGrids();
}
IncNdt::~IncNdt() {
}

void IncNdt::GenerateNearbyGrids() {
    if (!near_search_)
        nearby_grids_.emplace_back(KeyType::Zero());
    else if (near_search_) {  // 上下左右前后
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    }
}

void IncNdt::AddCloud(PointCloudPtr& cloud_world) {
    std::set<KeyType, less_vec<3>> active_voxels;
    for (const auto& point : cloud_world->points) {
        auto pt_eigen = ToV3D(point);
        KeyType key = (pt_eigen * inv_voxel_size_).cast<int>();
        auto iter = grids_.find(key);
        if (iter == grids_.end()) {
            // 栅格不存在
            data_.push_front({key, VoxelData(pt_eigen)});
            grids_.insert({key, data_.begin()});
            if (data_.size() >= max_capacity_) {
                grids_.erase(data_.back().first);
                data_.pop_back();
            }
        } else {
            // 如果栅格存在,获取迭代器，获取voxel数据，添加点
            iter->second->second.AddPoint(pt_eigen);
            // void splice(const_iterator pos, list& x, const_iterator pos_x);
            // 将源 list x 中位于迭代器 pos_x 处的单个元素，插入到目标 list *this 的迭代器 pos 之前。
            // 将data_的iter->second，放到data_最前面
            data_.splice(data_.begin(), data_, iter->second);
            iter->second = data_.begin();
        }
        active_voxels.insert(key);
    }
    // 更新voxel
    for (auto it = active_voxels.begin(); it != active_voxels.end(); it++) {
        KeyType key = *it;
        VoxelData& v = grids_[key]->second;
        UpdateVoxel(v);
    }
    first_frame_ = false;
}

void IncNdt::ComputeResidualAndJacobians(const SE3& input_pose, M12D& HTVH, V12D& HTVr) {
    assert(grids_.empty() == false);
    SE3 pose = input_pose;
    int num_residual_per_point = 1;
    if (near_search_) num_residual_per_point = 7;
    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }
    int total_size = index.size() * num_residual_per_point;
    std::vector<bool> effect_pts(total_size, false);                  // 用于标记有效点
    std::vector<Eigen::Matrix<double, 3, 12>> jacobians(total_size);  // 用于存储雅可比矩阵
    std::vector<V3D> errors(total_size);                              // 用于存储残差
    std::vector<M3D> infos(total_size);                               // 用于存储信息矩阵
    for (auto& idx : index) {
        auto point_body = source_->points[idx];
        V3D pt_body = ToV3D(point_body);
        V3D pt_world = pose * pt_body;
        Eigen::Vector3i key = (pt_world * inv_voxel_size_).cast<int>();
        
    }
}

void IncNdt::UpdateVoxel(VoxelData& v) {
    if (first_frame_) {
        // 第一帧，将所有体素都估计一次
        if (v.pts_.size() > 1) {
        } else if (v.pts_.size() == 1) {
            v.mu_ = v.pts_[0];
            v.info_ = M3D::Identity() * 1e2;
        } else {
            // 其实是一定有点的
            v.mu_ = V3D::Zero();
            v.info_ = M3D::Identity() * 1e2;
        }
        v.ndt_estimated_ = true;
        v.pts_.clear();
        return;
    }
    // 体素已经估计过了，并且点数 > 阈值
    if (v.ndt_estimated_ && v.num_pts_ > max_pts_in_voxel_) {
        return;
    }
    // 新增的体素
    if (!v.ndt_estimated_ && v.pts_.size() > min_pts_in_voxel_) {
        // 计算均值和方差
        ComputeMeanAndCov(v.pts_, v.mu_, v.sigma_, [this](const V3D& pt) { return pt; });
        v.info_ = (v.sigma_ + M3D::Identity() * 1e-3).inverse();
        v.ndt_estimated_ = true;
        v.pts_.clear();
    } else if (v.ndt_estimated_ && v.pts_.size() > min_pts_in_voxel_) {
        // 已经估计过了，但是还有新来的点,更新均值和方差
        V3D cur_mu, new_mu;
        M3D cur_var, new_var;
        // 先估计当前的均值和方差
        ComputeMeanAndCov(v.pts_, cur_mu, cur_var, [this](const V3D& pt) { return pt; });
        // 更新均值和方差
        UpdateMeanAndCov(v.num_pts_, v.pts_.size(), v.mu_, v.info_, cur_mu, cur_var, new_mu, new_var);
        v.mu_ = new_mu;
        v.sigma_ = new_var;
        v.num_pts_ += v.pts_.size();
        v.pts_.clear();
        // check info
        Eigen::JacobiSVD svd(v.sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
        V3D lambda = svd.singularValues();

        if (lambda[1] < lambda[0] * 1e-3) lambda[1] = lambda[0] * 1e-3;

        if (lambda[2] < lambda[0] * 1e-3) lambda[2] = lambda[0] * 1e-3;

        M3D inv_lambda = V3D(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();
        v.info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
    }
}
}  // namespace slam