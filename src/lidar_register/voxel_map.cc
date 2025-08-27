#include "lidar_register/voxel_map.hh"

namespace slam {

OctoTree::OctoTree(int _max_layer, int _layer, std::vector<int> _update_size_threshes, int _max_point_thresh,
                   double _plane_thresh)
    : max_layer(_max_layer),
      layer(_layer),
      update_size_threshes(_update_size_threshes),
      max_point_thresh(_max_point_thresh),
      plane_thresh(_plane_thresh) {
    tmp_points.clear();
    is_leave = false;
    all_point_num = 0;
    new_point_num = 0;
    update_size_thresh_for_new = 5;
    is_initialized = false;
    update_enable = true;
    update_size_thresh = update_size_threshes[layer];
    plane.is_valid = false;
    leaves.resize(8, nullptr);
}

void OctoTree::Insert(const std::vector<PointWithCov>& points) {
    // 空树
    if (!is_initialized) {
        tmp_points.insert(tmp_points.end(), points.begin(), points.end());
        all_point_num += tmp_points.size();
        new_point_num += tmp_points.size();
        InitialTree();
        return;
    }
    // 叶子节点了
    if (is_leave) {
        if (update_enable) {
            // 允许更新
        }
        return;
    }
    // 不是叶子节点
    if (layer < max_layer - 1) {
        // 清空
        if (tmp_points.size() != 0) std::vector<PointWithCov>().swap(tmp_points);
        std::vector<std::vector<PointWithCov>> package(8, std::vector<PointWithCov>(0));
        // 计算点落在哪个子体素中
        for (size_t i = 0; i < points.size(); ++i) {
            int xyz[3] = {0, 0, 0};
            int leafnum = SubIndex(points[i], xyz);
            if (leaves[leafnum] == nullptr) {
                // 创建子树
                leaves[leafnum] = std::make_shared<OctoTree>(max_layer, layer + 1, update_size_thresh, max_point_thresh,
                                                             plane_thresh);
                V3D shift((2 * xyz[0] - 1) * quater_length, (2 * xyz[1] - 1) * quater_length,
                          (2 * xyz[2] - 1) * quater_length);
                leaves[leafnum]->center = center + shift;
                leaves[leafnum]->quater_length = quater_length / 2;
            }
            package[leafnum].push_back(points[i]);
        }
        for (int i = 0; i < 8; ++i) {
            if (package[i].size() == 0) {
                continue;
            } else {
                leaves[i]->Insert(package[i]);
            }
        }
    }
}
void OctoTree::InitialTree() {
    if (all_point_num < update_size_thresh) {
        return;
    }
    is_initialized = true;
    new_point_num = 0;
    BuildPlane(tmp_points);
    if (plane.is_valid) {
        // 拟合平面成功
        is_leave = true;  // 设置为叶子节点
        if (tmp_points.size() > max_point_thresh) {
            update_enable = false;  // 不允许更新
            std::vector<PointWithCov>().swap(tmp_points);
        } else {
            update_enable = true;  // 允许更新
        }
    } else {
        // 拟合失败，则进行分割
        SplitTree();
    }
}
// 论文一样，计算平面的参数以及协方差
void OctoTree::BuildPlane(const std::vector<PointWithCov>& points) {
    plane.plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
    plane.covariance = Eigen::Matrix3d::Zero();
    plane.center = Eigen::Vector3d::Zero();
    plane.normal = Eigen::Vector3d::Zero();
    plane.points_size = points.size();

    for (auto pv : points) {
        plane.covariance += pv.point * pv.point.transpose();
        plane.center += pv.point;
    }
    plane.center = plane.center / plane.points_size;
    plane.covariance = plane.covariance / plane.points_size - plane.center * plane.center.transpose();

    Eigen::EigenSolver<Eigen::Matrix3d> es(plane.covariance);
    Eigen::Matrix3cd evecs = es.eigenvectors();
    Eigen::Vector3cd evals = es.eigenvalues();
    Eigen::Vector3d evalsReal = evals.real();
    Eigen::Matrix3d::Index evalsMin, evalsMax;
    evalsReal.rowwise().sum().minCoeff(&evalsMin);
    evalsReal.rowwise().sum().maxCoeff(&evalsMax);
    int evalsMid = 3 - evalsMin - evalsMax;

    Eigen::Matrix3d J_Q = Eigen::Matrix3d::Identity() / static_cast<double>(plane.points_size);
    plane.eigens << evalsReal(evalsMin), evalsReal(evalsMid), evalsReal(evalsMax);
    plane.normal = evecs.real().col(evalsMin);
    plane.x_normal = evecs.real().col(evalsMax);
    plane.y_normal = evecs.real().col(evalsMid);

    if (plane.eigens[0] < plane_thresh) {
        for (int i = 0; i < points.size(); i++) {
            Eigen::Matrix<double, 6, 3> J;
            Eigen::Matrix3d F;
            for (int m = 0; m < 3; m++) {
                if (m != (int)evalsMin) {
                    Eigen::Matrix<double, 1, 3> F_m = (points[i].point - plane.center).transpose() /
                                                      ((plane.points_size) * (evalsReal[evalsMin] - evalsReal[m])) *
                                                      (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() +
                                                       evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
                    F.row(m) = F_m;
                } else {
                    Eigen::Matrix<double, 1, 3> F_m;
                    F_m << 0, 0, 0;
                    F.row(m) = F_m;
                }
            }
            J.block<3, 3>(0, 0) = evecs.real() * F;
            J.block<3, 3>(3, 0) = J_Q;
            plane.plane_cov += J * points[i].cov * J.transpose();
        }
        plane.is_valid = true;
    } else {
        plane.is_valid = false;
    }
}

void OctoTree::SplitTree() {
    if (layer >= max_layer - 1) {
        is_leave = true;
        return;
    }
    std::vector<std::vector<PointWithCov>> package(8, std::vector<PointWithCov>(0));

    for (size_t i = 0; i < tmp_points.size(); i++) {
        int xyz[3] = {0, 0, 0};
        int leafnum = SubIndex(tmp_points[i], xyz);
        if (leaves[leafnum] == nullptr) {
            leaves[leafnum] =
                std::make_shared<OctoTree>(max_layer, layer + 1, update_size_threshes, max_point_thresh, plane_thresh);
            Eigen::Vector3d shift((2 * xyz[0] - 1) * quater_length, (2 * xyz[1] - 1) * quater_length,
                                  (2 * xyz[2] - 1) * quater_length);
            leaves[leafnum]->center = center + shift;
            leaves[leafnum]->quater_length = quater_length / 2;
        }
        package[leafnum].push_back(tmp_points[i]);
    }

    for (int i = 0; i < 8; i++) {
        if (package[i].size() == 0) continue;
        leaves[i]->Insert(package[i]);
    }
    std::vector<PointWithCov>().swap(tmp_points);
}
// 想象一个立方体，被切割成8个小立方体，编号从0-7
int OctoTree::SubIndex(const PointWithCov& pv, int* xyz) {
    if (pv.point[0] > center[0]) xyz[0] = 1;
    if (pv.point[1] > center[1]) xyz[1] = 1;
    if (pv.point[2] > center[2]) xyz[2] = 1;
    return 4 * xyz[0] + 2 * xyz[1] + xyz[2];
}

VoxelMap::VoxelMap(double voxel_size, int max_layer, const std::vector<int>& update_size_threshes, int max_point_thresh,
                   double plane_thresh, int capacity) {
    this->voxel_size = voxel_size;
    this->max_layer = max_layer;
    this->update_size_threshes = update_size_threshes;
    this->max_point_thresh = max_point_thresh;
    this->plane_thresh = plane_thresh;
    this->capacity = capacity;
    feat_map.clear();
    sub_map.clear();
    cache.clear();
}

void VoxelMap::Insert(const std::vector<PointWithCov>& points) {
    Pack(points);
    for (auto& pair : sub_map) {
        if (pair.second.type == SubVoxelType::INSERT) {
            cache.push_front(pair.first);
            pair.second.it = cache.begin();
            // 创建体素
            feat_map[pair.first].tree =
                std::make_shared<OctoTree>(max_layer, 0, update_size_threshes, max_point_thresh, plane_thresh);
            feat_map[pair.first].it = pair.second.it;
            feat_map[pair.first].tree->center =
                V3D((0.5 + pair.first.x_) * voxel_size, (0.5 + pair.first.y_) * voxel_size,
                    (0.5 + pair.first.z_) * voxel_size);
            feat_map[pair.first].tree->quater_length = voxel_size / 4;
            feat_map[pair.first].tree->Insert(pair.second.points);
            if (cache.size() > capacity) {
                feat_map.erase(cache.back());
                cache.pop_back();
            }
        } else {
            cache.splice(cache.begin(), cache, pair.second.it);
            feat_map[pair.first].tree->Insert(pair.second.points);
        }
    }
}
void VoxelMap::Pack(const std::vector<PointWithCov>& points) {
    sub_map.clear();
    // 决定点落在的体素是插入还是更新
    uint plsize = points.size();
    for (int i = 0; i < plsize; ++i) {
        const PointWithCov& p_v = points[i];
        VoxelKey key = Index(p_v.point);
        auto it = feat_map.find(key);
        auto sub_it = sub_map.find(key);
        if (it == feat_map.end()) {
            if (sub_it == sub_map.end()) {
                // 标记为插入新的体素
                sub_map[key].type = SubVoxelType::INSERT;
            }
            // 如果已存在子体素，则将点添加入体素中
            sub_map[key].points.push_back(p_v);
        } else {
            if (!feat_map[key].tree->update_enable) {
                continue;
            }
            if (sub_it == sub_map.end()) {
                sub_map[key].type = SubVoxelType::UPDATE;
                sub_map[key].it = feat_map[key].it;
            }
            sub_map[key].points.push_back(p_v);
        }
    }
}
VoxelKey VoxelMap::Index(const V3D& point) {
    Eigen::Vector3d idx = (point / voxel_size).array().floor();
    return VoxelKey(static_cast<int64_t>(idx(0)), static_cast<int64_t>(idx(1)), static_cast<int64_t>(idx(2)));
}

void VoxelMap::BuildResidual(ResidualData& info, std::shared_ptr<OctoTree> oct_tree) {
    if (oct_tree->plane.is_valid) {
        Eigen::Vector3d p_world_to_center = info.point_world - oct_tree->plane.center;
        info.plane_center = oct_tree->plane.center;
        info.plane_norm = oct_tree->plane.normal;
        info.plane_cov = oct_tree->plane.plane_cov;
        info.residual = info.plane_norm.transpose() * p_world_to_center;
        double dis_to_plane = std::abs(info.residual);
        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0) = p_world_to_center;
        J_nq.block<1, 3>(0, 3) = -info.plane_norm;
        double sigma_l = J_nq * info.plane_cov * J_nq.transpose();
        sigma_l += info.plane_norm.transpose() * info.cov * info.plane_norm;
        if (dis_to_plane < info.sigma_num * sqrt(sigma_l)) {
            info.is_valid = true;
        }
    } else {
        if (info.current_layer < max_layer - 1) {
            for (size_t i = 0; i < 8; i++) {
                if (oct_tree->leaves[i] == nullptr) continue;
                info.current_layer += 1;
                BuildResidual(info, oct_tree->leaves[i]);
                if (info.is_valid) break;
            }
        }
    }
}
}  // namespace slam