/*
 * @Author: lihang lihang@kilox.cn
 * @Date: 2025-08-25 20:36:34
 * @LastEditors: lihang lihang@kilox.cn
 * @LastEditTime: 2025-09-11 20:50:26
 * @FilePath: /fast_lvio_ws/src/lio_slam/include/lidar_register/inc_ndt.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <Eigen/Eigen>
#include <list>
#include "common/commons.hh"
#include "common/eigen_type.hh"
#include "common/logger.hh"
#include "lio/state.hh"
#include "utils/pointcloud_utils.hh"
namespace slam {

class IncNdt {
   public:
    enum class NEARBY_TYPE {
        CENTER,
        NEARBY6,
    };

    using KeyType = Eigen::Matrix<int, 3, 1>;
    using V3i = Eigen::Matrix<int, 3, 1>;

    // 体素内的结构体
    struct VoxelData {
        VoxelData() {
        }
        VoxelData(const V3D &pt) {
            pts_.emplace_back(pt);
            num_pts_ = 1;
        }

        void AddPoint(const V3D &pt) {
            pts_.emplace_back(pt);
            if (!ndt_estimated_) {
                num_pts_++;
            }
        }

        std::vector<V3D> pts_;
        V3D mu_ = V3D::Zero();     // 均值
        M3D sigma_ = M3D::Zero();  // 协方差
        M3D info_ = M3D::Zero();   // 信息矩阵
        int num_pts_ = 0;
        bool ndt_estimated_ = false;
    };

    IncNdt(double voxel_size, bool near_search, int max_capacity, int min_effective_pts, int min_pts_in_voxel,
           int max_pts_in_voxel, double res_outlier_thresh, double eps, bool calib_lidar2imu);
    ~IncNdt();

    /// 获取一些统计信息
    int NumGrids() const {
        return grids_.size();
    }

    /// 在voxel里添加点云，
    void AddCloud(PointCloudXYZIPtr &cloud_world);

    /// 设置被配准的Scan
    void SetSource(const PointCloudXYZIPtr &source) {
        // 需要被转换到imu坐标系
        source_ = source;
    }

    /**
     * 计算给定Pose下的雅可比和残差矩阵，符合IEKF中符号（8.17, 8.19）
     * @param pose
     * @param HTVH
     * @param HTVr
     */
    void ComputeResidualAndJacobians(State &nav_state, ESKFShareState &shared_data);

    /// 获取所有已估计体素的中心点，用于构建submap
    void GetVoxelCenters(PointCloudXYZIPtr &cloud) const;

   private:
    void GenerateNearbyGrids();

    void UpdateVoxel(VoxelData &v);

   public:
    PointCloudXYZIPtr source_;

   private:
    double voxel_size_;
    double inv_voxel_size_;
    bool near_search_;
    int max_capacity_;
    int min_effective_pts_;
    int min_pts_in_voxel_;
    int max_pts_in_voxel_;
    double res_outlier_thresh_;
    double eps_;

    bool first_frame_ = true;
    bool calib_lidar2imu_ = false;

    using KeyAndData = std::pair<KeyType, VoxelData>;
    std::list<KeyAndData> data_;  // 真实的数据，用于缓存/清理
    std::unordered_map<KeyType, std::list<KeyAndData>::iterator, hash_vec<3>> grids_;  // 栅格数据，存储真实数据的迭代器

    std::vector<KeyType> nearby_grids_;
};

}  // namespace slam