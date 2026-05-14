
#pragma once
#include "common/eigen_type.hh"
#include "common/lidar_point_type.hh"
#include "lidar_register.hh"
#include "lio/ikd_tree.hh"
#include "utils/pointcloud_utils.hh"
namespace slam {
struct LocalMap {
    bool initialized = false;
    BoxPointType local_map_corner;
    std::vector<BoxPointType> cub_to_rm;
};
class P2PlaneRegister : public LidarRegister {
   public:
    P2PlaneRegister(const std::shared_ptr<SystemConfig> &system_config, std::shared_ptr<IESKF> kf_ptr);

    ~P2PlaneRegister();

    virtual bool InitMap(PointCloudXYZIPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) override;
    virtual bool Align(PointCloudXYZIPtr &cloud_lidar, std::shared_ptr<IESKF> kf_ptr_) override;
    virtual void UpdateLidarFunc(State &nav_state, ESKFShareState &shared_data) override;
    virtual void UpdateMap() override;
    virtual PointCloudXYZIPtr GetSubmap() override;

    void TrimCloud();
    void IncreMap();

   private:
    bool EstimatePlane(const PointVec &points, double thresh, Eigen::Vector4d &plane_coeff);

    float sq_dist(const PointXYZI &p1, const PointXYZI &p2) {
        return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    }

   private:
    LocalMap m_local_map;
    std::shared_ptr<KD_TREE<PointXYZI>> m_ikdtree;

    PointCloudXYZIPtr cloud_world;
    std::vector<bool> m_point_selected_flag;
    PointCloudXYZIPtr m_norm_vec;
    PointCloudXYZIPtr m_effect_cloud_lidar;
    PointCloudXYZIPtr m_effect_norm_vec;

    std::vector<PointVec> m_nearest_points;

    double map_resolution = 0.1;
    int cube_len = 100;
    int det_range = 60;
    double move_thresh = 1.5;
    double p2plane_thresh = 0.01;

    double lidar_noise_std_;
    double lidar_info_matrix_;
    bool updated_success = true;
};
}  // namespace slam