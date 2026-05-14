#include "common/eigen_type.hh"
#include "geo_graphic/LocalCartesian.hpp"
#include "utils/logger.hh"

namespace slam {

class GnssProcess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GnssProcess() = default;
    ~GnssProcess() = default;

    double time = -1;
    V3D lla_ = V3D::Zero();     // 经纬高
    V3D enu_ = V3D::Zero();     // local的enu坐标
    V3D origin_ = V3D::Zero();  // 原点
    double heading_ = -1;       // 朝向角
    bool has_heading = false;   // 朝向是否有效
    int status_ = -1;           // gps解的状态
    int service_ = -1;          // gps服务类型

   public:
    void InitOrigin(const V3D& lla);

    void UpdateXYZYaw(const V3D& lla, double yaw = -1, bool heading_valid = false);

    void Reverse(const V3D& enu, V3D& lla);
    // 双天线的方案
    void SetHeadingValid(bool valid) {
        has_heading = valid;
    }

    void SetExtrinsic(const M3D& R_GtoI, const V3D& t_GinI) {
        R_GtoI_ = R_GtoI;
        t_GinI_ = t_GinI;
    }

   private:
    GeographicLib::LocalCartesian geo_converter_;
    // 外参信息
    M3D R_GtoI_ = M3D::Identity();
    V3D t_GinI_ = V3D::Zero();
    V3D pose_cov_ = V3D::Zero();
};
}  // namespace slam