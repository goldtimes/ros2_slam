#include "gnss_process.hh"

namespace slam {

void GnssProcess::InitOrigin(const V3D& lla) {
    geo_converter_.Reset(lla[0], lla[1], lla[2]);
    origin_ = lla;
    LOG_INFO("Init origin: {}, {}, {}", lla[0], lla[1], lla[2]);
}

void GnssProcess::UpdateXYZYaw(const V3D& lla, double yaw, bool heading_valid) {
    if (heading_valid && has_heading) {
        heading_ = yaw;
    }
    geo_converter_.Forward(lla[0], lla[1], lla[2], enu_[0], enu_[1], enu_[2]);
}

void GnssProcess::Reverse(const V3D& enu, V3D& lla) {
    geo_converter_.Reverse(enu[0], enu[1], enu[2], lla[0], lla[1], lla[2]);
}
}  // namespace slam