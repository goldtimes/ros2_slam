#include "lidar_register/voxelmap_register.hh"

namespace slam {
VoxelMapRegister::VoxelMapRegister(const std::shared_ptr<SystemConfig>& system_config) : LidarRegister(system_config) {
    LOG_INFO("VoxelMapRegister init done!");
}

VoxelMapRegister::~VoxelMapRegister() {
}

bool VoxelMapRegister::InitMap(PointCloudPtr& cloud_world) {
    return true;
}

}  // namespace slam