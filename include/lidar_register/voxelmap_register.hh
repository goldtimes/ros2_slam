#pragma
#include "lidar_register.hh"

namespace slam {
class VoxelMapRegister : public LidarRegister {
   public:
    VoxelMapRegister(const std::shared_ptr<SystemConfig>& system_config);

    ~VoxelMapRegister();

    virtual bool InitMap(PointCloudPtr& cloud_world) override;
};
}  // namespace slam