#include <pcl/point_cloud.h>
#include "lidar_point_type.hh"
namespace slam {
using PointType = PointXYZIRT;
using PointCloudType = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloudType::Ptr;

}  // namespace slam