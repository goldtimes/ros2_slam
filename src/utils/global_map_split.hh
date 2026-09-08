// ============================================================
//  全局地图切图元(共享工具)
//  由 map_split(命令行工具) 与 PGO 保存优化地图时共同调用。
//  无 ROS 依赖, 仅依赖 PCL / yaml-cpp / boost::filesystem。
// ============================================================
#pragma once
#include <cstddef>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace slam {

/**
 * @brief 将一张全局地图(内存点云)按 XY 网格切分为图元(META)。
 *
 * 输出目录结构(兼容 Localizer::LoadMetaMapsFromDir 的目录扫描):
 *   out_dir/<leaf_id>/META<i>/data.pcd   // 图元局部系: 点减图元中心(cx,cy), z 保持全局
 *                           /data.yaml   // floor:0, identity:<leaf_id>, T:[cx,cy,0,0,0,0,1]
 *
 * 说明: 图元必须存"局部坐标 + T", 这样 Localizer::LoadMapByPose 才能用
 * meta_info->x/y(=T.t) 做按位置动态加载/卸载的距离判定。
 *
 * @param cloud            全局地图点云(全局/map 系)
 * @param out_dir          输出根目录(其下创建 <leaf_id>)
 * @param leaf_id          叶子地图 identity(目录名)
 * @param tile_size        网格边长(米), 例如 50.0
 * @param min_tile_points  点数少于该值的格子丢弃(默认 10)
 * @return 生成的图元数量
 */
int SplitGlobalMapToTiles(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const std::string &out_dir,
                          const std::string &leaf_id, double tile_size, std::size_t min_tile_points = 10);

}  // namespace slam
