// ============================================================
//  离线全局地图切块工具(无 ROS 依赖, 独立命令行程序)
//
//  功能: 把单张全局地图 pcd(如 superlio_globalmap.pcd)按 XY 网格
//        切成多个"图元(tile)", 输出目录结构兼容 Localizer 的
//        LoadMapByPose 动态加载方式:
//
//          <out_dir>/<leaf_id>/META0/data.pcd
//                                  data.yaml  (floor/identity/T)
//          <out_dir>/<leaf_id>/META1/...
//
//  说明:
//    - 图元点云存为"图元局部坐标"(点减图元中心 cx,cy, z 保持全局);
//      T 记录图元中心平移, Localizer::LoadMapByPose 合并时会用
//      TransformLidar(T) 还原回全局坐标。
//    - 这样 meta_info->x/y(=T.t) 才是图元中心, 距离判定/动态卸载
//      才能正常工作(不能把所有图元的 T 都写成 0)。
//
//  用法:
//    map_split <input.pcd> <out_dir> [leaf_id] [tile_size]
//      leaf_id    图元所属叶子地图标识, 默认 MAP_GLOBAL
//      tile_size  网格边长(米), 默认 25.0
// ============================================================
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

struct TileKey {
    long ix = 0;
    long iy = 0;
    bool operator==(const TileKey &o) const {
        return ix == o.ix && iy == o.iy;
    }
};

struct TileKeyHash {
    std::size_t operator()(const TileKey &k) const {
        return std::hash<long>()(k.ix) ^ (std::hash<long>()(k.iy) << 1);
    }
};

struct Tile {
    TileKey key;
    double cx = 0.0;  // 图元中心(全局坐标)
    double cy = 0.0;
    pcl::PointCloud<pcl::PointXYZI> cloud;
};

long Clamp(long v, long lo, long hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <input.pcd> <out_dir> [leaf_id] [tile_size]\n"
                  << "  leaf_id    default MAP_GLOBAL\n"
                  << "  tile_size  default 25.0 (m)\n";
        return 1;
    }
    const std::string input_path = argv[1];
    const std::string out_dir = argv[2];
    const std::string leaf_id = argc > 3 ? argv[3] : "MAP_GLOBAL";
    const double tile_size = argc > 4 ? std::atof(argv[4]) : 25.0;
    if (tile_size <= 0.0) {
        std::cerr << "[map_split] invalid tile_size: " << tile_size << "\n";
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_path, cloud) < 0) {
        std::cerr << "[map_split] failed to load " << input_path << "\n";
        return 1;
    }
    std::cout << "[map_split] input points: " << cloud.size() << "\n";
    if (cloud.empty()) {
        std::cerr << "[map_split] empty input cloud\n";
        return 1;
    }

    // ---- 1. 计算 XY 包围盒 ----
    double minx = std::numeric_limits<double>::max();
    double maxx = -std::numeric_limits<double>::max();
    double miny = std::numeric_limits<double>::max();
    double maxy = -std::numeric_limits<double>::max();
    for (const auto &p : cloud.points) {
        minx = std::min(minx, static_cast<double>(p.x));
        maxx = std::max(maxx, static_cast<double>(p.x));
        miny = std::min(miny, static_cast<double>(p.y));
        maxy = std::max(maxy, static_cast<double>(p.y));
    }
    const long ncols = std::max(1L, static_cast<long>(std::floor((maxx - minx) / tile_size)) + 1);
    const long nrows = std::max(1L, static_cast<long>(std::floor((maxy - miny) / tile_size)) + 1);
    std::cout << "[map_split] bounds x[" << minx << ", " << maxx << "] y[" << miny << ", " << maxy << "] grid "
              << ncols << "x" << nrows << "\n";

    // ---- 2. 分桶: 每个非空格子生成一个图元 ----
    std::unordered_map<TileKey, Tile, TileKeyHash> tiles;
    tiles.reserve(static_cast<std::size_t>(ncols) * static_cast<std::size_t>(nrows));
    for (const auto &p : cloud.points) {
        long ix = Clamp(static_cast<long>(std::floor((p.x - minx) / tile_size)), 0, ncols - 1);
        long iy = Clamp(static_cast<long>(std::floor((p.y - miny) / tile_size)), 0, nrows - 1);
        TileKey key{ix, iy};
        auto it = tiles.find(key);
        if (it == tiles.end()) {
            Tile t;
            t.key = key;
            t.cx = minx + (ix + 0.5) * tile_size;
            t.cy = miny + (iy + 0.5) * tile_size;
            it = tiles.emplace(key, t).first;
        }
        it->second.cloud.points.push_back(p);
    }
    std::cout << "[map_split] non-empty cells: " << tiles.size() << "\n";

    // ---- 3. 输出目录结构 + data.pcd + data.yaml ----
    const boost::filesystem::path leaf_path = boost::filesystem::path(out_dir) / leaf_id;
    boost::filesystem::create_directories(leaf_path);

    constexpr std::size_t kMinTilePoints = 10;  // 点数过少的噪声格直接丢弃
    std::size_t tile_idx = 0;
    std::size_t saved_points = 0;
    std::size_t skipped = 0;
    for (auto &kv : tiles) {
        Tile &t = kv.second;
        if (t.cloud.points.size() < kMinTilePoints) {
            skipped++;
            continue;
        }
        // 点云转为图元局部坐标: 减图元中心(x,y), z 保持全局
        for (auto &p : t.cloud.points) {
            p.x -= static_cast<float>(t.cx);
            p.y -= static_cast<float>(t.cy);
        }
        t.cloud.width = static_cast<std::uint32_t>(t.cloud.points.size());
        t.cloud.height = 1;
        t.cloud.is_dense = false;

        const boost::filesystem::path meta_path = leaf_path / ("META" + std::to_string(tile_idx));
        boost::filesystem::create_directories(meta_path);
        const std::string pcd_path = (meta_path / "data.pcd").string();
        if (pcl::io::savePCDFileBinary(pcd_path, t.cloud) < 0) {
            std::cerr << "[map_split] save pcd failed: " << pcd_path << "\n";
            continue;
        }

        // data.yaml: floor / identity / T=[x,y,z,qx,qy,qz,qw]
        YAML::Emitter yaml;
        yaml << YAML::BeginMap;
        yaml << YAML::Key << "floor" << YAML::Value << 0;
        yaml << YAML::Key << "identity" << YAML::Value << leaf_id;
        yaml << YAML::Key << "T" << YAML::Value << YAML::Flow
             << std::vector<double>{t.cx, t.cy, 0.0, 0.0, 0.0, 0.0, 1.0};
        yaml << YAML::EndMap;
        const std::string yaml_path = (meta_path / "data.yaml").string();
        std::ofstream fout(yaml_path);
        fout << yaml.c_str();
        fout.close();

        saved_points += t.cloud.points.size();
        std::cout << "[map_split] META" << tile_idx << " center(" << t.cx << ", " << t.cy
                  << ") points " << t.cloud.points.size() << " -> " << pcd_path << "\n";
        tile_idx++;
    }

    std::cout << "[map_split] done. tiles saved: " << tile_idx << ", skipped(cell points<" << kMinTilePoints
              << "): " << skipped << ", saved points: " << saved_points << "\n";
    return 0;
}
