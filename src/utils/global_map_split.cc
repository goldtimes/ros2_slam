#include "utils/global_map_split.hh"
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
#include <cstdint>
#include <fstream>
#include <limits>
#include <unordered_map>

namespace slam {

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

int SplitGlobalMapToTiles(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const std::string &out_dir,
                          const std::string &leaf_id, double tile_size, std::size_t min_tile_points) {
    if (!cloud || cloud->empty() || tile_size <= 0.0) {
        return 0;
    }

    // 1. XY 包围盒
    double minx = std::numeric_limits<double>::max();
    double maxx = -std::numeric_limits<double>::max();
    double miny = std::numeric_limits<double>::max();
    double maxy = -std::numeric_limits<double>::max();
    for (const auto &p : cloud->points) {
        minx = std::min(minx, static_cast<double>(p.x));
        maxx = std::max(maxx, static_cast<double>(p.x));
        miny = std::min(miny, static_cast<double>(p.y));
        maxy = std::max(maxy, static_cast<double>(p.y));
    }
    const long ncols = std::max(1L, static_cast<long>(std::floor((maxx - minx) / tile_size)) + 1);
    const long nrows = std::max(1L, static_cast<long>(std::floor((maxy - miny) / tile_size)) + 1);

    // 2. 分桶
    std::unordered_map<TileKey, Tile, TileKeyHash> tiles;
    tiles.reserve(static_cast<std::size_t>(ncols) * static_cast<std::size_t>(nrows));
    for (const auto &p : cloud->points) {
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

    // 3. 输出 <out_dir>/<leaf_id>/META<i>/{data.pcd,data.yaml}
    const boost::filesystem::path leaf_path = boost::filesystem::path(out_dir) / leaf_id;
    boost::filesystem::create_directories(leaf_path);

    std::size_t tile_idx = 0;
    for (auto &kv : tiles) {
        Tile &t = kv.second;
        if (t.cloud.points.size() < min_tile_points) {
            continue;  // 噪声格丢弃
        }
        // 点云转图元局部坐标: 减图元中心(x,y), z 保持全局
        for (auto &p : t.cloud.points) {
            p.x -= static_cast<float>(t.cx);
            p.y -= static_cast<float>(t.cy);
        }
        t.cloud.width = static_cast<std::uint32_t>(t.cloud.points.size());
        t.cloud.height = 1;
        t.cloud.is_dense = false;

        const boost::filesystem::path meta_path = leaf_path / ("META" + std::to_string(tile_idx));
        boost::filesystem::create_directories(meta_path);
        if (pcl::io::savePCDFileBinary((meta_path / "data.pcd").string(), t.cloud) < 0) {
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
        std::ofstream fout((meta_path / "data.yaml").string());
        fout << yaml.c_str();
        fout.close();
        tile_idx++;
    }
    return static_cast<int>(tile_idx);
}

}  // namespace slam
