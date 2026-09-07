# 全局地图切图元 + 动态加载定位（ROS2）

> 目标：不整张加载 `superlio_globalmap.pcd`，而是参考定位模块现有的"图元(meta tile)
> 按位置动态加载/卸载"机制（`SetMetaMaps → MapUpdate → LoadMapByPose`），先把全局地图
> 离线切成多个图元，再在 ROS2 定位时按机器人位置只加载附近的图元。

---

## 1. 背景与复用机制

`Localizer` 原本有两条地图来源：

| 路径 | 机制 | ROS1 | ROS2 |
|---|---|---|---|
| 图元地图 | `SetMetaMaps` → `MapUpdate` → `LoadMapByPose`（按 50m 动态加载/卸载） | 有 `metaset_info` 话题驱动 | **无入口** |
| 单张全局图 | `SetMaps(pcd_path)` | 死代码（无调用） | 死代码 |

本次目标 = **把单张 global pcd 变成图元格式，并复用第 1 条机制在 ROS2 下跑通**。

关键点：图元必须存为 **"图元局部坐标 + T"**（点减图元中心），因为
`LoadMapByPose` 用 `meta_info->x/y (= T.t)` 作为与机器人的距离锚点，若所有图元
T 都写 0，距离判定会失效（无法正确动态加载/卸载）。

### 图元目录/文件结构（与 ROS1 兼容）
```
<local_map_dir>/<leaf_id>/META<i>/data.pcd   # 图元点云(局部系: 减中心 cx,cy, z 全局)
                              /data.yaml      # floor / identity / T=[x,y,z,qx,qy,qz,qw]
```

---

## 2. 改动清单

| 文件 | 改动 |
|---|---|
| `src/app/map_split.cc` | **新增**：独立 C++ 切块工具（无 ROS 依赖） |
| `cmake/ROS1.cmake`、`cmake/ROS2.cmake` | 加入 `map_split` 可执行目标（ROS2 加 install） |
| `src/localizer/localizer.hh` | 新增 `LoadMetaMapsFromDir(map_dir)`；成员 `map_identity_` |
| `src/localizer/localizer.cc` | 实现目录扫描加载；`SetInitPose` 增加 identity 解析；`LoadMapByPose` 记录 `curr_meta_info_.identity` |
| `src/system/system_config.hh/.cc` | `LocalizerConfig` 新增可选字段 `map_identity`（兼容旧配置） |
| `src/ros/ros2_manager.cc` | 定位模式 + `use_meta_maps` 时，启动即 `LoadMetaMapsFromDir(local_map_dir)` |
| `config/livox.yaml` | localizer 段指向切块目录并配置 `map_identity` |

---

## 3. 各部分说明

### 3.1 离线切块工具 `map_split`
```bash
map_split <input.pcd> <out_dir> [leaf_id] [tile_size]
#   leaf_id    默认 MAP_GLOBAL
#   tile_size  默认 25.0 m
```
流程：
1. 读 pcd（工程 `PointXYZI` 即标准 `pcl::PointXYZI`，字段 `x y z intensity`）；
2. 算 XY 包围盒 → 切成 `ncols × nrows` 网格（默认 25m）；
3. 逐点分桶（格中心 = 图元原点 `cx,cy`）；
4. 对每个非空格子（点数 ≥10）：
   - 点云减去 `(cx,cy)` 存为局部系 → `data.pcd`（二进制）；
   - 写 `data.yaml`：`floor:0`、`identity:<leaf_id>`、`T:[cx,cy,0,0,0,0,1]`；
5. 打印每块点数与路径。

> 选择"局部系 + T"而非全局系存点的原因：`LoadMapByPose` 中
> `dis_to_robot = dis_a_b(meta_info->x, meta_info->y, robot_x, robot_y)`，
> `meta_info->x/y` 取自 `T.t`，即图元中心；合并时用
> `TransformLidar(map_pcd, T.R, T.t)` 还原回全局坐标。

### 3.2 Localizer 目录扫描加载
`LoadMetaMapsFromDir(map_dir)`（与 ROS1 `MetamapsCallback` 等价，无消息依赖）：
- 遍历 `map_dir` 下每个叶子目录 → 其中 `META*` 子目录；
- 解析 `data.yaml` 的 `floor / identity / T`，填 `ids_metamap_map_[identity]`；
- 非空则 `SetMetaMaps`（状态 → `NOT_INIT`）；若未配置 `map_identity_` 且只有
  一个叶子地图，则记为默认 identity。

`SetInitPose` 的 identity 解析：
```
map_id 有效(存在于 ids)  → 用之
map_id 为空/无效         → 配置 map_identity_ → 唯一叶子地图
```
从而兼容 ROS2 仅有的 `/initialpose`（该消息不带 map_id）。

### 3.3 ROS2 启动接线
`ros2_manager.cc` 构造函数中（Localizer 已由 System 在定位模式创建）：
```cpp
if (slam_mode_ == SLAM_MODE::LOCALIZATION && GetLocalizer() != nullptr &&
    config.localizer_config_.use_meta_maps) {
    GetLocalizer()->LoadMetaMapsFromDir(config.localizer_config_.local_map_dir);
}
```

### 3.4 配置（`config/livox.yaml` localizer 段）
```yaml
localizer:
    use_meta_maps: true
    local_map_dir: "/home/li/ros2_ws/maps/superlio_tiles"
    map_identity: "MAP_GLOBAL"   # 可空；空时自动用唯一叶子地图
    # ... 其余搜索/GICP 参数沿用
```

---

## 4. 端到端流程

```mermaid
flowchart LR
    A[superlio_globalmap.pcd] -->|map_split 25m| B[图元目录 superlio_tiles]
    B --> C{启动 slam_mode=slam}
    C --> D[ROS2Manager ctor 调 LoadMetaMapsFromDir]
    D --> E[ids_metamap_map_ 就绪 / state=NOT_INIT]
    E --> F[/initialpose]
    F --> G[SetInitPose: identity=MAP_GLOBAL, update_map_=true]
    G --> H[MapUpdate 线程 → LoadMapByPose]
    H --> I[只加载机器人50m内图元, 合并成 global_map_]
    I --> J[MapRegister 初始化 GICP → INITED]
    J --> K[跟踪: submap 对 global_map_ 配准, 按位置增删图元]
```

---

## 5. 验证结果（ROS2 / livox.yaml / 25m 切分）

| 项目 | 结果 |
|---|---|
| 编译 | `colcon build --packages-select lio_slam` 通过 |
| 切块 | 输入 99116 点 → 34 个图元、99072 点（丢弃 17 个 <10 点噪声格） |
| 输出 | `<local_map_dir>/MAP_GLOBAL/META{0..33}/data.pcd + data.yaml` |
| 启动扫描 | `slam_mode:=slam` 启动即解析 34 tile → `receive meta maps` → `NOT_INIT` |
| 按位置加载 | 发布 `/initialpose`(63,-40) → `LoadMapByPose` 仅加载 50m 内 11 个 tile，合并 `global map point size 38996` |

---

## 6. 运行方式

```bash
# 1) 生成图元(每次换地图后重跑)
/install 下: map_split <map>.pcd <out_dir> MAP_GLOBAL 25.0

# 2) 定位运行
ros2 launch lio_slam mapping.launch.py \
    config_path:=<...>/config/livox.yaml slam_mode:=slam

# 3) rviz 用 2D Pose Estimate(/initialpose) 给初值 → INITING → INITED
```

---

## 7. 备注与后续

- 完整定位需回放传感器数据（如 `ros2 bag play go2_bag/...`）+ rviz 初始位姿；
  初始化/跟踪（GICP/Ceres、`T_OtoM` 等）为原有逻辑，本次未改动。
- 图元尺寸需与 `LoadMapByPose` 的距离阈值匹配：加载半径 50m、最近候选 25√2 m、
  过期 15s，默认 25m 较合适。
- 若要多叶子/多楼层，可给不同 `leaf_id`（对应不同 identity 分组），并在
  `/initialpose` 之外提供带 map_id 的初始位姿话题（类似 ROS1 `slam_pose`）。
- 若希望整图已足够小、无需分块，可继续使用 `Localizer::SetMaps(pcd_path)` 路径
  （需接线调用，当前未启用）。
