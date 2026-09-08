# PGO 结果保存(整图 + 关键帧 + 位姿 + 图元) 与回环修复

> 模块：`src/lio_slam/src/PGO/PosegraphOptimization.{cc,hh}`、`src/app/pgo_node.cc`
> 节点：`pgo_node`（启动：`ros2 launch lio_slam pgo.launch.py`）

---

## 1. 功能概述

`pgo_node` 在做位姿图优化(ISAM2)与回环闭合后，可以把结果一次保存：

1. **整张优化地图** pcd（各关键帧按优化位姿拼装 + 体素降采样）
2. **关键帧点云**（地图系 pcd）
3. **关键帧位姿** `keyframe_poses.txt`
4. **切图元(META)** —— 复用 `map_split` 的切图元逻辑，默认 50×50m，
   目录结构兼容 `Localizer::LoadMetaMapsFromDir`（可拿去定位）

触发方式：
- **srv 手动保存（推荐）**：`/pgo_node/save_map`，请求里指定地图名称
- 空闲自动保存（可选，默认关）

---

## 2. 保存服务 srv

自定义接口 `srv/SaveMap.srv`：

```text
# 保存 PGO 优化后的建图结果(整图 pcd + 关键帧 + 位姿 txt + 切图元)
# 保存位置: 参数 map_save_dir/<map_name>/ 目录下
string map_name  # 地图名称(不含扩展名/路径分隔符), 如 "scene_20260908"
---
bool success     # 是否保存成功
string message   # 保存目录或错误信息
```

### 调用方法
```bash
ros2 service call /pgo_node/save_map lio_slam/srv/SaveMap "{map_name: 'scene1'}"
```

实现要点：
- `pgo_node` 构造时创建 service（ROS2 only，`#if ROS_AVAILABLE == 2`）
- 地图名做安全过滤（只保留字母数字 `_-.`），防止路径逃逸
- 根目录 `map_save_dir`（默认 `/home/li/ros2_ws/maps`），每次调用存到
  `map_save_dir/<map_name>/` 独立目录

### 构建接线（ROS2）
- `package.xml`：`rosidl_default_generators` / `rosidl_default_runtime` /
  `<member_of_group>rosidl_interface_packages</member_of_group>`（ROS2 条件）
- `cmake/ROS2.cmake`：
  ```cmake
  find_package(rosidl_default_generators REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME} "srv/SaveMap.srv")
  # pgo_node 链接生成的 typesupport
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(pgo_node "${cpp_typesupport_target}")
  ```

---

## 3. 保存内容与目录结构

调用 `SaveMapByName("scene1")` 后（`map_save_dir = /home/li/ros2_ws/maps`）：

```
/home/li/ros2_ws/maps/scene1/
├── scene1.pcd                       # 优化后整图(体素降采样 map_save_voxel)
├── keyframes/
│   ├── keyframe_0.pcd ...           # 各关键帧点云(按优化位姿转到 map 系)
│   └── keyframe_poses.txt           # index tx ty tz qx qy qz qw
└── tiles/
    └── MAP_GLOBAL/
        └── META0, META1, .../       # 50x50 图元(data.pcd + data.yaml)
```

### keyframe_poses.txt 格式
```text
# keyframe poses (optimized, map frame)
# index tx ty tz qx qy qz qw
0 -0.000000 -0.000000 0.000000 0.000000 0.000000 0.000000 1.000000
...
```

---

## 4. 切图元（复用 map_split 逻辑）

把切块逻辑抽成共享函数，供 CLI 工具与 PGO 保存共同使用：
- `src/utils/global_map_split.{hh,cc}`：`SplitGlobalMapToTiles(cloud, out_dir, leaf_id, tile_size, min_pts)`
- `src/app/map_split.cc`（命令行工具）保留不变
- 图元保存为**图元局部系 + T**（点减图元中心 cx,cy，z 全局；
  `data.yaml` 记 `T=[cx,cy,0,0,0,0,1]`）——保证 `Localizer::LoadMapByPose`
  能按图元中心做距离判定动态加载
- PGO 保存时把降采样后的整图交给该函数，默认 50×50m

---

## 5. 保存逻辑（`SaveMapByName`）

1. 地图名净化
2. 锁内(`mKF`)快照：关键帧点云(shared_ptr)+优化位姿 → 锁外处理
3. 逐关键帧：`TransformCloud` 到 map 系 → 存 `keyframes/keyframe_<i>.pcd`
   → 累加进整图 `map_out`；同时写位姿行到 `keyframe_poses.txt`
4. 整图体素降采样(默认 0.1m) → 存 `<name>.pcd`
5. `SplitGlobalMapToTiles` 切 50×50 图元 → `tiles/<leaf>/META*/`

---

## 6. 参数（`pgo.launch.py` parameters）

| 参数 | 默认 | 说明 |
|---|---|---|
| `map_save_enable` | False | 空闲自动保存开关（推荐 False，用 srv） |
| `map_save_idle_sec` | 10.0 | 输入停止多久后自动保存（仅 enable 时） |
| `map_save_voxel` | 0.1 | 保存时体素降采样(m)，>0 |
| `map_save_dir` | /home/li/ros2_ws/maps | 保存根目录 |
| `map_save_name` | pgo_optimized_map | 自动保存默认名称（仅 enable 时） |
| `map_save_keyframes` | true | 保存关键帧点云 + 位姿 txt |
| `map_save_tile_enable` | true | 保存时切图元 |
| `map_save_tile_size` | 50.0 | 图元边长(m) |
| `map_save_leaf` | MAP_GLOBAL | 图元叶子地图 identity |

---

## 7. 回环崩溃修复（一并记录）

现象：`pgo_node` 在首次回环（如 186↔0）后抛
`gtsam::IndeterminantLinearSystemException` 崩溃。

根因：`runLoopConstraint()` 里判断**反向**——`doICPVirtualRelative` 在 ICP
失败时返回恒等位姿作哨兵，而代码"等于恒等才加"把**失败的 ICP 也当成恒等回环
约束**硬塞进因子图，与里程计链冲突 → ISAM2 线性系统奇异。

修复：改为 **ICP 成功(相对位姿非恒等)才加回环边**，失败则打 WARN 跳过：
```cpp
if (!relativePose3.equals(gtsam::Pose3::Identity())) { /* add */ }
else { LOG_WARN("Skip loop edge: ICP failed ..."); }
```

---

## 8. 产物用于定位

保存后如需用这份优化地图做定位：
1. 把 `config/livox.yaml` 的 localizer `local_map_dir` 指向
   `/home/li/ros2_ws/maps/<name>/tiles`
2. `map_identity: MAP_GLOBAL`
3. `ros2 launch lio_slam mapping.launch.py slam_mode:=slam` + rviz 初始位姿

---

## 9. 涉及文件清单

| 文件 | 说明 |
|---|---|
| `src/PGO/PosegraphOptimization.{cc,hh}` | 保存/回环逻辑、srv service |
| `srv/SaveMap.srv` | 自定义保存服务定义 |
| `src/utils/global_map_split.{cc,hh}` | 切图元共享函数 |
| `cmake/ROS2.cmake` | srv 接口生成与链接 |
| `package.xml` | rosidl 生成依赖 |
| `launch/ROS2/pgo.launch.py` | pgo 参数与 prefix(gdb) 支持 |
| `src/app/map_split.cc` | 命令行切块工具(未改动) |
