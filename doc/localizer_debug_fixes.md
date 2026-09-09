# 定位(Localizer)调试与 Bug 修复记录

> 现象来源：ROS2 定位模式（`slam_mode:=slam`）+ PGO 保存的图元地图
> （如 `maps/shizishan_park/tiles`，50×50m 图元）。
> 相关模块：`src/localizer/localizer.{cc,hh}`、`src/lidar_register/superlio_register.cc`、
> `src/PGO/PosegraphOptimization.cc`、`config/livox.yaml`。

---

## 0. 现象时间线

1. `pgo_node` 回环后抛 `gtsam::IndeterminantLinearSystemException` 崩溃（修）
2. 定位模式 `slam_node` 在 `insert [META..]` 后 SIGSEGV（exit -11），gdb 停在 `libpcl_kdtree`（修）
3. 加互斥锁后不崩，但 `imu/lidar data lost`、前端掉帧（修）
4. 初始化后 `UpdateSearch cost 1~2.4s`、`incre_trans 0.3~0.5`，定位"飞/跳"（修）
5. `update source 2.5w→3w` 点持续偏大 —— submap 叠帧膨胀（修）

---

## 1. PGO 回环崩溃（IndeterminantLinearSystemException）

- **现象**：`Loop closure detected between 186 and 0` 后，
  `terminate: gtsam::IndeterminantLinearSystemException`。
- **根因**：`runLoopConstraint()` 判断写反 —— `doICPVirtualRelative()` 在 ICP
  失败时返回 `PoseTrans()`(恒等)作为失败哨兵，而代码是"等于恒等才添加"，
  于是把**失败的 ICP 当成恒等回环约束**塞进因子图，与里程计链冲突 →
  ISAM2 线性系统奇异。
- **修复**（`PosegraphOptimization.cc`）：只有 ICP 成功(相对位姿非恒等)才加边：
  ```cpp
  if (!relativePose3.equals(gtsam::Pose3::Identity())) { /* add */ }
  else LOG_WARN("Skip loop edge: ICP failed ...");
  ```

---

## 2. SIGSEGV：global_map_ / KD 树多线程数据竞争

- **现象**：定位加载/合并图元时（`insert [META5/8]` 之后）段错误，
  gdb 停在 `libpcl_kdtree`。
- **根因**：`LoadMapByPose()`（`MapUpdate` 线程）在**无锁**情况下对
  `global_map_`/`global_map_tree_` 反复执行
  `global_map_tree_->setInputCloud(...)`（原代码在 tile 循环内每插一块重建一次
  KD 树），与其它线程的 `nearestKSearch`/GICP 检索并发 → KD 树"重建 vs 检索"
  竞争。
- **修复**：改为 **RCU（整体换新）**，见第 5 节。

---

## 3. 加锁后 `imu/lidar data lost`（锁拿太久）

- **现象**：上一版用"检索全程持锁"后不再崩，但出现大量
  `imu data lost` / `lidar data lost`、`fps 8`、imu `fps 64`。
- **根因**：`UpdateSearch`/`InitSearch` 在 GICP/Ceres（可达 1~2.4s）期间
  **一直持有 `global_map_mutex_`**，把可视化、地图加载乃至同一执行器里依赖该锁
  的回调长时间阻塞 → 传感器回调被延迟/丢消息。
- **修复**：检索不再持锁，只拿指针快照（第 5 节）。

---

## 4. `UpdateSearch` 空转（get_new_submap_ 未清零）

- **现象**：定位 `INITED` 后日志里 `UpdateSearch cost ~70ms` **无间隙刷屏**，
  里程计/地图加载被拖垮，表现为"加了锁反而更不正常"。
- **根因**：`SetSubmapCloud()` 把 `get_new_submap_` 置 `true`，但
  `MapRegister` 的 `INITED` 分支**用完不清零** → 永远把同一份 submap 反复配准。
- **修复**：`MapRegister` 里消费后立即清零：
  ```cpp
  { lock(state_mutex_); has_new_submap = get_new_submap_; get_new_submap_ = false; }
  if (has_new_submap) UpdateSearch();
  ```
  让配准只在真正来新 submap（关键帧）时执行一次。

---

## 5. 核心修复：RCU 整体换新 + 快照检索（localizer）

把"共享可变地图 + 长锁"改成 **"不可变地图整体换新 + 读者指针快照"**：

- **写**（`LoadMapByPose`/`SetMaps`）：锁外合并/降采样好 `filtered` 后，在临界区
  内只做指针交换，KD 树也用**新建对象**：
  ```cpp
  { lock(global_map_mutex_);
    global_map_ = filtered;                       // 换新点云(不可变)
    PointTree::Ptr t(new PointTree()); t->setInputCloud(filtered);
    global_map_tree_ = t;                         // 换新KD树
    global_map_update_ = true; loaded_map_ = true; }
  ```
  旧对象发布后不再被原地修改 → 持旧快照的读者安全并发使用。
- **读**（`InitSearch`/`UpdateSearch`）：临界区里只拷贝 `{cloud, tree}` 指针快照
  即释放锁，之后整段 GICP/Ceres 不持锁 → 不再阻塞其它线程，也不会崩溃。
- **可视化** `GetGlobalMapSnapshot`：改为共享指针快照（无深拷贝、微秒级）。

好处：既消除 KD 树并发重建崩溃，又不长时间阻塞传感器/地图加载线程。

---

## 6. 定位"飞 / 跳变"（慢 + 大步长）

- **现象**：`UpdateSearch cost 1133~2399 ms`、`incre_trans 0.35~0.53`、
  `icp score ~0.43`，机器人运动期间很久才修正一次、一修正就是大跳 → 轨迹飞。
- **根因**：匹配输入太大 + 迭代多 + 修正步长大：
  - submap 3 万点、目标 ~12 万点；
  - `use_ceres: false` 走了 GICP 100 次迭代；
  - 单次修正无保护，错误匹配也会乘进 `T_OtoM`。
- **修复**：
  1. `UpdateSearch`/`InitSearch` 先对源点云 **voxel 0.5m 降采样**（>5000 点时）；
  2. GICP 最大迭代 100 → 30；
  3. **拒绝大跳变**：`incre_pose` 平移>5m 或转角>0.5rad 时不应用并 WARN；
  4. 配置 `use_ceres: true`（ceres 2 次迭代 + 内部 0.2 降采样，更快更稳）。

---

## 7. submap 叠帧膨胀

- **现象**：`update source` 点数偏大（2.5w~3w），怀疑"很多帧叠加"。
- **结论**：逻辑上 submap 被 `keyframe_size: 15` 限制（最多 ~16 帧），**不是无限
  累积**；但每个关键帧存的是**整帧未降采样**点云，相邻关键帧(0.5m)高度重叠，
  合并时不去重 → submap 又大又叠。
- **修复**（`superlio_register.cc`）：
  1. 关键帧入队前 **0.3m 体素降采样**（`InitMap` 首帧同样）；
  2. 合并后 >8000 点再 **0.3m 去重**一次。

---

## 8. 涉及文件与关键参数

| 文件 | 改动 |
|---|---|
| `src/localizer/localizer.{cc,hh}` | RCU 地图换新、快照检索、get_new_submap_ 清零、源点云降采样、GICP 30 次、大跳变拒绝 |
| `src/lidar_register/superlio_register.cc` | 关键帧/合并 submap 0.3m 降采样去重 |
| `src/PGO/PosegraphOptimization.cc` | 回环边只在 ICP 成功时添加 |
| `config/livox.yaml` | `use_ceres: true` 等 |

`config/livox.yaml` localizer 段可调：`global_map_filter_size`(0.3)、
`match_score_thresh`、`init_icp_score`、`icp_dist_thresh`、`update_search_dist_thresh`、
`use_ceres`；front_end 段 `keyframe_size`(15)/`keyframe_distance`(0.5)。

---

## 9. 验证建议

重测时观察：
- 无崩溃、无 `imu/lidar data lost`（lidar ~10fps、imu ~200fps）；
- `UpdateSearch cost` 降到几十~百毫秒级；`incre_trans` 跟随真实位移、很小；
- `update source` 点数降到几千量级；
- 若 `icp score` 经常 >0.3，优先检查 `/initialpose` 初值（位置/航向）与回放数据
  是否和建图一致，而不是继续放宽阈值。

---

## 10. 概念补充：锁 / 持锁时间 / RCU 快照

### 10.1 为什么要加锁（不锁会崩）
`global_map_`（点云）与 `global_map_tree_`（KD 树）被多线程共享：

| 线程 | 角色 | 操作 |
|---|---|---|
| `MapUpdate` → `LoadMapByPose` | 写 | 拼新图元 → 重建 KD 树(`setInputCloud`) |
| `MapRegister` → `UpdateSearch`/`InitSearch` | 读 | `nearestKSearch`/GICP/Ceres 检索 |
| ROS `Visualize` | 读 | 取 `global_map_` 发 rviz |

若写者在 `setInputCloud`（清空+重建索引）的同时读者在 `nearestKSearch`
遍历同一棵树 → "边改边查" → 段错误（崩在 `libpcl_kdtree`）。

**锁(mutex)的作用 = 互斥**：同一时刻只允许一个线程进入临界区，
读写不再并发 → 崩溃消失。

### 10.2 加锁为什么还会 data lost（锁没问题，是"持锁太久"）
- **锁的正确性**由"临界区互斥"保证；
- **锁的性能/是否会堵**由"持锁时间"决定。

第一版在 `UpdateSearch`/`InitSearch` 开头拿锁、直到整段 GICP/Ceres（可达 1~2.4s）
结束才释放。这把锁其它线程（可视化、地图加载）也要用，于是它们被干等；
ROS 的 IMU/LiDAR 回调处理不及时，程序按"相邻帧时间差"判定 → 打印
`imu/lidar data lost`、掉帧。

> 类比：门锁本身没错，锁 2 秒没问题，锁 2 分钟外面排队的人就进不来。

### 10.3 RCU（整体换新 + 快照）：既不崩也不堵
核心思想：**让读者永远看不到"正在被改的对象"**。

- 写者（`LoadMapByPose`/`SetMaps`）：在门外做好**全新的**点云和**新的** KD 树，
  临界区内只做指针交换（纳秒级）：
  ```cpp
  { std::lock_guard lock(global_map_mutex_);
    global_map_     = filtered;                  // 换到新点云(此后不再改它)
    PointTree::Ptr t(new PointTree());
    t->setInputCloud(filtered);
    global_map_tree_ = t;                        // 换到新KD树
  }
  ```
  旧对象发布后**永远不再被原地修改**。
- 读者（`UpdateSearch`/`InitSearch`）：临界区内只拷贝一份指针快照就立刻放锁，
  之后在**锁外**对快照慢慢检索：
  ```cpp
  PointCloudXYZIPtr gmap; PointTree::Ptr gtree;
  { std::lock_guard lock(global_map_mutex_);
    gmap  = global_map_; gtree = global_map_tree_; }
  // 锁外: GICP/Ceres 随便跑多久都不挡别人
  ```
- 可视化 `GetGlobalMapSnapshot`：同样只共享指针，无深拷贝。

收益：
- **不崩**：写者换新后不再动旧对象，读者手里的旧快照只有自己在用；
- **不堵**：任何人持锁都只是"交换/取指针"，微秒级，没人被长时间挡在门口；
- 代价：地图更新时新分配一块内存并重建一次 KD 树（由写者承担，读者无感）。

### 10.4 三阶段对比

| 阶段 | 做法 | 结果 |
|---|---|---|
| 无锁 | 写者原地重建 KD 树，读者同时检索 | 崩溃（边改边查） |
| 全程持锁 | 锁保证互斥，但配准 1~2s 都锁着 | 不崩，但其它线程被堵 → data lost |
| RCU 换新+快照 | 写者换新对象、读者拿快照，持锁均微秒级 | 既不崩也不堵 |

### 10.5 其它锁（都很短，正常）
- `state_mutex_`：保护状态机标志（`local_state_`、`get_new_submap_` 等），
  临界区都是几条赋值；
- `lidar_mutex_`：保护 `curr_lidar_cloud_` 快照拷贝。
- 另注意：`get_new_submap_` 曾因**未清零**导致 `UpdateSearch` 空转（见 §4），
  那是状态机逻辑 bug，与锁无关。
