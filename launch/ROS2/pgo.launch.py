# ============================================================
#  lio_slam pgo_node 的 ROS2 launch 文件
# 用法:
#   ros2 launch lio_slam pgo.launch.py
#   # 崩溃时打印堆栈(需先 colcon build 安装本文件):
#   ros2 launch lio_slam pgo.launch.py \
#       prefix='gdb -batch -ex run -ex bt --args'
# 说明:
#   pgo_node 订阅 slam_node 输出的里程计/点云话题(私有名 remap 到 slam_node):
#     ~/lidar_odom             <- /slam_node/lie_slam/lio_odom
#     ~/lidar_registered_body  <- /slam_node/lie_slam/body
#   输出(私有名,默认 /pgo_node/aft_pgo_*):
#     ~/aft_pgo_path  ~/aft_pgo_odom  ~/aft_pgo_map
# ============================================================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='节点启动前缀,用于调试。例如崩溃时打印堆栈: gdb -batch -ex run -ex bt --args',
    )
    pgo_node = Node(
        package='lio_slam',
        executable='pgo_node',
        name='pgo_node',
        output='screen',
        prefix=LaunchConfiguration('prefix'),
        parameters=[{
            'keyframe_meter_gap': 1.0,
            'keyframe_deg_gap': 30.0,
            'historyKeyframeSearchRadius': 10.0,
            'historyKeyframeSearchTimeDiff': 30.0,
            'historyKeyframeSearchNum': 25,
            'loopNoise': 0.5,
            'graphUpdateTimes': 2,
            'loopFitnessScoreThreshold': 0.3,
            'use_gps': False,
            'speedFactor': 1.0,
            'loopClosureFrequency': 2.0,
            'graphUpdateFrequency': 1.0,
            'vizmapFrequency': 0.1,
            # 地图保存(默认用 srv 手动触发: ros2 service call /pgo_node/save_map lio_slam/srv/SaveMap "{map_name: 'xxx'}")
            'map_save_enable': False,        # 空闲自动保存开关(推荐 False, 用 srv)
            'map_save_idle_sec': 10.0,       # 输入停止多久后自动保存(秒, 仅 enable 时有效)
            'map_save_voxel': 0.1,           # 保存时体素降采样(m), >0
            'map_save_dir': '/home/li/ros2_ws/maps',   # 保存根目录
            'map_save_name': 'pgo_optimized_map',      # 自动保存默认名称(仅 enable 时有效)
            # 保存关键帧(地图系 pcd)与关键帧位姿 txt
            'map_save_keyframes': True,
            # 保存时把地图切成图元(META), 供 Localizer 动态加载定位
            'map_save_tile_enable': True,
            'map_save_tile_size': 50.0,      # 图元边长(m)
            'map_save_leaf': 'MAP_GLOBAL',   # 叶子地图 identity
        }],
        remappings=[
            ('~/lidar_odom', '/slam_node/lie_slam/lio_odom'),
            ('~/lidar_registered_body', '/slam_node/lie_slam/body'),
        ],
    )

    return LaunchDescription([
        prefix_arg,
        pgo_node,
    ])
