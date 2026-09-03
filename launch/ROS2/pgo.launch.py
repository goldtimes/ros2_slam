# ============================================================
#  lio_slam pgo_node 的 ROS2 launch 文件
# 用法:
#   ros2 launch lio_slam pgo.launch.py
# 说明:
#   pgo_node 订阅 slam_node 输出的里程计/点云话题(私有名 remap 到 slam_node):
#     ~/lidar_odom             <- /slam_node/lie_slam/lio_odom
#     ~/lidar_registered_body  <- /slam_node/lie_slam/body
#   输出(私有名,默认 /pgo_node/aft_pgo_*):
#     ~/aft_pgo_path  ~/aft_pgo_odom  ~/aft_pgo_map
# ============================================================
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pgo_node = Node(
        package='lio_slam',
        executable='pgo_node',
        name='pgo_node',
        output='screen',
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
        }],
        remappings=[
            ('~/lidar_odom', '/slam_node/lie_slam/lio_odom'),
            ('~/lidar_registered_body', '/slam_node/lie_slam/body'),
        ],
    )

    return LaunchDescription([
        pgo_node,
    ])
