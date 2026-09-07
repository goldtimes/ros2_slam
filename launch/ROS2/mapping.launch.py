# ============================================================
#  lio_slam slam_node 的 ROS2 launch 文件
# 用法:
#   ros2 launch lio_slam mapping.launch.py config_path:=<yaml> slam_mode:=mapping
#   ros2 launch lio_slam mapping.launch.py \
#       config_path:=/path/to/config/velodyne.yaml slam_mode:=odometer
# 说明:
#   节点自身的输出话题采用私有名,默认展开为 /slam_node/lie_slam/xxx;
#   若需要与 ROS1 的 /lie_slam/xxx 保持一致,可自行在 Node 的 remappings 中指定。
# ============================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lio_slam')

    default_config = os.path.join(pkg_share, 'config', 'livox.yaml')

    config_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config,
        description='路径:lio_slam 的 yaml 算法配置文件',
    )
    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode',
        default_value='mapping',
        description='模式:mapping / slam(定位) / odometer',
    )

    slam_node = Node(
        package='lio_slam',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'slam_mode': LaunchConfiguration('slam_mode'),
        }],
        # 示例:如需兼容 ROS1 的绝对话题名,可取消下面注释
        # remappings=[
        #     ('~/lie_slam/lio_odom', '/lie_slam/lio_odom'),
        #     ('~/lie_slam/body', '/lie_slam/body'),
        # ],
    )

    return LaunchDescription([
        config_arg,
        slam_mode_arg,
        slam_node,
    ])
