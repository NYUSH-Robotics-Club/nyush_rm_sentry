"""Foxglove bridge and Odin mode supervisor; Odin itself starts from the UI."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='RMUL2026'),
        DeclareLaunchArgument('map_dir', default_value=str(Path.home() / '.ros' / 'odin_maps')),
        DeclareLaunchArgument('grid_resolution', default_value='0.05'),
        DeclareLaunchArgument('imu_to_base', default_value='-0.10 0 -0.39 0 0 3.141592653589793'),
        DeclareLaunchArgument('bridge_address', default_value='0.0.0.0'),
        DeclareLaunchArgument('bridge_port', default_value='8765'),
        Node(
            package='rm_nav_bringup', executable='odin_foxglove_control.py',
            name='odin_foxglove_control', output='screen', parameters=[{
                'world': LaunchConfiguration('world'),
                'map_dir': LaunchConfiguration('map_dir'),
                'grid_resolution': LaunchConfiguration('grid_resolution'),
                'imu_to_base': LaunchConfiguration('imu_to_base'),
            }],
        ),
        Node(
            package='foxglove_bridge', executable='foxglove_bridge',
            name='foxglove_bridge', output='screen', parameters=[{
                'address': LaunchConfiguration('bridge_address'),
                'port': ParameterValue(LaunchConfiguration('bridge_port'), value_type=int),
                'capabilities': ['clientPublish', 'connectionGraph'],
                'client_topic_whitelist': ['^/odin_dashboard/command$', '^/goal_pose$'],
                'topic_whitelist': [
                    '^/odin_dashboard/.*$', '^/map$', '^/tf(_static)?$',
                    '^/(local|global)_costmap/.*$', '^/odin1/(odometry|cloud_raw|cloud_slam|path)$',
                    '^/odin1/image/compressed$',
                    '^/(plan|local_plan)$',
                ],
            }],
        ),
    ])
