import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ugv_bridge')
    default_params = os.path.join(pkg_dir, 'config', 'bridge_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to bridge parameters YAML',
        ),
        DeclareLaunchArgument(
            'robot_ids',
            default_value='ugv01',
            description='Comma-separated robot IDs to manage',
        ),
        Node(
            package='ugv_bridge',
            executable='bridge_node',
            name='bridge_node',
            parameters=[
                LaunchConfiguration('params_file'),
                {'robot_ids': LaunchConfiguration('robot_ids')},
            ],
            output='screen',
        ),
    ])
