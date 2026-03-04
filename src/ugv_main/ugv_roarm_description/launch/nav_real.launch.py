"""
Real Robot Navigation Launch File (Cartographer Localization)

Cartographer .pbstream을 로드하여 자동 초기 위치 인식 + Nav2 자율주행을 실행합니다.
AMCL과 달리 수동 initial pose 설정이 필요 없습니다.

RPi에서 rasp_bringup.launch.py 실행 후,
WSL에서 이 launch 파일을 실행하면 Nav2 스택이 뜹니다.
CycloneDDS를 통해 RPi의 토픽이 자동으로 수신됩니다.
시각화는 웹 대시보드(ugv_dashboard)를 사용합니다.

Usage:
  ros2 launch ugv_roarm_description nav_real.launch.py pbstream:=~/maps/lab_map.pbstream
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('ugv_roarm_description')
    cartographer_dir = get_package_share_directory('cartographer')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    cartographer_config_dir = os.path.join(cartographer_dir, 'config')

    pbstream_path = LaunchConfiguration('pbstream')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    lifecycle_nodes = [
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'pbstream',
            default_value=os.path.expanduser('~/maps/lab_map.pbstream'),
            description='Full path to Cartographer .pbstream file'),

        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Resolution of the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='OccupancyGrid publishing period'),

        # --- Cartographer: Localization (replaces map_server + amcl) ---
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', 'localization_2d.lua',
                '-load_state_filename', pbstream_path,
            ],
        ),

        # --- Cartographer: OccupancyGrid publisher (provides /map) ---
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-resolution', resolution,
                '-publish_period_sec', publish_period_sec,
            ],
        ),

        # --- Nav2: Planner Server ---
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params],
        ),

        # --- Nav2: Controller Server ---
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
        ),

        # --- Nav2: Behavior Server ---
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params],
        ),

        # --- Nav2: BT Navigator ---
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params],
        ),

        # velocity_smoother disabled — motor deadband too high for gradual ramp-up

        # --- Nav2: Lifecycle Manager ---
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),

    ])
