"""
Real Robot SLAM Launch File (Cartographer)

RPi 또는 WSL에서 실행 가능.
RPi에서 실행 시: rasp_bringup과 같은 머신에서 토픽 직접 수신.
WSL에서 실행 시: CycloneDDS를 통해 RPi 토픽 수신 + RViz 시각화.

Ctrl+C 종료 시 ~/maps/ 에 pbstream + pgm/yaml 자동 저장.

Usage:
  ros2 launch ugv_roarm_description slam_real.launch.py
  ros2 launch ugv_roarm_description slam_real.launch.py use_rviz:=true  # WSL with RViz
"""

import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('ugv_roarm_description')
    cartographer_config_dir = os.path.join(pkg_dir, 'config')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'slam_view.rviz')

    use_rviz = LaunchConfiguration('use_rviz')

    # Auto-save map on shutdown
    map_dir = os.path.expanduser('~/maps')
    os.makedirs(map_dir, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    map_path = os.path.join(map_dir, f'map_{timestamp}')
    pbstream_path = f'{map_path}.pbstream'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz (set true on WSL, false on RPi)'),

        # Cartographer SLAM node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', 'cartographer_mapping_2d.lua',
            ],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom'),
            ],
        ),

        # Cartographer occupancy grid node (publishes /map)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'resolution': 0.05,
                'publish_period_sec': 1.0,
            }],
        ),

        # RViz2 (optional, for WSL only)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
            condition=IfCondition(use_rviz),
        ),

        # Map auto-saver: saves pbstream + pgm/yaml on Ctrl+C
        ExecuteProcess(
            cmd=['bash', '-c',
                 f"cleanup() {{ "
                 f"echo '[map_saver] Finishing trajectory and saving pbstream...'; "
                 f"ros2 service call /finish_trajectory "
                 f"cartographer_ros_msgs/srv/FinishTrajectory "
                 f"'{{trajectory_id: 0}}' 2>/dev/null; "
                 f"sleep 1; "
                 f"ros2 service call /write_state "
                 f"cartographer_ros_msgs/srv/WriteState "
                 f"'{{filename: \"{pbstream_path}\", "
                 f"include_unfinished_submaps: true}}' 2>/dev/null; "
                 f"sleep 1; "
                 f"echo '[map_saver] Saving occupancy grid map...'; "
                 f"ros2 run nav2_map_server map_saver_cli -f {map_path} 2>/dev/null; "
                 f"echo '[map_saver] Done: {map_path}.pgm/.yaml + {pbstream_path}'; "
                 f"exit 0; "
                 f"}}; trap cleanup INT TERM; "
                 f"while true; do sleep 1; done"],
            output='screen',
        ),
    ])
