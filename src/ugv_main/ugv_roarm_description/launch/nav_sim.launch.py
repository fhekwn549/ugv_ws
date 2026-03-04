"""
Fake Odom + Fake Scan + Nav2 Simulation Launch File

Gazebo 없이 fake_odom_node와 fake_scan_node를 사용하여
Nav2 자율주행 알고리즘만 테스트하는 경량 시뮬레이션 환경입니다.

Usage:
  ros2 launch ugv_roarm_description nav_sim.launch.py
  ros2 launch ugv_roarm_description nav_sim.launch.py spawn_x:=-0.81 spawn_y:=-2.70
  ros2 launch ugv_roarm_description nav_sim.launch.py use_bridge:=true
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('ugv_roarm_description')

    # Default paths
    urdf_file = os.path.join(pkg_dir, 'urdf', 'ugv_roarm.xacro')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params_fake.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'nav_sim_view.rviz')

    # Try to find ugv_nav map; fall back to source tree path
    try:
        ugv_nav_dir = get_package_share_directory('ugv_nav')
        default_map = os.path.join(ugv_nav_dir, 'maps', 'map.yaml')
    except Exception:
        src_pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        default_map = os.path.join(
            os.path.dirname(src_pkg_dir), 'ugv_nav', 'maps', 'map.yaml')

    # Bridge launch (optional)
    try:
        bridge_dir = get_package_share_directory('ugv_bridge')
        bridge_launch = os.path.join(bridge_dir, 'launch', 'bridge.launch.py')
    except Exception:
        bridge_launch = None

    # Launch configurations
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    use_bridge = LaunchConfiguration('use_bridge')

    # URDF with Gazebo plugins excluded
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' use_gazebo:=false']),
        value_type=str,
    )

    # Nav2 lifecycle nodes (no AMCL — static map→odom TF)
    lifecycle_nodes = [
        'map_server',
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
    ]

    ld = LaunchDescription()

    # ----- Arguments -----
    ld.add_action(DeclareLaunchArgument(
        'spawn_x', default_value='0.0',
        description='Robot initial X position on map'))
    ld.add_action(DeclareLaunchArgument(
        'spawn_y', default_value='0.0',
        description='Robot initial Y position on map'))
    ld.add_action(DeclareLaunchArgument(
        'spawn_yaw', default_value='0.0',
        description='Robot initial yaw (radians)'))
    ld.add_action(DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Path to map.yaml for Nav2'))
    ld.add_action(DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2'))
    ld.add_action(DeclareLaunchArgument(
        'use_bridge', default_value='false',
        description='Launch ugv_bridge (RabbitMQ required)'))

    # ===== 1. Robot State Publisher (URDF without Gazebo plugins) =====
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    ))

    # ===== 2. Fake Odom Node (also publishes joint_states for TF sync) =====
    ld.add_action(Node(
        package='ugv_roarm_description',
        executable='fake_odom_node.py',
        name='fake_odom_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'initial_x': spawn_x,
            'initial_y': spawn_y,
            'initial_yaw': spawn_yaw,
            'update_rate': 20.0,
        }],
    ))

    # ===== 4. Fake Scan Node =====
    ld.add_action(Node(
        package='ugv_roarm_description',
        executable='fake_scan_node.py',
        name='fake_scan_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'scan_rate': 10.0,
            'num_samples': 360,
            'range_min': 0.16,
            'range_max': 3.5,
            'noise_stddev': 0.01,
        }],
    ))

    # map → odom TF는 fake_odom_node에서 dynamic으로 발행 (costmap TF 호환성)

    # ===== 5. Map Server =====
    ld.add_action(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params,
            {'yaml_filename': map_yaml},
        ],
    ))

    # ===== 7. Nav2 Stack =====
    ld.add_action(Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    ))
    ld.add_action(Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
    ))
    ld.add_action(Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
    ))
    ld.add_action(Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
    ))

    # ===== 8. Nav2 Lifecycle Manager =====
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': lifecycle_nodes,
            'bond_timeout': 10.0,
        }],
    ))

    # ===== 9. RViz2 =====
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz),
        additional_env={
            'DISPLAY': ':0',
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'MESA_GL_VERSION_OVERRIDE': '3.3',
        },
    ))

    # ===== 10. ugv_bridge (optional: web dashboard backend) =====
    if bridge_launch:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch),
            condition=IfCondition(use_bridge),
        ))

    return ld
