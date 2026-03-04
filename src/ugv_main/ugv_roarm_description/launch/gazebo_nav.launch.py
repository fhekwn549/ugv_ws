"""
Gazebo Digital Twin + Nav2 Autonomous Navigation Launch File

SLAM으로 만든 실제 맵을 Gazebo 월드로 변환한 환경에서
Nav2 자율주행을 테스트합니다.

시뮬레이션에서는 AMCL 대신 static map→odom TF를 사용합니다.
Gazebo에서 로봇 위치가 정확하므로 별도의 위치 추정이 필요 없습니다.

사전 준비:
  python3 scripts/map_to_gazebo_world.py  (최초 1회)

Usage:
  ros2 launch ugv_roarm_description gazebo_nav.launch.py
  ros2 launch ugv_roarm_description gazebo_nav.launch.py gui:=false use_rviz:=true
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('ugv_roarm_description')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # Default paths
    world_file = os.path.join(pkg_dir, 'worlds', 'digital_twin.world')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params_sim.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'nav_sim_view.rviz')

    # Try to find ugv_nav map; fall back to source tree path
    try:
        ugv_nav_dir = get_package_share_directory('ugv_nav')
        default_map = os.path.join(ugv_nav_dir, 'maps', 'map.yaml')
    except Exception:
        src_pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        default_map = os.path.join(
            os.path.dirname(src_pkg_dir), 'ugv_nav', 'maps', 'map.yaml')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    map_yaml = LaunchConfiguration('map')
    world = LaunchConfiguration('world')

    # GAZEBO_MODEL_PATH: include map_walls model directory
    map_walls_parent = os.path.join(pkg_dir, 'worlds')
    ugv_desc_models = os.path.dirname(get_package_share_directory('ugv_description'))
    roarm_desc_models = os.path.dirname(pkg_dir)
    gazebo_model_paths = [
        map_walls_parent,
        ugv_desc_models,
        roarm_desc_models,
        '/usr/share/gazebo-11/models',
    ]
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    if existing:
        gazebo_model_paths.append(existing)
    gazebo_model_path = ':'.join(gazebo_model_paths)

    # Nav2 lifecycle nodes (no AMCL — using static map→odom TF in sim)
    lifecycle_nodes = [
        'map_server',
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
    ]

    return LaunchDescription([
        # ----- Environment -----
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11'),

        # ----- Arguments -----
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock'),

        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Launch Gazebo GUI (gzclient)'),

        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz'),

        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Path to map.yaml for Nav2'),

        DeclareLaunchArgument(
            'world', default_value=world_file,
            description='Gazebo world file'),

        DeclareLaunchArgument(
            'spawn_x', default_value='-0.81',
            description='Robot spawn X (free space on map)'),

        DeclareLaunchArgument(
            'spawn_y', default_value='-2.70',
            description='Robot spawn Y (free space on map)'),

        # ===== 1. Gazebo (reuse existing gazebo.launch.py) =====
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'world': world,
                'gui': gui,
                'pause': 'false',
                'spawn_x': LaunchConfiguration('spawn_x'),
                'spawn_y': LaunchConfiguration('spawn_y'),
            }.items(),
        ),

        # ===== 2. Sim Pose Bridge (RViz 2D Pose Estimate → Gazebo teleport) =====
        Node(
            package='ugv_roarm_description',
            executable='sim_pose_bridge.py',
            name='sim_pose_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ===== 3. Static map → odom TF =====
        # Gazebo diff_drive reports odom in world coordinates (not relative
        # to spawn), so map and odom frames are identical.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ===== 3. Map Server =====
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                nav2_params,
                {'yaml_filename': map_yaml},
            ],
        ),

        # ===== 4. Nav2 Stack =====
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params],
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params],
        ),

        # ===== 5. Nav2 Lifecycle Manager =====
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes,
                'bond_timeout': 60.0,
            }],
        ),

        # ===== 6. RViz =====
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            additional_env={
                'DISPLAY': ':0',
                'LIBGL_ALWAYS_SOFTWARE': '1',
                'MESA_GL_VERSION_OVERRIDE': '3.3',
            },
        ),
    ])
