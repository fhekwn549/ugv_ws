import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('ugv_roarm_description')

    # ugv_gazebo is not ament-built; locate it via source tree
    # pkg_dir = .../install/ugv_roarm_description/share/ugv_roarm_description
    ws_dir = os.path.normpath(os.path.join(pkg_dir, '..', '..', '..', '..'))
    ugv_gazebo_src = os.path.join(ws_dir, 'src', 'ugv_main', 'ugv_gazebo')

    # Paths
    gazebo_launch = os.path.join(pkg_dir, 'launch', 'gazebo.launch.py')
    slam_params = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'slam_view.rviz')
    obstacle_world = os.path.join(ugv_gazebo_src, 'worlds', 'ugv_world.world')

    # Add ugv_gazebo/models to GAZEBO_MODEL_PATH so model://world resolves
    ugv_gazebo_models = os.path.join(ugv_gazebo_src, 'models')
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    model_paths = [ugv_gazebo_models]
    if existing_model_path:
        model_paths.append(existing_model_path)
    gazebo_model_path = ':'.join(model_paths)

    # Launch arguments
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        # Prepend ugv_gazebo models to GAZEBO_MODEL_PATH
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),

        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch Gazebo GUI'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz with SLAM view'),

        # Gazebo with obstacle world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': obstacle_world,
                'gui': gui,
                'pause': 'false',
            }.items(),
        ),

        # slam_toolbox (online async)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
        ),

        # RViz with SLAM visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_rviz),
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        ),
    ])
