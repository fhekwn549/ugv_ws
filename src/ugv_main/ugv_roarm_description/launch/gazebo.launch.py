import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('ugv_roarm_description')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # Paths
    gazebo_xacro = os.path.join(pkg_dir, 'urdf', 'ugv_roarm.gazebo.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'ugv_roarm.world')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    robot_description = ParameterValue(
        Command(['xacro ', gazebo_xacro]),
        value_type=str
    )

    pause = LaunchConfiguration('pause')
    gui = LaunchConfiguration('gui')

    # Gazebo converts URDF package:// to model:// URIs for meshes.
    # Add package share parent dirs so model://pkg_name/meshes/... resolves correctly.
    ugv_desc_models = os.path.dirname(get_package_share_directory('ugv_description'))
    roarm_desc_models = os.path.dirname(pkg_dir)
    gazebo_model_paths = [
        ugv_desc_models,
        roarm_desc_models,
        '/usr/share/gazebo-11/models',
    ]
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    if existing:
        gazebo_model_paths.append(existing)
    gazebo_model_path = ':'.join(gazebo_model_paths)

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'),

        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Gazebo world file'),

        DeclareLaunchArgument(
            'pause',
            default_value='false',
            description='Start Gazebo paused'),

        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch Gazebo GUI (gzclient)'),

        # Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={
                'world': world,
                'pause': pause,
            }.items(),
        ),

        # Gazebo client (DISPLAY set only for this process, not gzserver)
        ExecuteProcess(
            cmd=['gzclient', '--gui-client-plugin', 'libgazebo_ros_eol_gui.so'],
            output='screen',
            condition=IfCondition(gui),
            additional_env={'DISPLAY': ':0', 'LIBGL_ALWAYS_SOFTWARE': '1'},
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        # Spawn robot from robot_description topic
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'ugv_roarm',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1',
            ],
        ),

        # ros2_control controller spawners (arm joints)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller'],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller'],
            output='screen',
        ),
    ])
