import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('ugv_roarm_description')

    model = LaunchConfiguration('model')
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_jsp = LaunchConfiguration('use_jsp')

    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'view_ugv_roarm.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='ugv_roarm',
            description='Xacro model name (ugv_roarm or rasp_roarm)'),

        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Use joint_state_publisher_gui'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2'),

        DeclareLaunchArgument(
            'use_jsp',
            default_value='true',
            description='Launch joint_state_publisher (disable when using teleop_all.py rviz mode)'),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', os.path.join(pkg_dir, 'urdf'), '/', model, '.xacro']),
                    value_type=str)
            }]
        ),

        # Joint State Publisher GUI (use_gui:=true AND use_jsp:=true)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(PythonExpression([
                "'", use_gui, "' == 'true' and '", use_jsp, "' == 'true'"
            ]))
        ),

        # Joint State Publisher (use_gui:=false AND use_jsp:=true)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(PythonExpression([
                "'", use_gui, "' == 'false' and '", use_jsp, "' == 'true'"
            ]))
        ),

        # RViz2 (software rendering for WSL2 compatibility)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(use_rviz),
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}
        ),
    ])
