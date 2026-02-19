import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('ugv_roarm_description')
    # URDF via xacro
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg_dir, 'urdf', 'rasp_roarm.xacro')]),
        value_type=str)

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 2. UGV Bringup — ESP32 sensor feedback (IMU, encoder, voltage)
    ugv_bringup_node = Node(
        package='ugv_bringup',
        executable='ugv_bringup',
        name='ugv_bringup',
        output='screen',
    )

    # 3. UGV Driver — /cmd_vel -> ESP32 motor commands
    ugv_driver_node = Node(
        package='ugv_bringup',
        executable='ugv_driver',
        name='ugv_driver',
        output='screen',
    )

    # 4. Base Node — odometry calculation + odom -> base_footprint TF
    base_node = Node(
        package='ugv_base_node',
        executable='base_node',
        name='base_node',
        output='screen',
        parameters=[{
            'pub_odom_tf': True,
            'wheel_separation': 0.076,
        }]
    )

    # 5. RoArm-M2 Driver — /arm_controller/joint_trajectory -> ESP32 serial
    roarm_driver_node = Node(
        package='ugv_bringup',
        executable='roarm_driver',
        name='roarm_driver',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
        }],
    )

    # 6. LDLidar (STL-19P / LD19) — /scan laser data
    ldlidar_pkg_dir = get_package_share_directory('ldlidar_ros2')
    ldlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ldlidar_pkg_dir, 'launch', 'ld19.launch.py')
        ),
        launch_arguments={'serial_port': '/dev/ttyUSB1'}.items(),
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ugv_bringup_node,
        ugv_driver_node,
        base_node,
        roarm_driver_node,
        ldlidar_launch,
    ])
