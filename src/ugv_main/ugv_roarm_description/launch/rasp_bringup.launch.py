import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
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

    # 2. UGV Driver (C++) — 제어 + 센서 피드백 통합
    # (기존 ugv_bringup + ugv_driver를 하나의 C++ 노드로 통합)
    ugv_driver_node = Node(
        package='ugv_cpp_nodes',
        executable='ugv_driver_node',
        name='ugv_driver',
        output='screen',
    )

    # 4. rf2o_laser_odometry — LiDAR scan-based odometry + odom -> base_footprint TF
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'laser_frame_id': 'base_laser',
            'init_pose_from_topic': '',
            'freq': 20.0,
        }],
    )

    # 5. RoArm-M2 Driver — /arm_controller/joint_trajectory -> ESP32 serial (C++)
    roarm_driver_node = Node(
        package='ugv_cpp_nodes',
        executable='roarm_driver_node',
        name='roarm_driver',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
        }],
    )

    # 6. LDLidar (STL-19P / LD19) — /scan laser data
    # Node launched directly (not via ld19.launch.py) to avoid its
    # conflicting base_link→base_laser static TF
    ldlidar_node = Node(
        package='ldlidar_ros2',
        executable='ldlidar_ros2_node',
        name='ldlidar_publisher_ld19',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'laser_scan_topic_name': 'scan',
            'point_cloud_2d_topic_name': 'pointcloud2d',
            'frame_id': 'base_laser',
            'port_name': '/dev/ttyUSB1',
            'serial_baudrate': 230400,
            'laser_scan_dir': True,
            'enable_angle_crop_func': True,
            'angle_crop_min': 210.0,
            'angle_crop_max': 329.0,
            'range_min': 0.02,
            'range_max': 12.0,
        }],
    )

    # 7. UGV Bridge — STOMP + REST API bridge for web dashboard
    bridge_params = os.path.join(
        get_package_share_directory('ugv_bridge'), 'config', 'bridge_params.yaml')
    bridge_node = Node(
        package='ugv_bridge',
        executable='bridge_node',
        name='ugv_bridge',
        output='screen',
        parameters=[bridge_params],
    )

    # 8. Static TF: base_lidar_link → base_laser
    # LiDAR publishes with frame_id 'base_laser', URDF has 'base_lidar_link'
    # yaw=π/2: LiDAR physical mounting is 90° rotated from URDF assumption
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_lidar_to_base_laser',
        arguments=['0', '0', '0', '1.5708', '0', '0',
                   'base_lidar_link', 'base_laser']
    )

    # CycloneDDS 사용 (FastDDS 2.6.x/Humble는 대형 메시지에서
    # "sequence size exceeds remaining buffer" 문제 있음. Iron+ 에서 개선 예정)
    # serdata 경고는 rcutils stderr로 출력되어 환경변수로 억제 불가.
    # 필터링 실행: ugv-launch alias 참고 (grep -v serdata)
    use_cyclonedds = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    return LaunchDescription([
        use_cyclonedds,
        robot_state_publisher_node,
        ugv_driver_node,
        rf2o_node,
        roarm_driver_node,
        ldlidar_node,
        bridge_node,
        lidar_static_tf,
    ])
