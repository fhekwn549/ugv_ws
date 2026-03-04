"""
WSL Remote View Launch File

RPi에서 rasp_bringup.launch.py 실행 후,
WSL에서 이 launch 파일을 실행하면 RViz가 열립니다.
CycloneDDS를 통해 RPi의 토픽(/joint_states, /scan, /tf 등)이 자동으로 수신됩니다.

Usage:
  ros2 launch ugv_roarm_description remote_view.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('ugv_roarm_description')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'remote_view.rviz')

    return LaunchDescription([
        # RViz2 (software rendering for WSL2)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}
        ),
    ])
