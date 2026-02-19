cmake_minimum_required(VERSION 3.8)
project(ugv_roarm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz meshes worlds config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/teleop_all.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
