cmake_minimum_required(VERSION 3.8)
project(ugv_cpp_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# --- Serial driver library (pure C++, no ROS2 dependency) ---
add_library(serial_driver STATIC
  src/serial_driver.cpp
)
target_include_directories(serial_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(serial_driver PUBLIC cxx_std_17)

# --- RoArm serial driver library ---
add_library(roarm_serial_driver STATIC
  src/roarm_serial_driver.cpp
)
target_include_directories(roarm_serial_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(roarm_serial_driver PUBLIC serial_driver)
target_compile_features(roarm_serial_driver PUBLIC cxx_std_17)

# --- UGV serial driver library ---
add_library(ugv_serial_driver STATIC
  src/ugv_serial_driver.cpp
)
target_include_directories(ugv_serial_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ugv_serial_driver PUBLIC serial_driver)
target_compile_features(ugv_serial_driver PUBLIC cxx_std_17)

# --- RoArm driver ROS2 node ---
add_executable(roarm_driver_node src/roarm_driver_node.cpp)
target_link_libraries(roarm_driver_node roarm_serial_driver)
ament_target_dependencies(roarm_driver_node
  rclcpp std_msgs sensor_msgs trajectory_msgs
)

# --- UGV driver ROS2 node ---
add_executable(ugv_driver_node src/ugv_driver_node.cpp)
target_link_libraries(ugv_driver_node ugv_serial_driver)
ament_target_dependencies(ugv_driver_node
  rclcpp std_msgs sensor_msgs geometry_msgs
)

# Install
install(TARGETS
  roarm_driver_node
  ugv_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
