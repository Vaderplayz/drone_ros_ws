# Timestamp: 2026-02-21 11:18:02 +07+0700
# Most Recent Update: Metadata timestamp now uses local time
cmake_minimum_required(VERSION 3.8)
project(vertical_lidar_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)

add_executable(vertical_lidar_mapper_node
  src/structural_mesh_exporter.cpp
  src/vertical_lidar_mapper.cpp
)
target_link_libraries(vertical_lidar_mapper_node PUBLIC
  rclcpp::rclcpp
  ${std_srvs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${message_filters_TARGETS}
  message_filters::message_filters
  ${sensor_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  tf2::tf2
  ${tf2_ros_TARGETS}
  ${tf2_sensor_msgs_TARGETS}
  ${laser_geometry_TARGETS}
  ${pcl_conversions_TARGETS}
)

target_link_libraries(vertical_lidar_mapper_node PUBLIC ${PCL_LIBRARIES})
target_link_libraries(vertical_lidar_mapper_node PUBLIC ${pcl_conversions_LIBRARIES})

add_executable(odom_to_tf_bridge_node
  src/odom_to_tf_bridge.cpp
)
target_link_libraries(odom_to_tf_bridge_node PUBLIC
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${tf2_ros_TARGETS}
)

add_executable(scan_frame_override_node
  src/scan_frame_override.cpp
)
target_link_libraries(scan_frame_override_node PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
)

install(TARGETS
  vertical_lidar_mapper_node
  odom_to_tf_bridge_node
  scan_frame_override_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_srvs
  diagnostic_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  message_filters
  tf2
  tf2_ros
  tf2_sensor_msgs
  laser_geometry
  pcl_conversions
)

ament_package()
