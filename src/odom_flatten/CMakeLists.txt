cmake_minimum_required(VERSION 3.8)
project(odom_flatten)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(px4_odom_flatten_node
  src/px4_odom_flatten_node.cpp
)

target_link_libraries(px4_odom_flatten_node PUBLIC
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  tf2::tf2
  ${tf2_ros_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
)

install(TARGETS px4_odom_flatten_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
