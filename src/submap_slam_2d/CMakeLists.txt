cmake_minimum_required(VERSION 3.16)
project(submap_slam_2d)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExternalLoopClosure.msg"
  DEPENDENCIES builtin_interfaces
)

add_library(submap_slam_2d_core
  src/descriptor.cpp
  src/mapping_core.cpp
  src/pose_graph.cpp
  src/scan_matcher.cpp
)
target_include_directories(submap_slam_2d_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(submap_slam_2d_core PUBLIC Ceres::ceres ${OpenCV_LIBS})

add_executable(submap_slam_2d_node src/submap_slam_node.cpp)
target_include_directories(submap_slam_2d_node PRIVATE include)
target_link_libraries(submap_slam_2d_node submap_slam_2d_core)
ament_target_dependencies(submap_slam_2d_node
  diagnostic_msgs geometry_msgs nav_msgs rclcpp sensor_msgs tf2
  std_msgs tf2_geometry_msgs tf2_ros visualization_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(submap_slam_2d_node "${cpp_typesupport_target}")

install(TARGETS submap_slam_2d_core submap_slam_2d_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch docs DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/replay_and_compare.sh scripts/compare_results.py
  scripts/comparison_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mapping_core test/test_mapping_core.cpp)
  if(TARGET test_mapping_core)
    target_link_libraries(test_mapping_core submap_slam_2d_core)
    target_include_directories(test_mapping_core PRIVATE include)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(submap_slam_2d_core)
ament_export_dependencies(Ceres OpenCV)
ament_package()
