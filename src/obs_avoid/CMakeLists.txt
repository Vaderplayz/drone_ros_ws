cmake_minimum_required(VERSION 3.8)
project(obs_avoid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  set(ament_cmake_copyright_FOUND TRUE)
 
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(user_ctrl src/offboard_node_user_ctrl.cpp)
target_link_libraries(user_ctrl PUBLIC
  rclcpp::rclcpp
  ${mavros_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)
install(TARGETS user_ctrl DESTINATION lib/${PROJECT_NAME})

add_executable(spiral_mapping_mode src/spiral_mapping_mode.cpp)
target_link_libraries(spiral_mapping_mode PUBLIC
  rclcpp::rclcpp
  ${mavros_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)
install(TARGETS spiral_mapping_mode DESTINATION lib/${PROJECT_NAME})

add_executable(lawnmower_mapping_mode src/lawnmower_mapping_mode.cpp)
target_link_libraries(lawnmower_mapping_mode PUBLIC
  rclcpp::rclcpp
  ${mavros_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)
install(TARGETS lawnmower_mapping_mode DESTINATION lib/${PROJECT_NAME})

add_executable(slam_navigation_mode src/slam_navigation_mode.cpp)
target_link_libraries(slam_navigation_mode PUBLIC
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
)
install(TARGETS slam_navigation_mode DESTINATION lib/${PROJECT_NAME})


add_executable(local_planner_mode_a src/local_planner_mode_a.cpp)
target_link_libraries(local_planner_mode_a PUBLIC
  rclcpp::rclcpp
  ${mavros_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  ${nav_msgs_TARGETS}
  tf2::tf2
)
install(TARGETS local_planner_mode_a DESTINATION lib/${PROJECT_NAME})

add_executable(local_planner_sector_mode src/local_planner_sector_mode.cpp)
target_link_libraries(local_planner_sector_mode PUBLIC
  rclcpp::rclcpp
  ${mavros_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  ${nav_msgs_TARGETS}
  tf2::tf2
)
install(TARGETS local_planner_sector_mode DESTINATION lib/${PROJECT_NAME})

add_executable(local_planner_hybrid_mode src/local_planner_hybrid_mode.cpp)
target_link_libraries(local_planner_hybrid_mode PUBLIC
  rclcpp::rclcpp
  ${mavros_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  ${nav_msgs_TARGETS}
  tf2::tf2
)
install(TARGETS local_planner_hybrid_mode DESTINATION lib/${PROJECT_NAME})

add_executable(odom_gps_validator src/odom_gps_validator.cpp)
target_link_libraries(odom_gps_validator PUBLIC
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  ${geometry_msgs_TARGETS}
)
install(TARGETS odom_gps_validator DESTINATION lib/${PROJECT_NAME})

add_executable(mavros_trajectory_3d_node src/mavros_trajectory_3d_node.cpp)
target_link_libraries(mavros_trajectory_3d_node PUBLIC
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)
install(TARGETS mavros_trajectory_3d_node DESTINATION lib/${PROJECT_NAME})

ament_package()
