cmake_minimum_required(VERSION 3.8)
project(apriltag_precision_landing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco videoio)

add_executable(apriltag_camera_detector_node src/apriltag_camera_detector_node.cpp)
target_link_libraries(apriltag_camera_detector_node PUBLIC
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  opencv_core
  opencv_imgproc
  opencv_calib3d
  opencv_aruco
  opencv_videoio
)

add_executable(apriltag_precision_landing_node src/apriltag_precision_landing_node.cpp)
target_link_libraries(apriltag_precision_landing_node PUBLIC
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
  tf2::tf2
  ${tf2_ros_TARGETS}
)

install(TARGETS
  apriltag_camera_detector_node
  apriltag_precision_landing_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/start_real_apriltag_pipeline.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
