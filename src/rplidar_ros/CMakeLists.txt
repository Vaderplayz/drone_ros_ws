cmake_minimum_required(VERSION 3.9.5)
project(rplidar_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -pedantic)
endif()

set(RPLIDAR_SDK_PATH "./sdk/")

file(GLOB RPLIDAR_SDK_SRC
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

set(req_deps
  "rclcpp"
  "sensor_msgs"
  "std_srvs"
  "rclcpp_components"
)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)

ament_auto_find_build_dependencies(REQUIRED ${req_deps})

include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  include
)

add_library(rplidar_composition_node SHARED
  src/rplidar_node.cpp
  ${RPLIDAR_SDK_SRC}
)
target_link_libraries(rplidar_composition_node
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  ${std_srvs_TARGETS}
  rclcpp_components::component
)

ament_auto_add_executable(rplidarNode src/node.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(rplidarNode
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  ${std_srvs_TARGETS}
  rclcpp_components::component
)

ament_auto_add_executable(rplidarNodeClient src/client.cpp)
target_link_libraries(rplidarNodeClient
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  ${std_srvs_TARGETS}
  rclcpp_components::component
)

ament_auto_add_executable(rplidar_composition src/standalone_rplidar.cpp)
target_link_libraries(rplidar_composition
  rplidar_composition_node
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  ${std_srvs_TARGETS}
  rclcpp_components::component
)

install(
  TARGETS rplidarNode rplidarNodeClient rplidar_composition
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS rplidar_composition_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_auto_package()
