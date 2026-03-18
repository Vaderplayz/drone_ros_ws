cmake_minimum_required(VERSION 3.8)
project(px4_vio_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# Compatibility for environments where ament_target_dependencies is unavailable.
if(NOT COMMAND ament_target_dependencies)
  function(ament_target_dependencies target)
    foreach(dep IN LISTS ARGN)
      if(dep STREQUAL "rclcpp" AND TARGET rclcpp::rclcpp)
        target_link_libraries(${target} rclcpp::rclcpp)
      elseif(DEFINED ${dep}_TARGETS)
        target_link_libraries(${target} ${${dep}_TARGETS})
      elseif(DEFINED ${dep}_LIBRARIES)
        target_link_libraries(${target} ${${dep}_LIBRARIES})
      elseif(TARGET ${dep}::${dep})
        target_link_libraries(${target} ${dep}::${dep})
      endif()
      if(DEFINED ${dep}_INCLUDE_DIRS)
        target_include_directories(${target} PUBLIC ${${dep}_INCLUDE_DIRS})
      endif()
    endforeach()
  endfunction()
endif()

add_executable(vio_to_mavros_bridge_node src/vio_to_mavros_bridge_node.cpp)
ament_target_dependencies(vio_to_mavros_bridge_node rclcpp geometry_msgs nav_msgs mavros_msgs)

install(TARGETS
  vio_to_mavros_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
