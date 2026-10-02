cmake_minimum_required(VERSION 3.8)
project(zemo_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(zemo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ruckig REQUIRED)

add_library(move SHARED src/move.cpp)

target_include_directories(move PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  move
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_costmap_2d
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_util
  zemo_msgs
  std_msgs
  geometry_msgs
)
target_link_libraries(move ruckig::ruckig)

rclcpp_components_register_node(
  move
  PLUGIN "zemo_navigation::MoveActionServer"
  EXECUTABLE move_server
)

install(
  TARGETS move
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files.
install(DIRECTORY
  launch
  config
  map
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
