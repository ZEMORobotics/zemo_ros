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
find_package(ruckig REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(zemo_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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
  nav2_util
  zemo_msgs
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


ament_package()
