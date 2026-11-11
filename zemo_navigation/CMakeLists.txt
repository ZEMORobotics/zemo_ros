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
find_package(nav2_collision_monitor REQUIRED)
find_package(nav2_core REQUIRED)
find_package(angles REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(zemo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ruckig REQUIRED)



set(detector_executable_name zemo_collision_detector)
set(detector_library_name ${detector_executable_name}_core)
set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)


# --------------------------------------------------------------------------
# move action server
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
  pluginlib
)
target_link_libraries(move ruckig::ruckig)

rclcpp_components_register_node(
  move
  PLUGIN "zemo_navigation::MoveActionServer"
  EXECUTABLE move_server
)

# --------------------------------------------------------------------------
# additional obstacle plugin
add_library(additional_obstacle_layer SHARED
  src/AdditionalObstacleLayer.cpp)

target_include_directories(additional_obstacle_layer PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  additional_obstacle_layer
  rclcpp
  nav2_costmap_2d
  nav2_util
  geometry_msgs
  std_msgs
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_costmap_2d additional_obstacle_layer.xml)


install(
  TARGETS 
    move
    additional_obstacle_layer 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --------------------------------------------------------------------------
# collison detector
add_library(${detector_library_name} SHARED
  src/collision_detector/collision_detector.cpp
  src/collision_detector/dynamic_polygon.cpp
)

add_executable(${detector_executable_name}
  src/collision_detector/collision_detector_main.cpp
)

set_target_properties(${detector_executable_name} PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}"
  INSTALL_RPATH "$ORIGIN/../../.."
)

target_include_directories(${detector_library_name} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${detector_executable_name} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${detector_library_name}
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_util
  nav2_costmap_2d
  nav2_collision_monitor
  nav2_msgs
  visualization_msgs
)

target_link_libraries(${detector_executable_name}
  ${detector_library_name}
)

ament_target_dependencies(${detector_executable_name}
  rclcpp
)

rclcpp_components_register_nodes(${detector_library_name} "zemo_navigation::CollisionDetector")

install(TARGETS ${detector_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${detector_executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------------------------------
# controller plugin circular_arc
add_library(circular_arc SHARED src/controller_plugins/circular_arc.cpp)

target_include_directories(circular_arc PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(circular_arc
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  std_msgs
  angles
  nav2_util
  nav2_core
  tf2
  tf2_geometry_msgs
)
target_link_libraries(circular_arc ruckig::ruckig)

install(TARGETS circular_arc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(nav2_core circular_arc_plugin.xml)

# --------------------------------------------------------------------------
# controller plugin straight_line_move
add_library(straight_line_move SHARED src/controller_plugins/straight_line_move.cpp)

target_include_directories(straight_line_move PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(straight_line_move
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  std_msgs
  angles
  nav2_util
  nav2_core
  tf2
  tf2_geometry_msgs
)
target_link_libraries(straight_line_move ruckig::ruckig)

install(TARGETS straight_line_move
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(nav2_core straight_line_move_plugin.xml)


# --------------------------------------------------------------------------
# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
