cmake_minimum_required(VERSION 3.8)
project(zemo_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Threads REQUIRED)

set(CAN_INTERFACE can_interface)

add_library(${CAN_INTERFACE} SHARED
  src/can_system/can_interface.cpp
  src/can_system/diff_drive.cpp
  src/can_system/ir_sensor.cpp
  src/can_system/cinch.cpp
)

target_link_libraries(${CAN_INTERFACE}
  Threads::Threads
)

target_include_directories(${CAN_INTERFACE} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${CAN_INTERFACE}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  hardware_interface
  zemo_can_interface.xml
)

install(
  TARGETS ${CAN_INTERFACE}
  EXPORT export_zemo_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_zemo_hardware
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
