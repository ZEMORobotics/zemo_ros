cmake_minimum_required(VERSION 3.8)
project(zemo_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(zemo_msgs REQUIRED)

set(SERVO servo_controller)

# Servo controller

add_library(${SERVO} SHARED
  src/servo_controller.cpp
)

target_include_directories(${SERVO} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(controller_interface zemo_servo_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include/
)
install(TARGETS ${SERVO}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_target_dependencies(${SERVO}
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  zemo_msgs
)
ament_package()
