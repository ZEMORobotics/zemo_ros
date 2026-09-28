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
find_package(PkgConfig REQUIRED)

set(CAN_INTERFACE can_interface)
set(SERVO_DRIVER scservo)
set(SERVO_INTERFACE servo_interface)


# ================ CAN SYSTEM ===================
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

# ================ SERVO SYSTEM ===================

add_library(${SERVO_DRIVER} STATIC
  src/servo_system/waveshare_driver/SCSerial.cpp
  src/servo_system/waveshare_driver/SCS.cpp
  src/servo_system/waveshare_driver/SMS_STS.cpp
  src/servo_system/waveshare_driver/SCSCL.cpp
  src/servo_system/waveshare_driver/HLSCL.cpp
  src/servo_system/waveshare_driver/SCS0009.cpp
)

target_include_directories(${SERVO_DRIVER} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GPIOD_INCLUDE_DIRS}
)

target_link_libraries(${SERVO_DRIVER}
  ${GPIOD_LIBRARIES}
)

target_compile_features(${SERVO_DRIVER} PUBLIC cxx_std_17)

set_target_properties(${SERVO_DRIVER} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

add_library(${SERVO_INTERFACE} SHARED
  src/servo_system/servo_interface.cpp
)

target_include_directories(${SERVO_INTERFACE} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${SERVO_INTERFACE}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(${SERVO_INTERFACE}
  ${SERVO_DRIVER}
)

pluginlib_export_plugin_description_file(
  hardware_interface
  zemo_servo_interface.xml
)

# =================================================

install(
  TARGETS ${CAN_INTERFACE} ${SERVO_INTERFACE}
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

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
