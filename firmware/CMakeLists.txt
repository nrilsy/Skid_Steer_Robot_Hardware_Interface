cmake_minimum_required(VERSION 3.8)
project(firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS and other dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Set paths for pigpio
set(PIGPIOD_IF2_INCLUDE_DIR "/home/pi/pigpio-master")
set(PIGPIOD_IF2_LIBRARY "/home/pi/pigpio-master/libpigpiod_if2.so")

# Add library
add_library(firmware_library SHARED src/raspberryPi_hardware_interface.cpp src/wheel.cpp)

target_include_directories(firmware_library PRIVATE 
  include
  ${PIGPIOD_IF2_INCLUDE_DIR}
)

ament_target_dependencies(firmware_library
  rclcpp
  rclcpp_lifecycle
  pluginlib
  hardware_interface
  nav_msgs
)

pluginlib_export_plugin_description_file(hardware_interface hardware_interface.xml)

target_link_libraries(firmware_library
  ${PIGPIOD_IF2_LIBRARY}
)

# Install targets
install(
  TARGETS firmware_library
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(
  include
)

ament_export_libraries(
  firmware_library
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  nav_msgs
)

ament_package()
