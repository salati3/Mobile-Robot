cmake_minimum_required(VERSION 3.8)
project(rovo_can_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Library implementing the SystemInterface
add_library(rovo_can_driver SHARED
  src/rovo_can_system.cpp
)

target_include_directories(rovo_can_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rovo_can_driver
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# Export plugin description to ament index for 'hardware_interface'
pluginlib_export_plugin_description_file(
  hardware_interface
  plugin_description.xml
)

install(
  TARGETS rovo_can_driver
  EXPORT export_rovo_can_driver
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_rovo_can_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)
ament_export_libraries(rovo_can_driver)

ament_package()
