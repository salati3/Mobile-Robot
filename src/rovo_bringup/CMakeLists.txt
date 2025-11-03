cmake_minimum_required(VERSION 3.10)
project(rovo_bringup)


find_package(ament_cmake REQUIRED)
# ONLY install what actually exists
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()