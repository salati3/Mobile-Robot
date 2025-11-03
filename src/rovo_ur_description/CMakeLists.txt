cmake_minimum_required(VERSION 3.10)
project(rovo_ur_description)

find_package(ament_cmake REQUIRED)
install(
  DIRECTORY urdf meshes rviz config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()