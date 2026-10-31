cmake_minimum_required(VERSION 3.8)
project(vehicle_1811_description)

find_package(ament_cmake REQUIRED)

# Install description assets so other packages can $(find vehicle_1811_description)/...
install(
  DIRECTORY urdf launch rviz config meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
