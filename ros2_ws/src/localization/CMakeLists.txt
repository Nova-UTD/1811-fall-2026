cmake_minimum_required(VERSION 3.8)
project(localization)

find_package(ament_cmake REQUIRED)

# Install launch + config so they resolve via $(find localization)/...
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
