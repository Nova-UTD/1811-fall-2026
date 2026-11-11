cmake_minimum_required(VERSION 3.8)
project(lidar_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the single Python file on the workspace's Python import path.
ament_python_install_module(lidar_perception_transforms.py)

install(PROGRAMS
  lidar_camera_registration.py
  corner_viewer.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES interactive_calibration.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Run the frame-composition and live-URDF regression checks with colcon test.
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME calibration COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_calibration.py")
  add_test(NAME hub_calibration COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_hub_calibration.py")
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
