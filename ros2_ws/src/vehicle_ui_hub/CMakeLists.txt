# Build the native RViz bridge and install the Python vehicle hub.
cmake_minimum_required(VERSION 3.8)
project(vehicle_ui_hub)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

# Humble uses Qt 5; newer ROS distributions may export Qt 6 through RViz.
if(TARGET Qt6::Widgets)
  set(HUB_QT_WIDGETS Qt6::Widgets)
  set(HUB_QT_MAJOR 6)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  set(HUB_QT_WIDGETS Qt5::Widgets)
  set(HUB_QT_MAJOR 5)
endif()
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/rviz_qt_major" "${HUB_QT_MAJOR}\n")

# RViz remains native; Python embeds its QWidget through the bridge ABI.
add_library(vehicle_hub_rviz_bridge SHARED src/rviz_bridge.cpp src/rviz_bridge.hpp)
target_link_libraries(vehicle_hub_rviz_bridge ${HUB_QT_WIDGETS})
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(vehicle_hub_rviz_bridge rclcpp rviz_common rviz_rendering)
else()
  target_link_libraries(vehicle_hub_rviz_bridge rclcpp::rclcpp rviz_common::rviz_common rviz_rendering::rviz_rendering)
endif()

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME hub_python COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_hub.py
    BagRecorderTest CalibrationDetailsTest CalibrationSweepTest CameraViewsTest NetworkStatusTest PageTransitionTest ScreensTest SketchPadTest
    SoftwareUpdateTest TouchInteractionTest VehicleStatusTest RvizViewportTest.test_qt_mismatch_is_rejected_before_loading_native_libraries)
  # RViz owns process-wide OGRE resources, so each native integration test runs
  # in its own process instead of constructing RViz twice in one process.
  add_test(NAME rviz_bridge_python COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_hub.py
    RvizViewportTest.test_native_rviz_in_pyqt)
  add_test(NAME hub_integration COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_hub.py
    HubIntegrationTest)
  # Exercise an actual short recording separately from the mock process tests.
  add_test(NAME hub_rosbag COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_hub.py
    RealBagTest)
  set_tests_properties(hub_python rviz_bridge_python hub_integration hub_rosbag PROPERTIES TIMEOUT 75 SKIP_RETURN_CODE 77
    ENVIRONMENT "ROS_LOCALHOST_ONLY=1;PYTHONDONTWRITEBYTECODE=1;NOVA_RVIZ_LIBRARY=$<TARGET_FILE:vehicle_hub_rviz_bridge>;NOVA_RVIZ_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/config/hub.rviz;NOVA_RVIZ_QT_MAJOR=${HUB_QT_MAJOR}")
endif()

# Keep the existing executable name used by ros2 launch and ./g.
install(TARGETS vehicle_hub_rviz_bridge LIBRARY DESTINATION lib)
install(FILES src/rviz_viewport.py src/camera_views.py src/developer_tools.py src/screens.py src/vehicle_status.py
  src/calibration_details.py src/bag_recorder.py src/network_status.py src/sketch_pad.py src/hub_widgets.py
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/hub.py DESTINATION lib/${PROJECT_NAME} RENAME vehicle_ui_hub)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/rviz_qt_major" DESTINATION share/${PROJECT_NAME})
install(DIRECTORY assets DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
