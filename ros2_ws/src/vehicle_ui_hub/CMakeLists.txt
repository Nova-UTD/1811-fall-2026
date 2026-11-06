cmake_minimum_required(VERSION 3.8)
project(vehicle_ui_hub)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

if(TARGET Qt6::Widgets)
  set(HUB_QT_WIDGETS Qt6::Widgets)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  set(HUB_QT_WIDGETS Qt5::Widgets)
endif()

add_executable(vehicle_ui_hub src/main.cpp)
target_link_libraries(vehicle_ui_hub ${HUB_QT_WIDGETS})
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(vehicle_ui_hub
    ament_index_cpp rclcpp rviz_common std_msgs std_srvs)
else()
  target_link_libraries(vehicle_ui_hub
    ament_index_cpp::ament_index_cpp rclcpp::rclcpp rviz_common::rviz_common
    std_msgs::std_msgs__rosidl_typesupport_cpp std_srvs::std_srvs__rosidl_typesupport_cpp)
endif()

install(TARGETS vehicle_ui_hub DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
