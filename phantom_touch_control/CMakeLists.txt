cmake_minimum_required(VERSION 3.8)
project(phantom_touch_control)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(phantom_touch_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(HD REQUIRED)
find_package(range-v3 REQUIRED)

add_library(phantom_touch_control_node SHARED src/phantom_touch_control_node.cpp src/hd++.cpp src/hd++.h)
target_compile_features(phantom_touch_control_node PUBLIC cxx_std_17)
target_compile_options(phantom_touch_control_node PRIVATE -Wall -Wextra)
ament_target_dependencies(phantom_touch_control_node rclcpp rclcpp_components std_msgs geometry_msgs sensor_msgs phantom_touch_msgs Boost Eigen3 HD range-v3)
target_include_directories(phantom_touch_control_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
rclcpp_components_register_nodes(phantom_touch_control_node "phantom_touch_control::TouchControlNode")



install(
  TARGETS phantom_touch_control_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
