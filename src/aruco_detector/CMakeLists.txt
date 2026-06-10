cmake_minimum_required(VERSION 3.10)
project(aruco_detector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(aruco_detector_node
  src/aruco_detector_node.cpp
)

ament_target_dependencies(
  aruco_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(
  aruco_detector_node
  ${OpenCV_LIBS}
)

install(TARGETS
  aruco_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
