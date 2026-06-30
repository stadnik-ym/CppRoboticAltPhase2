cmake_minimum_required(VERSION 3.8)
project(arena)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# --- ИЗМЕНЕНИЕ ЗДЕСЬ: Глобально подключаем заголовки OpenCV ---
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(arena_tracker_node src/arena_tracker_node.cpp)

# Линкуем ROS 2 зависимости
ament_target_dependencies(arena_tracker_node
  rclcpp
  geometry_msgs
)

# Линкуем библиотеки OpenCV (без слова PRIVATE)
target_link_libraries(arena_tracker_node ${OpenCV_LIBRARIES})

install(TARGETS
  arena_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
