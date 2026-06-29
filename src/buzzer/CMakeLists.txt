cmake_minimum_required(VERSION 3.14)
project(buzzer_node)

# --- C++ standard ---
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- ROS 2 dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# --- libgpiod (v2) ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod>=2.0)

# --- Executable ---
add_executable(buzzer_node
  src/buzzer_node.cpp
)

target_include_directories(buzzer_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GPIOD_INCLUDE_DIRS}
)

target_link_libraries(buzzer_node
  ${GPIOD_LIBRARIES}
)

ament_target_dependencies(buzzer_node
  rclcpp
  std_msgs
)

# --- Install ---
install(TARGETS buzzer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

# --- Tests (optional) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
