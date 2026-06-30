cmake_minimum_required(VERSION 3.14)
project(buzzer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(std_msgs    REQUIRED)

find_package(PkgConfig REQUIRED)

# libgpiod >= 2.0 required — the C++ API (request_builder, line_request)
# used in this node only exists from 2.0 onward. libgpiodcxx is the
# C++ bindings pkg-config module name on most distros / nix-ros-overlay.
pkg_check_modules(GPIODCXX REQUIRED libgpiodcxx>=2.0)

add_executable(buzzer_node
  src/buzzer_node.cpp
)

target_include_directories(buzzer_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${GPIODCXX_INCLUDE_DIRS}
)

target_link_libraries(buzzer_node
  ${GPIODCXX_LIBRARIES}
)

target_compile_options(buzzer_node PRIVATE ${GPIODCXX_CFLAGS_OTHER})

ament_target_dependencies(buzzer_node
  rclcpp
  std_msgs
)

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
