cmake_minimum_required(VERSION 3.8)
project(rogilink_flex)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rogilink_flex_interfaces REQUIRED)

add_executable(rogilink_flex src/uart_node.cpp src/uart.cpp)
ament_target_dependencies(rogilink_flex rclcpp rogilink_flex_interfaces)

set(BUILD_TESTS FALSE CACHE INTERNAL "" FORCE)

add_subdirectory(lib/CppLinuxSerial)
target_link_libraries(rogilink_flex CppLinuxSerial)

target_include_directories(
  rogilink_flex
  PRIVATE include/rogilink_flex/
  PRIVATE lib/CppLinuxSerial/include/CppLinuxSerial/
  PRIVATE lib/json/include/
)


install(TARGETS
  rogilink_flex
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
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
