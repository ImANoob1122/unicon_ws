cmake_minimum_required(VERSION 3.8)
project(uniconlinkFlex)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uniconlink_flex_interfaces REQUIRED)

# Include directories
include_directories(include)

# Create library
add_library(${PROJECT_NAME}_lib
  src/memory_map.cpp
  src/variant_converter.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  uniconlink_flex_interfaces
)

# Create executable
add_executable(${PROJECT_NAME}_node src/uniconlink_flex_node.cpp)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  uniconlink_flex_interfaces
)

# Install
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
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
