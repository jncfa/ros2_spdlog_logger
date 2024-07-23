cmake_minimum_required(VERSION 3.8)
project(ros2_spdlog_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(spdlog_vendor REQUIRED)
find_package(rcl_logging_interface REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(fmt REQUIRED)

add_library(ros2_spdlog_logger src/ros2_spdlog_logger.cpp)
add_library(ros2_spdlog_logger::ros2_spdlog_logger ALIAS ros2_spdlog_logger)
target_compile_features(ros2_spdlog_logger PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(ros2_spdlog_logger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(
  ros2_spdlog_logger
  "rclcpp"
  "rcl"
  "spdlog_vendor"
  "rcl_logging_interface"
  "rcutils"
)
target_link_libraries(
  ros2_spdlog_logger
  fmt
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ros2_spdlog_logger PRIVATE "ROS2_SPDLOG_LOGGER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ros2_spdlog_logger
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

if(true)
add_executable(test_node src/test_node.cpp)
target_include_directories(test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(
  test_node
  "ros2_spdlog_logger"
)

install(TARGETS test_node
  DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  ros2_spdlog_logger
)

ament_export_dependencies(
  "rclcpp"
  "rclc"
  "spdlog_vendor"
  "rcl_logging_interface"
  "rcutils"
  "fmt"
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
