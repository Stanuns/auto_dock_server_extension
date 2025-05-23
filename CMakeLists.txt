cmake_minimum_required(VERSION 3.8)
project(auto_dock_server_extension)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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

add_executable(remote_auto_dock_server src/remote_auto_dock_server.cpp)
ament_target_dependencies(remote_auto_dock_server
  robot_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_msgs
  std_srvs
  nav_msgs
  tf2 
  tf2_ros 
  tf2_geometry_msgs
)

install(TARGETS remote_auto_dock_server
    DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY launch params
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
