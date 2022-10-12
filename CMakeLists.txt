cmake_minimum_required(VERSION 3.8)
project(wrp_ros_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(wrp_ros2 REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(tier4_api_msgs REQUIRED)
# find_package(tier4_api_utils REQUIRED)
# find_package(tier4_debug_msgs REQUIRED)
# find_package(tier4_external_api_msgs REQUIRED)
# find_package(tier4_vehicle_msgs REQUIRED)
# find_package(vehicle_info_util REQUIRED)
# find_package(diagnostic_updater REQUIRED)
# find_package(message_filters REQUIRED)
include_directories(
  include
)

add_executable(wrp_ros_interface_node src/wrp_ros_interface_node.cpp)
ament_target_dependencies(wrp_ros_interface_node
  rclcpp
  std_msgs
  nav_msgs
  wrp_ros2
  autoware_auto_control_msgs
  autoware_auto_vehicle_msgs
  geometry_msgs
)

install(TARGETS
  wrp_ros_interface_node
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
