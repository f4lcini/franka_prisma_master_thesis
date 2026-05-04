cmake_minimum_required(VERSION 3.8)
project(franka_bimanual_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(franka_custom_interfaces REQUIRED)

# Add executable
add_executable(bimanual_planner_node src/bimanual_planner_node.cpp)
target_include_directories(bimanual_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(bimanual_planner_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  geometry_msgs
  shape_msgs
  control_msgs
  trajectory_msgs
  franka_custom_interfaces
)

# Install
install(TARGETS bimanual_planner_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
