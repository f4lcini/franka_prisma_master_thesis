cmake_minimum_required(VERSION 3.8)
project(fr3_mtc_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(franka_custom_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(mtc_action_servers_node src/mtc_action_servers_node.cpp)
target_include_directories(mtc_action_servers_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mtc_action_servers_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  mtc_action_servers_node
  "rclcpp"
  "rclcpp_action"
  "moveit_core"
  "moveit_ros_planning_interface"
  "moveit_task_constructor_core"
  "franka_custom_interfaces"
  "geometry_msgs"
)

install(TARGETS mtc_action_servers_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
