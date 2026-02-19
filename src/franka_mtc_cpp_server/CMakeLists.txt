cmake_minimum_required(VERSION 3.8)
project(franka_mtc_cpp_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(franka_custom_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(mtc_pick_server_cpp src/mtc_pick_server.cpp)
ament_target_dependencies(mtc_pick_server_cpp 
  rclcpp 
  rclcpp_action 
  moveit_task_constructor_core 
  franka_custom_interfaces 
  geometry_msgs
  moveit_ros_planning_interface
  shape_msgs
  moveit_msgs
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS mtc_pick_server_cpp DESTINATION lib/${PROJECT_NAME})
ament_package()
