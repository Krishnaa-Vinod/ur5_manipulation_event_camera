cmake_minimum_required(VERSION 3.8)
project(inverse_kinematics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(linkattacher_msgs REQUIRED)

add_executable(ur5_pick_place src/ur5_pick_place.cpp)
ament_target_dependencies(ur5_pick_place
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
  shape_msgs
  moveit_msgs
  linkattacher_msgs
)
install(TARGETS ur5_pick_place DESTINATION lib/${PROJECT_NAME})

ament_package()
