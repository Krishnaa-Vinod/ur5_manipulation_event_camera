cmake_minimum_required(VERSION 3.8)
project(inverse_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(linkattacher_msgs REQUIRED) # CORRECT DEPENDENCY

# Add the executable
add_executable(
  ur5_pick_place
  src/ur5_pick_place.cpp
)

# Link dependencies
ament_target_dependencies(
  ur5_pick_place
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
  linkattacher_msgs # CORRECT DEPENDENCY
)

# Install the executable and launch file
install(
  TARGETS ur5_pick_place
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()