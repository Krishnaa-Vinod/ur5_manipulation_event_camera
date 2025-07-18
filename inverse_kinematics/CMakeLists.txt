cmake_minimum_required(VERSION 3.8)
project(inverse_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(linkattacher_msgs REQUIRED)

# (optional) if you use tf2_geometry_msgs directly
find_package(tf2_geometry_msgs REQUIRED)

# original parse node
add_executable(pick_place_parse src/pick_place_parse.cpp)
ament_target_dependencies(pick_place_parse
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  linkattacher_msgs
)

# HSV‐based pick/place
add_executable(pick_place_hsv src/pick_place_hsv.cpp)
ament_target_dependencies(pick_place_hsv
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
  linkattacher_msgs
)

# YOLO‐based pick/place (new)
add_executable(pick_place_yolo src/pick_place_yolo.cpp)
ament_target_dependencies(pick_place_yolo
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
  linkattacher_msgs
)

add_executable(move_to_joint_goal src/move_to_joint_goal.cpp)
ament_target_dependencies(move_to_joint_goal
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  linkattacher_msgs
)

add_executable(move_to_location src/move_to_location.cpp)
ament_target_dependencies(move_to_location
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
)

add_executable(simple_pick_place src/simple_pick_place.cpp)
ament_target_dependencies(simple_pick_place
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  linkattacher_msgs
)

add_executable(simple_move_with_gripper src/simple_move_with_gripper.cpp)
ament_target_dependencies(simple_move_with_gripper
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
)

# Install all executables
install(TARGETS
  pick_place_parse
  pick_place_hsv
  pick_place_yolo
  move_to_joint_goal
  move_to_location
  simple_pick_place
  simple_move_with_gripper
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
