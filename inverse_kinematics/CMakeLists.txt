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

# Add the executable for your C++ node
add_executable(pick_place_parse src/pick_place_parse.cpp)

# Link the executable against its dependencies
ament_target_dependencies(pick_place_parse
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  linkattacher_msgs
)

add_executable(move_to_joint_goal src/move_to_joint_goal.cpp)

# Link it against its dependencies
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

find_package(moveit_ros_planning_interface REQUIRED)

add_executable(simple_move_with_gripper src/simple_move_with_gripper.cpp)
ament_target_dependencies(simple_move_with_gripper
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
)


# Install the executable
install(TARGETS
  move_to_joint_goal
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS move_to_location
  simple_move_with_gripper
  simple_pick_place
  pick_place_parse
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()