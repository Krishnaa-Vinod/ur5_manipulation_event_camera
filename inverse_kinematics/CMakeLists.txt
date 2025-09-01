cmake_minimum_required(VERSION 3.8)
project(inverse_kinematics)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ─────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)                  # added
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)                    # added (used by many targets)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(linkattacher_msgs REQUIRED)

# ── Executables ──────────────────────────────────────────────────────
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

# YOLO‐based pick/place
add_executable(pick_place_yolo src/pick_place_yolo.cpp)
ament_target_dependencies(pick_place_yolo
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
  linkattacher_msgs
)

add_executable(track_pick_place src/track_pick_place_yolo.cpp)
ament_target_dependencies(track_pick_place
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

# ── NEW: Micro-saccade Servo node (C++) ─────────────────────────────
add_executable(micro_saccade_servo_cpp src/micro_saccade_servo.cpp)
ament_target_dependencies(micro_saccade_servo_cpp
  rclcpp
  geometry_msgs
  std_srvs 
)

target_link_libraries(micro_saccade_servo_cpp Threads::Threads)

# ── Installation ─────────────────────────────────────────────────────
install(TARGETS
  pick_place_parse
  pick_place_hsv
  pick_place_yolo
  track_pick_place
  move_to_joint_goal
  move_to_location
  simple_pick_place
  simple_move_with_gripper
  micro_saccade_servo_cpp
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
