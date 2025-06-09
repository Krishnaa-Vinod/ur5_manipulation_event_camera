#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # ────────────────────────────────────────────────
    # ❶  General arguments (UR5 spawn position)
    # ────────────────────────────────────────────────
    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="UR5 X")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="UR5 Y")
    z_arg = DeclareLaunchArgument("z", default_value="0.0", description="UR5 Z")

    # ────────────────────────────────────────────────
    # ❷  Include the *conveyorbelt* launch file
    #     (this starts gazebo + the conveyor world)
    # ────────────────────────────────────────────────
    conveyor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("conveyorbelt_gazebo"),
                "launch",
                "conveyorbelt.launch.py",
            )
        ),
        launch_arguments={                # same flags you used before
            "use_sim_time": "true",
            "debug": "false",
            "gui": "true",
            "paused": "true",
        }.items(),
    )

    # ────────────────────────────────────────────────
    # ❸  Build MoveIt config
    # ────────────────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name="ur5_camera_moveit_config")
        .robot_description(file_path="config/ur.urdf.xacro")
        .robot_description_semantic(file_path="config/ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # ────────────────────────────────────────────────
    # ❹  Nodes: Robot description → Gazebo
    # ────────────────────────────────────────────────
    spawn_ur5 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "cobot",
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
        ],
        output="screen",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                get_package_share_directory("ur_sim"),
                "config",
                "ur5_controllers.yaml",
            ),
        ],
        output="screen",
        remappings=[("~/robot_description", "/robot_description")],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    arm_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[{**moveit_config.to_dict(), "use_sim_time": True}],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("ur5_camera_moveit_config"),
                "config",
                "moveit.rviz",
            )
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # ────────────────────────────────────────────────
    # ❺  Start controllers in sequence
    # ────────────────────────────────────────────────
    delay_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )
    delay_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[arm_trajectory_controller_spawner],
        )
    )
    delay_rviz = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[rviz_node],
        )
    )

    # ────────────────────────────────────────────────
    # ❻  Assemble LaunchDescription
    # ────────────────────────────────────────────────
    ld = LaunchDescription()
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)

    ld.add_action(conveyor_launch)               # Gazebo + conveyor world
    ld.add_action(controller_manager)
    ld.add_action(spawn_ur5)
    ld.add_action(robot_state_publisher)
    ld.add_action(move_group_node)

    ld.add_action(delay_broadcaster)
    ld.add_action(delay_arm_controller)
    ld.add_action(delay_rviz)

    return ld
