import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # Simulation time for all relevant nodes
    use_sim_time_param = {"use_sim_time": True}

    # Build the MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name="ur5_camera_gripper_moveit_config")
        .robot_description(file_path="config/ur.urdf.xacro")
        .robot_description_semantic(file_path="config/ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True
        )
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Combine MoveIt params + sim time
    move_group_params = moveit_config.to_dict()
    move_group_params.update(use_sim_time_param)

    # --- Nodes ---

    # 1) MoveGroup (with sim time)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # 2) RViz (with sim time)
    rviz_config_file = os.path.join(
        get_package_share_directory("ur5_camera_gripper_moveit_config"),
        "config",
        "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            use_sim_time_param,
        ],
    )

    # 3) Robot State Publisher (with sim time)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, use_sim_time_param],
    )

    # 4) Gazebo Spawner (needs sim time to read /clock)
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "ur"],
        output="screen",
        parameters=[use_sim_time_param],
    )

    # 5) Controller Spawners (joint_state_broadcaster publishes /joint_states)
    controller_spawners = [
        "joint_state_broadcaster",
        "joint_trajectory_controller",
        "gripper_position_controller",
    ]
    spawner_nodes = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller],
            parameters=[use_sim_time_param],
        )
        for controller in controller_spawners
    ]

    # 6) Underlying Gazebo world (conveyor belt)
    gazebo_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("conveyorbelt_gazebo"),
            "launch",
            "conveyorbelt.launch.py",
        )
    )

    return LaunchDescription([
        gazebo_launch,
        rviz_node,
        robot_state_publisher_node,
        move_group_node,
        spawn_entity_node,
        *spawner_nodes,
    ])
