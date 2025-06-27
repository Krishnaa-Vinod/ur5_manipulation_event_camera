import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # Define the simulation time parameter dictionary
    use_sim_time_param = {"use_sim_time": True}

    # Build the MoveIt configuration
    # This builder now correctly finds and loads all your YAML files automatically by name
    moveit_config = (
        MoveItConfigsBuilder("ur5_camera_gripper_moveit_config", package_name="ur5_camera_gripper_moveit_config")
        .robot_description(file_path="config/ur.urdf.xacro")
        .to_moveit_configs()
    )

    # Create a combined parameter dictionary for MoveGroup
    move_group_params = moveit_config.to_dict()
    move_group_params.update(use_sim_time_param)

    # Define the move_group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Define other nodes, ensuring use_sim_time is passed where needed

    # RViz Node
    rviz_config_file = os.path.join(
        get_package_share_directory("ur5_camera_gripper_moveit_config"), "config", "moveit.rviz"
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

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, use_sim_time_param],
    )

    # Gazebo Spawner Node
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "ur"],
        output="screen",
    )

    # Controller Spawner Nodes
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
        )
        for controller in controller_spawners
    ]

    # Gazebo Launch File
    gazebo_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("conveyorbelt_gazebo"),
            "launch",
            "conveyorbelt.launch.py",
        )
    )

    # Assemble the final launch description
    return LaunchDescription(
        [
            gazebo_launch,
            rviz_node,
            robot_state_publisher_node,
            move_group_node,
            spawn_entity_node,
        ]
        + spawner_nodes
    )