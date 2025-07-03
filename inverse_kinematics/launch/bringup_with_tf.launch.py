import os

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Expand your Xacro via MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder("ur5_camera_gripper_moveit_config",
                             package_name="ur5_camera_gripper_moveit_config")
        .robot_description(file_path="config/ur.urdf.xacro")
        .to_moveit_configs()
    )

    # robot_state_publisher with the URDF already loaded into 'robot_description'
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[ moveit_config.robot_description ]
    )

    # Your mover node
    mover_node = Node(
        package="inverse_kinematics",
        executable="simple_move_with_gripper",
        name="simple_move_with_gripper",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    return LaunchDescription([rsp_node, mover_node])
