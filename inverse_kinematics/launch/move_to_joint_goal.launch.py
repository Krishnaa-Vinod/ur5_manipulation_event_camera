# File: launch/move_to_joint_goal.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Load the MoveIt! configuration for the UR5 + camera + gripper
    moveit_config = (
        MoveItConfigsBuilder("ur5_camera_gripper_moveit_config", package_name="ur5_camera_gripper_moveit_config")
        .robot_description(file_path="config/ur.urdf.xacro")
        .to_moveit_configs()
    )

    # Node that runs the move_to_joint_goal executable (now using IK)
    move_to_pose_node = Node(
        package="inverse_kinematics",
        executable="move_to_joint_goal",
        name="move_to_pose_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        move_to_pose_node
    ])
