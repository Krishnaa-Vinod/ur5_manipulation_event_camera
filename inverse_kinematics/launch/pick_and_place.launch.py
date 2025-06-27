from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Use MoveItConfigsBuilder to easily load the robot description,
    # SRDF, and kinematics config
    moveit_config = (
        MoveItConfigsBuilder("ur5_camera_gripper_moveit_config", package_name="ur5_camera_gripper_moveit_config")
        .to_moveit_configs()
    )

    # Create the node for your C++ application
    pick_and_place_node = Node(
        package="inverse_kinematics",
        executable="ur5_pick_place",
        output="screen",
        # IMPORTANT: Pass the MoveIt configs to the node.
        # This ensures it can find the robot description and SRDF.
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([pick_and_place_node])