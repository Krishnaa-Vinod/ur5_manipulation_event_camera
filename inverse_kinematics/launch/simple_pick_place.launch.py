# File: launch/simple_pick_place.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Load URDF/SRDF/etc
    moveit_config = (
        MoveItConfigsBuilder("ur5_camera_gripper_moveit_config",
                             package_name="ur5_camera_gripper_moveit_config")
        .robot_description(file_path="config/ur.urdf.xacro")
        .to_moveit_configs()
    )

    # 1) robot_state_publisher to publish joint TFs
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[ moveit_config.robot_description ],
    )

    # 2) static world→base_link at identity
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_link",
        arguments=[
            "0","0","0",    # x y z
            "0","0","0",    # roll pitch yaw
            "world","base_link"
        ]
    )

    # 3) your pick‐place node
    simple_pp = Node(
        package="inverse_kinematics",
        executable="simple_pick_place",
        name="simple_pick_place",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            # adjust your pick/place poses here (all in 'world')
            {"pick_pose.x":  0.0}, {"pick_pose.y":  0.0}, {"pick_pose.z":  0.76},
            {"place_pose.x": 0.0}, {"place_pose.y": 0.2}, {"place_pose.z": 0.76},
        ],
    )

    return LaunchDescription([ rsp, static_tf, simple_pp ])
