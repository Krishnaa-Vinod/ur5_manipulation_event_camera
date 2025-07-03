from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur5_camera_gripper_moveit_config",
                             package_name="ur5_camera_gripper_moveit_config")
        .robot_description(file_path="config/ur.urdf.xacro")
        .to_moveit_configs()
    )

    move_to_location = Node(
        package="inverse_kinematics",
        executable="move_to_location",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"target_pose.x": 0.5},
            {"target_pose.y": 0.0},
            {"target_pose.z": 0.4},
            {"target_pose.qx": -0.7071},
            {"target_pose.qy":  0.0   },
            {"target_pose.qz":  0.0   },
            {"target_pose.qw":  0.7071},
        ],
    )

    return LaunchDescription([move_to_location])
