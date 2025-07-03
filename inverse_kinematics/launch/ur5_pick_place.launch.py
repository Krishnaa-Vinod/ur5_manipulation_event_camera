from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Allow simulation clock
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    # Build MoveIt configuration (must match your ur5_camera_gripper_moveit_config package)
    moveit_config = (
        MoveItConfigsBuilder("ur5_camera_gripper_moveit_config",
                             package_name="ur5_camera_gripper_moveit_config")
        .robot_description(file_path="config/ur.urdf.xacro")
        .to_moveit_configs()
    )

    # C++ pick & place node
    pick_place_node = Node(
        package="inverse_kinematics",
        executable="ur5_pick_place",
        name="ur5_pick_place",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        pick_place_node,
    ])
