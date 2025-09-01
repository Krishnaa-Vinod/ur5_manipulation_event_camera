# spawn_ur5_table.launch.py
import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def _load_yaml_from_pkg(pkg: str, rel_path: str):
    """Try to load YAML from an installed package (share/...). Return dict or None."""
    try:
        share = get_package_share_directory(pkg)
        full = os.path.join(share, rel_path)
        if os.path.exists(full):
            with open(full, "r") as f:
                return yaml.safe_load(f)
    except Exception:
        pass
    return None

def generate_launch_description():
    # ── Args ──────────────────────────────────────────────────────────
    use_sim_time_arg     = DeclareLaunchArgument("use_sim_time", default_value="true")
    use_servo_arg        = DeclareLaunchArgument("use_servo",    default_value="true")
    servo_exec_arg       = DeclareLaunchArgument("servo_exec",   default_value="servo_node_main")
    servo_param_file_arg = DeclareLaunchArgument("servo_param_file", default_value="")  # absolute path or empty

    use_sim_time     = LaunchConfiguration("use_sim_time")
    use_servo        = LaunchConfiguration("use_servo")
    servo_exec       = LaunchConfiguration("servo_exec")
    servo_param_file = LaunchConfiguration("servo_param_file")

    use_sim_time_param = {"use_sim_time": use_sim_time}

    # ── MoveIt config (robot_description*, pipelines, kinematics, etc.) ──────
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
    move_group_params = moveit_config.to_dict()
    move_group_params.update(use_sim_time_param)

    # ── Gazebo ────────────────────────────────────────────────────────
    world_path = os.path.join(
        get_package_share_directory("ur_sim"),
        "worlds",
        "empty_with_link_attacher.world",
    )
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_path}.items(),
    )

    # ── RViz ──────────────────────────────────────────────────────────
    rviz_config_file = os.path.join(
        get_package_share_directory("ur5_camera_gripper_moveit_config"),
        "config",
        "moveit.rviz",
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

    # ── Robot State Publisher ─────────────────────────────────────────
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, use_sim_time_param],
    )

    # ── Move Group ───────────────────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # ── Spawn UR in Gazebo ───────────────────────────────────────────
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic", "robot_description", "-entity", "ur"],
        parameters=[use_sim_time_param],
    )

    # ── Controller Spawners ──────────────────────────────────────────
    controller_spawners = [
        "joint_state_broadcaster",
        "joint_trajectory_controller",
        "gripper_position_controller",
    ]
    spawner_nodes = [
        Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[controller],
            parameters=[use_sim_time_param],
        )
        for controller in controller_spawners
    ]

    # ── Servo fallback params (safe defaults; NOTE thresholds order) ─────────
    servo_fallback = {
        "moveit_servo": {
            "update_rate": 250,
            "command_in_type": "geometry_msgs/TwistStamped",
            "command_in_frame": "base_link",
            "planning_frame": "base_link",
            "move_group_name": "ur_manipulator",   # change to "manipulator" if your SRDF uses that
            "ee_frame_name": "tool0",

            "command_out_type": "trajectory_msgs/JointTrajectory",
            "publish_joint_trajectory": True,
            "command_out_topic": "/joint_trajectory_controller/joint_trajectory",
            "joint_trajectory_topic": "/joint_trajectory_controller/joint_trajectory",

            "incoming_command_timeout": 0.2,
            "num_outgoing_halt_msgs_to_publish": 4,
            "publish_status": True,
            "collision_check": True,

            "lower_singularity_threshold": 5.0,
            "hard_stop_singularity_threshold": 15.0,  # MUST be > lower
        }
    }

    # Try package YAML (used if no external file):
    servo_yaml_pkg = (
        _load_yaml_from_pkg("ur5_camera_gripper_moveit_config", "config/moveit_servo.yaml")
        or _load_yaml_from_pkg("ur5_camera_gripper_moveit_config", "config/moveit_servo_config.yaml")
        or servo_fallback
    )

    # ── Build Servo node at runtime to choose params deterministically ───────
    def _make_servo_node(context, *args, **kwargs):
        if LaunchConfiguration("use_servo").perform(context).strip().lower() not in ("true", "1", "yes"):
            return []

        exec_val = LaunchConfiguration("servo_exec").perform(context)
        file_arg = LaunchConfiguration("servo_param_file").perform(context)

        # Resolve sim time to a real bool (works either way though)
        sim_time_str = LaunchConfiguration("use_sim_time").perform(context).strip().lower()
        sim_time_bool = sim_time_str in ("true", "1", "yes")

        # Build parameter list (order matters: robot desc → servo config → sim_time)
        params = [moveit_config.to_dict()]

        if file_arg and os.path.exists(file_arg):
            with open(file_arg, "r") as f:
                params.append(yaml.safe_load(f))
        else:
            params.append(servo_yaml_pkg)

        params.append({"use_sim_time": sim_time_bool})

        servo_node = Node(
            package="moveit_servo",
            executable=exec_val,       # e.g., servo_node_main from your overlay
            name="servo_server",
            output="screen",
            emulate_tty=True,
            respawn=True,              # auto-restart if it throws
            parameters=params,
        )
        return [servo_node]

    servo_node_builder = OpaqueFunction(function=_make_servo_node)

    return LaunchDescription([
        use_sim_time_arg, use_servo_arg, servo_exec_arg, servo_param_file_arg,
        gazebo_launch,
        rviz_node,
        robot_state_publisher_node,
        move_group_node,
        spawn_entity_node,
        *spawner_nodes,
        servo_node_builder,
    ])
