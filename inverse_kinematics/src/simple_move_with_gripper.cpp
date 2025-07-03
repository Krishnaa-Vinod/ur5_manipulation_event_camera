/*  simple_move_with_gripper.cpp
 *  Move UR arm **only if you ask for it** and always open/close the Robotiq gripper.
 *  ROS 2 Humble  –  MoveIt 2
 *
 *  Examples
 *  ---------
 *  # Just close the gripper (no arm motion)
 *  ros2 run inverse_kinematics simple_move_with_gripper --grip close
 *
 *  # Move the arm and close the gripper
 *  ros2 run inverse_kinematics simple_move_with_gripper --x 0.35 --z 0.20 --grip close
 *
 *  CLI flags (all optional)
 *  -----------------------
 *    --x <m>             Cartesian X in base_link frame (triggers arm motion)
 *    --y <m>             Cartesian Y (triggers arm motion)
 *    --z <m>             Cartesian Z (triggers arm motion)
 *    --grip open|close   Gripper command (default: open)
 *    --help              Print usage and exit
 */

#include <memory>
#include <thread>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

constexpr double PI = 3.141592653589793;
constexpr double GRIP_OPEN   = 0.0;   // rad – fully open
constexpr double GRIP_CLOSED = 0.8;   // rad – tune for your gripper

struct CliOptions
{
  double x = 0.5;
  double y = 0.0;
  double z = 0.1;
  bool   grip_open = true;   // true = open, false = close
  bool   move_arm  = false;  // becomes true if user specified any x/y/z
};

static void print_usage(const char * prog)
{
  std::cout << "Usage: " << prog
            << " [--x <m>] [--y <m>] [--z <m>] [--grip open|close]" << std::endl;
}

static CliOptions parse_arguments(int & argc, char ** argv)
{
  CliOptions opt;

  // Extract non‑ROS arguments first so we don't conflict with rclcpp parsing.
  auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<std::string> args{ non_ros_args.begin(), non_ros_args.end() };

  for (size_t i = 1; i < args.size(); ++i)  // index 0 = program name
  {
    const std::string & flag = args[i];

    if (flag == "--help" || flag == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    }

    auto need_value = [&](size_t idx){
      if (idx + 1 >= args.size()) {
        std::cerr << flag << " expects a value" << std::endl;
        print_usage(argv[0]);
        std::exit(1);
      }
    };

    if (flag == "--x") {
      need_value(i);
      opt.x = std::stod(args[++i]);
      opt.move_arm = true;
    }
    else if (flag == "--y") {
      need_value(i);
      opt.y = std::stod(args[++i]);
      opt.move_arm = true;
    }
    else if (flag == "--z") {
      need_value(i);
      opt.z = std::stod(args[++i]);
      opt.move_arm = true;
    }
    else if (flag == "--grip") {
      need_value(i);
      std::string g = args[++i];
      if (g == "open") opt.grip_open = true;
      else if (g == "close") opt.grip_open = false;
      else {
        std::cerr << "--grip must be 'open' or 'close'" << std::endl;
        print_usage(argv[0]);
        std::exit(1);
      }
    }
    else {
      std::cerr << "Unknown argument: " << flag << std::endl;
      print_usage(argv[0]);
      std::exit(1);
    }
  }
  return opt;
}

int main(int argc, char ** argv)
{
  /* ─────────  Parse CLI before ROS init  ───────── */
  CliOptions cli = parse_arguments(argc, argv);

  /* ─────────  ROS 2 init & spinner  ───────── */
  rclcpp::init(argc, argv);
  auto node   = rclcpp::Node::make_shared("simple_move_with_gripper");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]{ exec.spin(); });

  auto log = node->get_logger();

  RCLCPP_INFO(log, "%sarm motion, grip %s", cli.move_arm ? "With " : "Skipping ",
              cli.grip_open ? "open" : "close");

  /* ─────────  MoveGroup interface for gripper ───────── */
  moveit::planning_interface::MoveGroupInterface grip (node, "robotiq_gripper");

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm;
  if (cli.move_arm)
    arm = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node, "ur_manipulator");

  /* ─────────  Optional arm motion ───────── */
  if (cli.move_arm)
  {
    arm->setPoseReferenceFrame("base_link");
    arm->setEndEffectorLink("wrist_3_link");
    arm->setMaxVelocityScalingFactor(0.3);
    arm->setMaxAccelerationScalingFactor(0.3);
    arm->setPlanningTime(10.0);

    tf2::Quaternion q_down; q_down.setRPY(-PI, 0.0, -PI/2);

    geometry_msgs::msg::Pose goal;
    goal.orientation = tf2::toMsg(q_down);
    goal.position.x = cli.x;
    goal.position.y = cli.y;
    goal.position.z = cli.z;

    arm->setStartStateToCurrentState();
    arm->setPoseTarget(goal);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      arm->execute(plan);
      RCLCPP_INFO(log, "Arm reached goal pose.");
    }
    else
      RCLCPP_WARN(log, "Arm planning failed – skipping arm motion.");
  }

  /* ─────────  Gripper action ───────── */
  grip.setStartStateToCurrentState();
  double target = cli.grip_open ? GRIP_OPEN : GRIP_CLOSED;
  grip.setJointValueTarget("robotiq_85_left_knuckle_joint", target);
  grip.move();
  RCLCPP_INFO(log, "Gripper %s.", cli.grip_open ? "opened" : "closed");

  /* ─────────  Shutdown  ───────── */
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
