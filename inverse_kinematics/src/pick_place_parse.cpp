// src/pick_place_parse.cpp

#include <memory>
#include <thread>
#include <chrono>
#include <iostream>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

using namespace std::chrono_literals;
static constexpr double PI           = 3.141592653589793;
static constexpr double OFFSET_Z     = 0.3;    // 30 cm lift
static constexpr double GRIP_PERCENT = 70.0;   // close to 70%
static constexpr double EE_MIN       = 0.0;    // fully open
static constexpr double EE_MAX       = 0.8;    // fully closed

struct PickTask {
  std::string model_name;
  std::string link_name;
  geometry_msgs::msg::Pose  pick;
  geometry_msgs::msg::Pose  pre_pick;
};

int main(int argc, char** argv)
{
  // Determine whether we're in 5-arg or 4-arg mode
  int rem = argc - 1;
  int group_size;
  bool five_arg_mode = false;

  if (rem % 5 == 0) {
    group_size     = 5;
    five_arg_mode  = true;
  } else if (rem % 4 == 0) {
    group_size     = 4;
    five_arg_mode  = false;
  } else {
    std::cerr << "Usage:\n"
              << "  (1) 4-arg mode:  obj1_name px1 py1 pz1 [obj2_name px2 py2 pz2 ...]\n"
              << "  (2) 5-arg mode:  obj1_name link1_name px1 py1 pz1 [obj2_name link2_name px2 py2 pz2 ...]\n";
    return 1;
  }

  int num_tasks = rem / group_size;
  std::vector<PickTask> tasks;
  tasks.reserve(num_tasks);

  // Common down-facing orientation
  tf2::Quaternion q; 
  q.setRPY(-PI, 0, -PI/2);
  auto common_q = tf2::toMsg(q);

  // Parse each group
  for (int i = 0; i < num_tasks; ++i) {
    int base = 1 + i * group_size;
    PickTask t;
    t.model_name = argv[base + 0];

    if (five_arg_mode) {
      // explicit link name
      t.link_name = argv[base + 1];
    } else {
      // assume URDF link is always "box_link"
      t.link_name = "box";
    }

    // coords start after model_name (+ link_name if five_arg_mode)
    int coord_offset = five_arg_mode ? 2 : 1;
    double px = std::stod(argv[base + coord_offset + 0]);
    double py = std::stod(argv[base + coord_offset + 1]);
    double pz = std::stod(argv[base + coord_offset + 2]);

    t.pick.position.x    = px;
    t.pick.position.y    = py;
    t.pick.position.z    = pz;
    t.pick.orientation   = common_q;

    t.pre_pick           = t.pick;
    t.pre_pick.position.z += OFFSET_Z;

    tasks.push_back(std::move(t));
  }

  // Fixed bin location
  geometry_msgs::msg::Pose bin, pre_bin;
  bin.position.x    = 0.0;
  bin.position.y    = 0.7;
  bin.position.z    = 0.02;
  bin.orientation   = common_q;
  pre_bin           = bin;
  pre_bin.position.z += OFFSET_Z;

  // --- ROS2 & MoveIt setup ---
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pick_place_parse");
  auto log  = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thread([&]{ exec.spin(); });

  moveit::planning_interface::MoveGroupInterface arm (node, "ur_manipulator");
  moveit::planning_interface::MoveGroupInterface grip(node, "robotiq_gripper");
  arm.setPoseReferenceFrame("base_link");
  grip.setEndEffectorLink("robotiq_85_left_finger_tip_link");
  arm.setPlanningTime(10.0);
  arm.setMaxVelocityScalingFactor(3.0);
  arm.setMaxAccelerationScalingFactor(3.0);

  auto attach_cli = node->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
  auto detach_cli = node->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");
  if (!attach_cli->wait_for_service(5s) || !detach_cli->wait_for_service(5s)) {
    RCLCPP_ERROR(log, "LinkAttacher services unavailable");
    return 1;
  }

  auto moveToPose = [&](auto &mg, auto const &target) {
    mg.setStartStateToCurrentState();
    mg.setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS ||
        mg.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(log, "Plan/execute failed");
      return false;
    }
    return true;
  };

  // ↑↑↑ INSERT the “go‐to‐home‐pose” block here if you like ↑↑↑

  // --- Execute tasks ---
  for (int i = 0; i < num_tasks; ++i) {
    auto &t = tasks[i];
    RCLCPP_INFO(log,
      "[Task %d] pick '%s' (link '%s') @ (%.2f,%.2f,%.2f) → bin @ (0.0,0.7,0.12)",
      i+1, t.model_name.c_str(), t.link_name.c_str(),
      t.pick.position.x, t.pick.position.y, t.pick.position.z
    );

    // 1) pre-pick
    if (!moveToPose(arm, t.pre_pick)) return 1;

    // 2) open
    grip.setStartStateToCurrentState();
    grip.setJointValueTarget("robotiq_85_left_knuckle_joint", EE_MIN);
    grip.move();

    // 3) pick
    if (!moveToPose(arm, t.pick)) return 1;

    // 4) close
    {
      double cmd = EE_MIN + (EE_MAX - EE_MIN) * (GRIP_PERCENT/100.0);
      grip.setStartStateToCurrentState();
      grip.setJointValueTarget("robotiq_85_left_knuckle_joint", cmd);
      grip.move();
      rclcpp::sleep_for(300ms);
    }

    // 5) attach
    {
      auto req = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
      req->model1_name = "ur";
      req->link1_name  = "robotiq_85_left_finger_tip_link";
      req->model2_name = t.model_name;
      req->link2_name  = t.link_name;
      auto res = attach_cli->async_send_request(req).get();
      if (!res->success) {
        RCLCPP_ERROR(log, "Attach failed: %s", res->message.c_str());
        return 1;
      }
    }

    // 6) lift-to-pre-pick
    if (!moveToPose(arm, t.pre_pick)) return 1;

    // 7) move-to-pre-place
    if (!moveToPose(arm, pre_bin)) return 1;

    // 8) place
    if (!moveToPose(arm, bin)) return 1;

    // 9) detach
    {
      auto req = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
      req->model1_name = "ur";
      req->link1_name  = "robotiq_85_left_finger_tip_link";
      req->model2_name = t.model_name;
      req->link2_name  = t.link_name;
      auto res = detach_cli->async_send_request(req).get();
      if (!res->success) {
        RCLCPP_ERROR(log, "Detach failed: %s", res->message.c_str());
        return 1;
      }
    }

    // 10) open
    grip.setStartStateToCurrentState();
    grip.setJointValueTarget("robotiq_85_left_knuckle_joint", EE_MIN);
    grip.move();
    rclcpp::sleep_for(300ms);

    // 11) back-to-pre-place
    if (!moveToPose(arm, pre_bin)) return 1;
  }

  RCLCPP_INFO(log, "*** All %d tasks done ***", num_tasks);
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
