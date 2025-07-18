// src/pick_place_yolo.cpp

#include <memory>
#include <thread>
#include <chrono>
#include <iostream>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pick_place_yolo");
  auto log  = node->get_logger();

  // Variables for received position
  double x_pos = 0.0;
  double y_pos = 0.0;
  bool received = false;

  // Subscribe to the world‐frame pose published by your YOLO node
  auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/object_world_pose", 10,
    [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      if (!received) {
        x_pos = msg->pose.position.x;
        y_pos = msg->pose.position.y;
        received = true;
        RCLCPP_INFO(log, "Received YOLO world pose: x=%.3f, y=%.3f", x_pos, y_pos);
      }
    }
  );

  // Executor and spinner thread
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thread([&]{ exec.spin(); });

  // Wait for initial position
  RCLCPP_INFO(log, "Waiting for initial YOLO pose...");
  while (rclcpp::ok() && !received) {
    std::this_thread::sleep_for(100ms);
  }

  // Common end‐effector orientation
  tf2::Quaternion q;
  q.setRPY(-PI, 0, -PI/2);
  auto common_q = tf2::toMsg(q);

  // Build the single pick task
  std::vector<PickTask> tasks;
  tasks.reserve(1);
  PickTask t;
  t.model_name = "box";
  t.link_name  = "box";
  t.pick.position.x  = x_pos;
  t.pick.position.y  = y_pos;
  t.pick.position.z  = 0.012;
  t.pick.orientation = common_q;
  t.pre_pick         = t.pick;
  t.pre_pick.position.z += OFFSET_Z;
  tasks.push_back(std::move(t));

  // Fixed bin location
  geometry_msgs::msg::Pose bin, pre_bin;
  bin.position.x    = 0.0;
  bin.position.y    = 0.7;
  bin.position.z    = 0.02;
  bin.orientation   = common_q;
  pre_bin           = bin;
  pre_bin.position.z += OFFSET_Z;

  // --- ROS2 & MoveIt setup ---
  moveit::planning_interface::MoveGroupInterface arm (node, "ur_manipulator");
  moveit::planning_interface::MoveGroupInterface grip(node, "robotiq_gripper");
  arm.setPoseReferenceFrame("base_link");
  arm.setEndEffectorLink("wrist_3_link");
  arm.setPlanningTime(10.0);
  arm.setMaxVelocityScalingFactor(3.0);
  arm.setMaxAccelerationScalingFactor(3.0);

  auto attach_cli = node->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
  auto detach_cli = node->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");
  if (!attach_cli->wait_for_service(5s) || !detach_cli->wait_for_service(5s)) {
    RCLCPP_ERROR(log, "LinkAttacher services unavailable");
    return 1;
  }

  // Utility for motion planning
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

  // --- Execute pick and place ---
  for (size_t i = 0; i < tasks.size(); ++i) {
    auto &task = tasks[i];
    RCLCPP_INFO(log,
      "[Task %zu] pick '%s' (link '%s') @ (%.2f,%.2f,%.2f) → bin @ (0.0,0.7,0.12)",
      i+1, task.model_name.c_str(), task.link_name.c_str(),
      task.pick.position.x, task.pick.position.y, task.pick.position.z
    );

    // 1) pre-pick
    if (!moveToPose(arm, task.pre_pick)) return 1;

    // 2) open
    grip.setStartStateToCurrentState();
    grip.setJointValueTarget("robotiq_85_left_knuckle_joint", EE_MIN);
    grip.move();

    // 3) pick
    if (!moveToPose(arm, task.pick)) return 1;

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
      req->model2_name = task.model_name;
      req->link2_name  = task.link_name;
      auto res = attach_cli->async_send_request(req).get();
      if (!res->success) {
        RCLCPP_ERROR(log, "Attach failed: %s", res->message.c_str());
        return 1;
      }
    }

    // 6) lift-to-pre-pick
    if (!moveToPose(arm, task.pre_pick)) return 1;

    // 7) move-to-pre-place
    if (!moveToPose(arm, pre_bin)) return 1;

    // 8) place
    if (!moveToPose(arm, bin)) return 1;

    // 9) detach
    {
      auto req = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
      req->model1_name = "ur";
      req->link1_name  = "robotiq_85_left_finger_tip_link";
      req->model2_name = task.model_name;
      req->link2_name  = task.link_name;
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

  RCLCPP_INFO(log, "*** Pick and place sequence completed ***");
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
