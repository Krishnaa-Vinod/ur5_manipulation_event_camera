// src/simple_pick_place.cpp

#include <memory>
#include <thread>
#include <chrono>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

using namespace std::chrono_literals;
static constexpr double PI           = 3.141592653589793;
static constexpr double OFFSET_Z     = 0.05;    // 5 cm lift
static constexpr double GRIP_PERCENT = 70.0;    // close to 70 %
static constexpr double EE_MIN       = 0.0;     // fully open
static constexpr double EE_MAX       = 0.8;     // fully closed

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simple_pick_place");
  auto log  = node->get_logger();

  // ─── Executor & spinner ───────────────────────────
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thread([&exec]{ exec.spin(); });

  // ─── MoveIt groups ─────────────────────────────────
  moveit::planning_interface::MoveGroupInterface arm (node, "ur_manipulator");
  moveit::planning_interface::MoveGroupInterface grip(node, "robotiq_gripper");
  arm.setPoseReferenceFrame(  "base_link");
  arm.setEndEffectorLink(     "wrist_3_link");
  arm.setPlanningTime(        10.0);
  arm.setMaxVelocityScalingFactor(    0.3);
  arm.setMaxAccelerationScalingFactor(0.3);

  // ─── LinkAttacher clients ─────────────────────────
  auto attach_cli = node->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
  auto detach_cli = node->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");
  if (!attach_cli->wait_for_service(5s) || !detach_cli->wait_for_service(5s)) {
    RCLCPP_ERROR(log, "LinkAttacher services unavailable");
    return 1;
  }

  // ─── Planner helper ────────────────────────────────
  auto moveToPose = [&](auto &mg, const geometry_msgs::msg::Pose &target) {
    mg.setStartStateToCurrentState();
    mg.setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS ||
        mg.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(log, "Plan/execute failed");
      return false;
    }
    return true;
  };

  // ─── Define poses ──────────────────────────────────
  geometry_msgs::msg::Pose pick, place, pre_pick, pre_place;
  {
    tf2::Quaternion q; q.setRPY(-PI,0,-PI/2);
    pick.orientation  = tf2::toMsg(q);
    place.orientation = pick.orientation;
    pick.position.x  = 0.5; pick.position.y  = 0.0; pick.position.z  = 0.12;
    place.position.x = 0.5; place.position.y = 0.2; place.position.z = 0.12;
    pre_pick  = pick;  pre_pick.position.z  += OFFSET_Z;
    pre_place = place; pre_place.position.z += OFFSET_Z;
  }

  // 1) Pre-pick
  if (!moveToPose(arm, pre_pick)) return 1;

  // 2) Open gripper
  grip.setStartStateToCurrentState();
  grip.setJointValueTarget("robotiq_85_left_knuckle_joint", EE_MIN);
  grip.move();

  // 3) Descend to pick
  if (!moveToPose(arm, pick)) return 1;

  // 4) Close gripper to GRIP_PERCENT
  {
    double GP  = (EE_MAX - EE_MIN) * (GRIP_PERCENT/100.0);
    double cmd = EE_MIN + GP;
    RCLCPP_INFO(log, "Closing gripper to %.1f%% -> %.3f rad", GRIP_PERCENT, cmd);
    grip.setStartStateToCurrentState();
    grip.setJointValueTarget("robotiq_85_left_knuckle_joint", cmd);
    grip.move();
    rclcpp::sleep_for(300ms);  // let it settle
  }

  // 5) Attach at finger tip
  {
    auto req = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
    req->model1_name = "ur";
    req->link1_name  = "robotiq_85_left_finger_tip_link";
    req->model2_name = "box";
    req->link2_name  = "box";
    auto res = attach_cli->async_send_request(req).get();
    if (!res->success) {
      RCLCPP_ERROR(log, "Attach failed: %s", res->message.c_str());
      return 1;
    }
  }

  // 6) Lift back to pre-pick
  if (!moveToPose(arm, pre_pick)) return 1;

  // 7) Move to pre-place
  if (!moveToPose(arm, pre_place)) return 1;

  // 8) Descend to place
  if (!moveToPose(arm, place)) return 1;

  // 9) Detach
  {
    auto req = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
    req->model1_name = "ur";
    req->link1_name  = "robotiq_85_left_finger_tip_link";
    req->model2_name = "box";
    req->link2_name  = "box";
    auto res = detach_cli->async_send_request(req).get();
    if (!res->success) {
      RCLCPP_ERROR(log, "Detach failed: %s", res->message.c_str());
      return 1;
    }
  }

  // 10) **Open gripper after detach**
  RCLCPP_INFO(log, "Opening gripper after detach");
  grip.setStartStateToCurrentState();
  grip.setJointValueTarget("robotiq_85_left_knuckle_joint", EE_MIN);
  grip.move();
  rclcpp::sleep_for(300ms);

  // 11) Return to pre-place
  moveToPose(arm, pre_place);

  RCLCPP_INFO(log, "*** Pick-and-place done ***");
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
