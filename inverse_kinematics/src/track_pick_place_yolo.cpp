// src/track_pick_place.cpp

#include <memory>
#include <chrono>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

using namespace std::chrono_literals;

static constexpr double PI       = 3.141592653589793;
static constexpr double OFFSET_Z = 0.3;    // 30 cm above pick height
static constexpr double FIXED_Z  = 0.012;  // table clearance + small lift
static constexpr double Kp       = 1.0;    // gain for Cartesian servo

class TrackPickPlace
{
public:
  explicit TrackPickPlace(rclcpp::Node::SharedPtr node)
  : node_(std::move(node)),
    arm_(node_, "ur_manipulator"),
    grip_(node_, "robotiq_gripper"),
    received_(false),
    moved_to_prepick_(false)
  {
    // Subscribe to YOLO object pose
    sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/object_world_pose", 10,
      std::bind(&TrackPickPlace::poseCallback, this, std::placeholders::_1)
    );

    // Publisher for MoveIt Servo velocity commands
    servo_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_server/delta_twist_cmds", 10
    );

    // Configure MoveIt
    arm_.setPoseReferenceFrame("base_link");
    arm_.setEndEffectorLink("wrist_3_link");
    arm_.setPlanningTime(5.0);

    // 50 Hz servo loop
    servo_timer_ = node_->create_wall_timer(
      20ms, std::bind(&TrackPickPlace::servoLoop, this)
    );

    RCLCPP_INFO(node_->get_logger(), "TrackPickPlace ready.");
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!received_) {
      // On first detection, plan+execute to pre-pick pose
      geometry_msgs::msg::Pose target = msg->pose;
      target.position.z = FIXED_Z + OFFSET_Z;
      tf2::Quaternion q;
      q.setRPY(-PI, 0, -PI/2);
      target.orientation = tf2::toMsg(q);

      arm_.setStartStateToCurrentState();
      arm_.setPoseTarget(target);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (arm_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS &&
          arm_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(node_->get_logger(), "Moved to pre‑pick pose.");
        moved_to_prepick_ = true;
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to move to pre‑pick.");
      }

      received_ = true;
    }

    // Always update latest pose
    latest_pose_ = *msg;
  }

  void servoLoop()
  {
    if (!(received_ && moved_to_prepick_)) {
      return;  // wait until we’re in pre-pick
    }

    // Get current end-effector pose
    auto current = arm_.getCurrentPose();

    // Compute XY error
    double ex = latest_pose_.pose.position.x - current.pose.position.x;
    double ey = latest_pose_.pose.position.y - current.pose.position.y;

    // Publish Cartesian velocity command
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = node_->now();
    twist.header.frame_id = arm_.getPoseReferenceFrame();
    twist.twist.linear.x  = Kp * ex;
    twist.twist.linear.y  = Kp * ey;
    twist.twist.linear.z  = 0.0;  // hold height
    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = 0.0;

    servo_pub_->publish(twist);
  }

  // Member declarations (order governs initialization order)
  rclcpp::Node::SharedPtr                                        node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr   servo_pub_;
  rclcpp::TimerBase::SharedPtr                                   servo_timer_;
  moveit::planning_interface::MoveGroupInterface                 arm_;
  moveit::planning_interface::MoveGroupInterface                 grip_;
  std::atomic<bool>                                              received_;
  std::atomic<bool>                                              moved_to_prepick_;
  geometry_msgs::msg::PoseStamped                                latest_pose_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("track_pick_place");
  auto tracker = std::make_shared<TrackPickPlace>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
