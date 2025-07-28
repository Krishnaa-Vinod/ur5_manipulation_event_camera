// src/REViTArm_IK.cpp

#include <memory>
#include <chrono>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_interface/planning_interface.h"

using namespace std::chrono_literals;

// Fixed EE orientation constants
static constexpr double PI           = 3.141592653589793;
static constexpr double CONST_ROLL   = -PI;      // –π
static constexpr double CONST_PITCH  = 0.0;      //  0
static constexpr double CONST_YAW    = -PI / 2; // –π/2

// Gripper joint limits
static constexpr double EE_MIN = 0.0;   // fully open
static constexpr double EE_MAX = 0.8;   // fully closed

class REViTArmIKNode : public rclcpp::Node
{
public:
  REViTArmIKNode()
  : Node("revit_arm_ik"),
    latest_grip_(0.0),
    got_pose_(false)
  {
    // 1) TF setup
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "REViTArmIKNode constructed; call init() next");
  }

  void init()
  {
    // 2) Shared-from-this for MoveGroupInterface
    auto self = shared_from_this();

    // 3) MoveIt groups with TF buffer & timeout
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      self, "ur_manipulator", tf_buffer_, rclcpp::Duration(10s));
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      self, "robotiq_gripper", tf_buffer_, rclcpp::Duration(10s));

    // 4) Configure arm planner
    arm_group_->setPoseReferenceFrame("base_link");
    arm_group_->setEndEffectorLink("wrist_3_link");
    arm_group_->setPlanningTime(5.0);
    arm_group_->setMaxVelocityScalingFactor(1.0);
    arm_group_->setMaxAccelerationScalingFactor(1.0);

    // 5) Subscribe to inference outputs
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "revit_arm/ee_pose", 10,
      std::bind(&REViTArmIKNode::poseCallback, this, std::placeholders::_1)
    );
    grip_sub_ = create_subscription<std_msgs::msg::Float32>(
      "revit_arm/ee_gripper", 10,
      std::bind(&REViTArmIKNode::gripCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(),
      "Subscribed to /revit_arm/ee_pose and /revit_arm/ee_gripper; ready to move"
    );
  }

private:
  // Called when a new target pose arrives (we only use position)
  void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    // Take only x,y,z — ignore msg->pose.orientation completely
    latest_pose_.position = msg->pose.position;
    got_pose_            = true;
    doExecute();
  }

  // Called when a new gripper fraction arrives
  void gripCallback(const std_msgs::msg::Float32::ConstSharedPtr &msg)
  {
    latest_grip_ = msg->data;
  }

  // Plan & execute the move with the fixed orientation
  void doExecute()
  {
    if (!got_pose_) return;

    // 1) Stamp in the constant orientation
    tf2::Quaternion q;
    q.setRPY(CONST_ROLL, CONST_PITCH, CONST_YAW);
    latest_pose_.orientation = tf2::toMsg(q);

    // 2) Plan & execute arm movement
    arm_group_->setStartStateToCurrentState();
    arm_group_->setPoseTarget(latest_pose_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
                && (arm_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
      RCLCPP_ERROR(get_logger(), "Arm plan/execute failed");
      return;
    }

    // 3) Command gripper
    double frac = std::clamp(latest_grip_, 0.0, 1.0);
    double cmd  = EE_MIN + (EE_MAX - EE_MIN) * frac;

    gripper_group_->setStartStateToCurrentState();
    gripper_group_->setJointValueTarget("robotiq_85_left_knuckle_joint", cmd);
    if (!gripper_group_->move()) {
      RCLCPP_ERROR(get_logger(), "Gripper move failed");
    }
  }

  // Members
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  std::shared_ptr<tf2_ros::Buffer>   tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr        grip_sub_;

  geometry_msgs::msg::Pose latest_pose_;
  double                   latest_grip_;
  bool                     got_pose_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // 1) Create node
  auto node = std::make_shared<REViTArmIKNode>();

  // 2) Finish setup
  node->init();

  // 3) Spin
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
