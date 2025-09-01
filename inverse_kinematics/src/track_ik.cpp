// src/track_ik.cpp

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

static constexpr double PICK_HEIGHT      = 0.012;  // original pick z
static constexpr double OFFSET_Z         = 0.250;   // lift above pick
static constexpr double TRACK_VEL_SCALE  = 0.5;    // slow tracking
static constexpr double TRACK_ACC_SCALE  = 0.5;

class ArmTracker : public rclcpp::Node
{
public:
  ArmTracker()
  : Node("revit_arm_tracker"),
    arm_initialized_(false)
  {
    // fixed end‐effector orientation
    tf2::Quaternion q;
    q.setRPY(-M_PI, 0, -M_PI/2);
    common_ori_ = tf2::toMsg(q);

    // subscribe to inference topic
    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "revit_arm_xy_inference", 10,
      std::bind(&ArmTracker::on_pose, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "ArmTracker node started, waiting for PoseStamped...");
  }

private:
  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // lazy init MoveGroupInterface
    if (!arm_initialized_) {
      auto node_ptr = this->shared_from_this();  // now unambiguous
      arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        node_ptr, "ur_manipulator"
      );
      arm_->setPoseReferenceFrame("base_link");
      arm_->setEndEffectorLink("wrist_3_link");
      arm_->setPlanningTime(5.0);
      arm_->setMaxVelocityScalingFactor(TRACK_VEL_SCALE);
      arm_->setMaxAccelerationScalingFactor(TRACK_ACC_SCALE);
      arm_initialized_ = true;
      RCLCPP_INFO(get_logger(), "MoveGroupInterface initialized.");
    }

    // build target at pre-pick height
    geometry_msgs::msg::Pose target;
    target.orientation = common_ori_;
    target.position.x = msg->pose.position.x;
    target.position.y = msg->pose.position.y;
    target.position.z = PICK_HEIGHT + OFFSET_Z;

    RCLCPP_INFO(get_logger(),
      "Tracking → moving to x=%.3f, y=%.3f, z=%.3f",
      target.position.x, target.position.y, target.position.z
    );

    // plan & execute
    arm_->setStartStateToCurrentState();
    arm_->setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
           && (arm_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!ok) {
      RCLCPP_WARN(get_logger(),
        "Tracking plan failed for x=%.3f, y=%.3f",
        target.position.x, target.position.y
      );
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr   sub_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface>   arm_;
  geometry_msgs::msg::Quaternion                                    common_ori_;
  bool                                                              arm_initialized_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto tracker = std::make_shared<ArmTracker>();
  rclcpp::spin(tracker);
  rclcpp::shutdown();
  return 0;
}
