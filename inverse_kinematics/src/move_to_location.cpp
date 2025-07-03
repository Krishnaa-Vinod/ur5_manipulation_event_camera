// File: src/move_to_location.cpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_to_location");

class MoveToLocationNode : public rclcpp::Node
{
public:
  MoveToLocationNode(const rclcpp::NodeOptions & options)
  : Node("move_to_location", options)
  {
    // Declare parameters with defaults
    declare_parameter("target_pose.x", 0.0);
    declare_parameter("target_pose.y", 0.2);
    declare_parameter("target_pose.z", 0.8);
    declare_parameter("target_pose.qx", 0.0);
    declare_parameter("target_pose.qy", 0.0);
    declare_parameter("target_pose.qz", 0.0);
    declare_parameter("target_pose.qw", 1.0);
  }

  void initialize()
  {
    auto self = shared_from_this();
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      self, "ur_manipulator");
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      self, "robotiq_gripper");

    // Arm configuration
    arm_group_->setPoseReferenceFrame("base_link");
    arm_group_->setEndEffectorLink("wrist_3_link");
    arm_group_->setMaxVelocityScalingFactor(0.2);
    arm_group_->setMaxAccelerationScalingFactor(0.2);
    arm_group_->setPlanningTime(10.0);

    // Gripper configuration: assume single joint named 'robotiq_85_left_knuckle_joint'
    gripper_group_->setMaxVelocityScalingFactor(1.0);
    gripper_group_->setMaxAccelerationScalingFactor(1.0);
    gripper_group_->setPlanningTime(2.0);

    RCLCPP_INFO(LOGGER, "MoveGroupInterfaces initialized");
  }

  void moveToTarget()
  {
    // Read target pose
    geometry_msgs::msg::Pose target;
    get_parameter("target_pose.x", target.position.x);
    get_parameter("target_pose.y", target.position.y);
    get_parameter("target_pose.z", target.position.z);
    get_parameter("target_pose.qx", target.orientation.x);
    get_parameter("target_pose.qy", target.orientation.y);
    get_parameter("target_pose.qz", target.orientation.z);
    get_parameter("target_pose.qw", target.orientation.w);

    // Override to face down
    tf2::Quaternion q;
    q.setRPY(-M_PI_2, 0.0, 0.0);
    target.orientation = tf2::toMsg(q);

    RCLCPP_INFO_STREAM(LOGGER, "Planning arm to "
      << target.position.x << ", "
      << target.position.y << ", "
      << target.position.z);

    // Plan and move arm
    arm_group_->setStartStateToCurrentState();
    arm_group_->clearPoseTargets();
    arm_group_->setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    if (arm_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      arm_group_->execute(arm_plan);
    } else {
      RCLCPP_ERROR(LOGGER, "Arm planning failed");
      return;
    }

    // Close gripper
    RCLCPP_INFO(LOGGER, "Closing gripper");
    controlGripper(0.785);

  
    arm_group_->setStartStateToCurrentState();
    arm_group_->setPoseTarget(target);
    arm_group_->move();
  }

  void controlGripper(double position)
  {
    // Set the gripper joint to 'position' (radians)
    gripper_group_->setJointValueTarget(
      std::vector<double>{position}
    );
    gripper_group_->move();
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToLocationNode>(rclcpp::NodeOptions());
  std::thread spin_thread([&]() { rclcpp::spin(node); });

  node->initialize();
  node->moveToTarget();

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
