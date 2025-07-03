// File: src/simple_pick_place.cpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class SimplePickPlace : public rclcpp::Node
{
public:
  SimplePickPlace(const rclcpp::NodeOptions & options)
  : Node("simple_pick_place", options)
  {
    // no manual declare_parameter() calls here!
  }

  void initialize()
  {
    auto node_ptr = shared_from_this();
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_ptr, "ur_manipulator");
    move_group_->setPoseReferenceFrame("base_link");
    move_group_->setEndEffectorLink("wrist_3_link");
    move_group_->setMaxVelocityScalingFactor(0.1);
    move_group_->setMaxAccelerationScalingFactor(0.1);
    RCLCPP_INFO(get_logger(), "MoveGroupInterface initialized");
  }

  void run()
  {
    // Load pick pose (with defaults if not overridden)
    geometry_msgs::msg::Pose pick;
    get_parameter_or("pick_pose.x",  pick.position.x,  0.0);
    get_parameter_or("pick_pose.y",  pick.position.y,  0.0);
    get_parameter_or("pick_pose.z",  pick.position.z,  0.8);
    get_parameter_or("pick_pose.qx", pick.orientation.x, 0.0);
    get_parameter_or("pick_pose.qy", pick.orientation.y, 0.0);
    get_parameter_or("pick_pose.qz", pick.orientation.z, 0.0);
    get_parameter_or("pick_pose.qw", pick.orientation.w, 1.0);

    // Load place pose
    geometry_msgs::msg::Pose place;
    get_parameter_or("place_pose.x",  place.position.x,  0.0);
    get_parameter_or("place_pose.y",  place.position.y,  0.2);
    get_parameter_or("place_pose.z",  place.position.z,  0.8);
    get_parameter_or("place_pose.qx", place.orientation.x, 0.0);
    get_parameter_or("place_pose.qy", place.orientation.y, 0.0);
    get_parameter_or("place_pose.qz", place.orientation.z, 0.0);
    get_parameter_or("place_pose.qw", place.orientation.w, 1.0);

    // 1) Move into pick pose
    RCLCPP_INFO(get_logger(), "Moving to pick pose...");
    executePose(pick);

    // 2) Lift after pick
    pick.position.z += 0.10;
    RCLCPP_INFO(get_logger(), "Lifting after pick...");
    executePose(pick);

    // 3) Move into place pose
    RCLCPP_INFO(get_logger(), "Moving to place pose...");
    executePose(place);

    // 4) Lift after place
    place.position.z += 0.10;
    RCLCPP_INFO(get_logger(), "Lifting after place...");
    executePose(place);

    RCLCPP_INFO(get_logger(), "Pick-and-place sequence complete.");
  }

private:
  void executePose(const geometry_msgs::msg::Pose & target)
  {
    move_group_->setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_->execute(plan);
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to plan to target pose");
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  // still allow overrides, but we don't manually declare anything
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<SimplePickPlace>(options);

  // Spinner for MoveIt
  std::thread spinner([node]() { rclcpp::spin(node); });

  node->initialize();
  node->run();

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
