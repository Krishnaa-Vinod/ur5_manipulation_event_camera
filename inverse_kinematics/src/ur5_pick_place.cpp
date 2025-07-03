#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <atomic>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur5_pick_place");

class UR5PickPlace : public rclcpp::Node
{
public:
  UR5PickPlace(const rclcpp::NodeOptions & options)
  : Node("ur5_pick_place", options)
  {}

  void initialize()
  {
    auto node_ptr = shared_from_this();

    // Arm and gripper interfaces
    move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_ptr, "ur_manipulator");
    move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_ptr, "robotiq_gripper");

  
    move_group_arm_->setMaxVelocityScalingFactor(0.2);
    move_group_arm_->setMaxAccelerationScalingFactor(0.2);

    // **Set reference frame and end effector link**
    move_group_arm_->setPoseReferenceFrame("base_link");
    move_group_arm_->setEndEffectorLink("wrist_3_link");


    // Gazebo service for attaching/detaching
    gazebo_state_client_ =
      this->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");
    while (!gazebo_state_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(LOGGER, "Interrupted waiting for Gazebo service.");
        return;
      }
      RCLCPP_INFO(LOGGER, "Waiting for /gazebo/set_entity_state...");
    }

    RCLCPP_INFO(LOGGER, "Initialization complete.");
    initialization_complete_ = true;
  }

  bool isInitialized() const { return initialization_complete_; }

  void run()
  {
    RCLCPP_INFO(LOGGER, "Starting pick & place sequence");
    move_to_home();
    open_gripper();

    move_to_pick_approach();
    move_to_pick_pose();
    close_gripper();
    attach_object_to_gripper();
    move_to_pick_retreat();

    move_to_place_approach();
    move_to_place_pose();
    open_gripper();
    detach_object_from_gripper();
    move_to_place_retreat();

    RCLCPP_INFO(LOGGER, "Sequence finished.");
  }

private:
  // Constants
  const std::string BOX_ID       = "box";
  const double      BOX_SIZE     = 0.03;
  const double      GRIPPER_OPEN = 0.8;
  const double      GRIPPER_CLOSED = 0.0;

  // Interfaces
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr gazebo_state_client_;
  std::atomic<bool> initialization_complete_{false};



  void move_to_home()
  {
    RCLCPP_INFO(LOGGER, "Moving to home.");
    move_group_arm_->setJointValueTarget(
      move_group_arm_->getNamedTargetValues("home"));
    plan_and_execute_arm();
  }

  void set_gripper(double pos)
  {
    move_group_gripper_->setJointValueTarget("robotiq_85_left_knuckle_joint", pos);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_gripper_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_gripper_->execute(plan);
    } else {
      RCLCPP_ERROR(LOGGER, "Gripper plan failed.");
    }
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  void open_gripper()  { set_gripper(GRIPPER_OPEN);  }
  void close_gripper() { set_gripper(GRIPPER_CLOSED); }

  void move_to_pick_approach()
  {
    geometry_msgs::msg::Pose target;
    target.orientation.w = 1.0;
    target.position.z = 0.76 + 0.10;
    move_group_arm_->setPoseTarget(target);
    plan_and_execute_arm();
  }

  void move_to_pick_pose()
  {
    geometry_msgs::msg::Pose target;
    target.orientation.w = 1.0;
    target.position.z = 0.76 + 0.02;
    move_group_arm_->setPoseTarget(target);
    plan_and_execute_arm();
  }

  void move_to_pick_retreat()
  {
    geometry_msgs::msg::Pose target;
    target.orientation.w = 1.0;
    target.position.z = 0.76 + 0.10;
    move_group_arm_->setPoseTarget(target);
    plan_and_execute_arm();
  }

  void attach_object_to_gripper()
  {
    move_group_arm_->attachObject(BOX_ID, "wrist_3_link");
    auto req = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    req->state.name = BOX_ID;
    req->state.reference_frame = "wrist_3_link";
    req->state.pose.position.z = BOX_SIZE / 2;
    req->state.pose.orientation.w = 1.0;
    auto fut = gazebo_state_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(shared_from_this(), fut, std::chrono::seconds(2))
        == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(LOGGER, "Attached in Gazebo.");
    } else {
      RCLCPP_ERROR(LOGGER, "Gazebo attach failed.");
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  void detach_object_from_gripper()
  {
    move_group_arm_->detachObject(BOX_ID);
    auto req = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    req->state.name = BOX_ID;
    req->state.reference_frame = "";
    req->state.pose.position.x = 0.0;
    req->state.pose.position.y = 0.2;
    req->state.pose.position.z = 0.76;
    req->state.pose.orientation.w = 1.0;
    auto fut = gazebo_state_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(shared_from_this(), fut, std::chrono::seconds(2))
        == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(LOGGER, "Detached in Gazebo.");
    } else {
      RCLCPP_ERROR(LOGGER, "Gazebo detach failed.");
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  void move_to_place_approach()
  {
    geometry_msgs::msg::Pose target;
    target.orientation.w = 1.0;
    target.position.y = 0.2;
    target.position.z = 0.76 + 0.10;
    move_group_arm_->setPoseTarget(target);
    plan_and_execute_arm();
  }

  void move_to_place_pose()
  {
    geometry_msgs::msg::Pose target;
    target.orientation.w = 1.0;
    target.position.y = 0.2;
    target.position.z = 0.76 + 0.02;
    move_group_arm_->setPoseTarget(target);
    plan_and_execute_arm();
  }

  void move_to_place_retreat()
  {
    geometry_msgs::msg::Pose target;
    target.orientation.w = 1.0;
    target.position.y = 0.2;
    target.position.z = 0.76 + 0.10;
    move_group_arm_->setPoseTarget(target);
    plan_and_execute_arm();
  }

  void plan_and_execute_arm()
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_arm_->execute(plan);
    } else {
      RCLCPP_ERROR(LOGGER, "Arm planning failed.");
    }
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<UR5PickPlace>(opts);
  std::thread spinner([&]() { rclcpp::spin(node); });

  node->initialize();
  if (node->isInitialized() && rclcpp::ok()) {
    node->run();
  } else {
    RCLCPP_ERROR(LOGGER, "Initialization failed. Exiting.");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
