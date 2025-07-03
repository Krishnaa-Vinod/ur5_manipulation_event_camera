#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <cmath> // For M_PI

// Define a logger for easy printing
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_to_joint_goal");

// Function to convert degrees to radians
double deg_to_rad(double degrees) {
    return degrees * M_PI / 180.0;
}

int main(int argc, char **argv) {
    // 1. Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // 2. Create a ROS 2 node
    // We use automatically_declare_parameters_from_overrides(true) to handle parameters passed from launch files
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = std::make_shared<rclcpp::Node>("move_to_joint_goal_node", node_options);
    
    // We spin the node in a separate thread so we can wait for the action to complete
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // 3. Create a MoveGroupInterface object for the arm
    // This object is the main interface for planning and executing motions
    static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
    auto move_group_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, PLANNING_GROUP_ARM);

    // Set a lower velocity and acceleration to make the movement smoother and more observable
    move_group_arm->setMaxVelocityScalingFactor(0.2);
    move_group_arm->setMaxAccelerationScalingFactor(0.2);

    // 4. Define the target joint values in RADIANS
    // The values are taken from your screenshots and converted from degrees
    std::vector<double> joint_group_positions = {
        deg_to_rad(-11.0), // shoulder_pan_joint
        deg_to_rad(-75.0), // shoulder_lift_joint
        deg_to_rad(95.0),  // elbow_joint
        deg_to_rad(-113.0),// wrist_1_joint
        deg_to_rad(-89.0), // wrist_2_joint
        deg_to_rad(-11.0)  // wrist_3_joint
    };

    // 5. Set the joint value target
    RCLCPP_INFO(LOGGER, "Setting joint value target");
    move_group_arm->setJointValueTarget(joint_group_positions);

    // 6. Plan the motion
    RCLCPP_INFO(LOGGER, "Planning motion to joint goal...");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_arm->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // 7. Execute the motion if the plan was successful
    if (success) {
        RCLCPP_INFO(LOGGER, "Plan successful! Executing motion...");
        move_group_arm->execute(my_plan);
    } else {
        RCLCPP_ERROR(LOGGER, "Failed to plan motion to joint goal.");
    }

    // 8. Shutdown ROS
    rclcpp::shutdown();
    return 0;
}