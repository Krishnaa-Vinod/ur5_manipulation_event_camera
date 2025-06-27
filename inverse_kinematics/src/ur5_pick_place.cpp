#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <memory>
#include <string>
#include <vector>
#include <thread>

// Define constants for readability
const std::string UR_MANIPULATOR_GROUP = "ur_manipulator";
const std::string ROBOTIQ_GRIPPER_GROUP = "robotiq_gripper";
const std::string ATTACH_SERVICE = "/ATTACHLINK";
const std::string DETACH_SERVICE = "/DETACHLINK";

class PickAndPlaceNode : public rclcpp::Node {
public:
    PickAndPlaceNode() : Node("ur5_pick_place_node") {}

    // Initializes all the necessary MoveIt and Gazebo components
    bool initialize() {
        RCLCPP_INFO(get_logger(), "Initializing Pick and Place Node...");
        
        ur_manipulator_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), UR_MANIPULATOR_GROUP);
        robotiq_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), ROBOTIQ_GRIPPER_GROUP);
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        attach_client_ = create_client<linkattacher_msgs::srv::AttachLink>(ATTACH_SERVICE);
        detach_client_ = create_client<linkattacher_msgs::srv::DetachLink>(DETACH_SERVICE);

        if (!attach_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_logger(), "Link Attacher service is not available.");
            return false;
        }
        RCLCPP_INFO(get_logger(), "Initialization complete.");
        return true;
    }

    // This is the main function where all the logic happens
    void run_pick_place() {
        RCLCPP_INFO(get_logger(), "Starting pick and place sequence.");
        rclcpp::sleep_for(std::chrono::seconds(2));

        // ===================================================================
        // 1. SETUP THE SCENE
        // ===================================================================

        // Add the box to MoveIt's planning scene so it knows about the object
        RCLCPP_INFO(get_logger(), "Adding box to the planning scene.");
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = ur_manipulator_->getPlanningFrame();
        collision_object.id = "box";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {0.03, 0.03, 0.03};

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = 0.76;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;
        planning_scene_interface_->addCollisionObjects({collision_object});
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // Set the planner and motion speed
        ur_manipulator_->setPlannerId("pilz_industrial_motion_planner");
        ur_manipulator_->setMaxVelocityScalingFactor(0.5);
        ur_manipulator_->setMaxAccelerationScalingFactor(0.5);

        // Define the target poses
        tf2::Quaternion orientation;
        orientation.setRPY(M_PI, 0, 0); // Pointing down
        geometry_msgs::msg::Pose pre_pick_pose, pick_pose, pre_place_pose, place_pose;
        pre_pick_pose.orientation = pick_pose.orientation = pre_place_pose.orientation = place_pose.orientation = tf2::toMsg(orientation);

        // CORRECTED: Assign position members one by one
        pre_pick_pose.position.x = 0.0;
        pre_pick_pose.position.y = 0.0;
        pre_pick_pose.position.z = 0.86;

        pick_pose.position.x = 0.0;
        pick_pose.position.y = 0.0;
        pick_pose.position.z = 0.76;

        pre_place_pose.position.x = 0.0;
        pre_place_pose.position.y = 0.2;
        pre_place_pose.position.z = 0.86;

        place_pose.position.x = 0.0;
        place_pose.position.y = 0.2;
        place_pose.position.z = 0.76;

        // ===================================================================
        // 2. EXECUTE THE PICK SEQUENCE
        // ===================================================================
        
        RCLCPP_INFO(get_logger(), "--- PICK SEQUENCE ---");

        // Open the gripper
        robotiq_gripper_->setNamedTarget("open");
        robotiq_gripper_->move();

        // Move to pre-pick pose
        ur_manipulator_->setPoseTarget(pre_pick_pose);
        ur_manipulator_->move();

        // Move to pick pose
        ur_manipulator_->setPoseTarget(pick_pose);
        ur_manipulator_->move();

        // Close the gripper and attach the object
        robotiq_gripper_->setNamedTarget("closed");
        robotiq_gripper_->move();
        ur_manipulator_->attachObject("box", "wrist_3_link"); // Attach in MoveIt
        
        auto attach_req = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
        attach_req->model1_name = "ur"; attach_req->link1_name = "wrist_3_link";
        attach_req->model2_name = "box"; attach_req->link2_name = "link";
        attach_client_->async_send_request(attach_req); // Attach in Gazebo
        rclcpp::sleep_for(std::chrono::milliseconds(500));


        // ===================================================================
        // 3. EXECUTE THE PLACE SEQUENCE
        // ===================================================================

        RCLCPP_INFO(get_logger(), "--- PLACE SEQUENCE ---");

        // Retreat from pick pose
        ur_manipulator_->setPoseTarget(pre_pick_pose);
        ur_manipulator_->move();

        // Move to pre-place pose
        ur_manipulator_->setPoseTarget(pre_place_pose);
        ur_manipulator_->move();

        // Move to place pose
        ur_manipulator_->setPoseTarget(place_pose);
        ur_manipulator_->move();

        // Open the gripper and detach the object
        robotiq_gripper_->setNamedTarget("open");
        robotiq_gripper_->move();
        ur_manipulator_->detachObject("box"); // Detach in MoveIt

        auto detach_req = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
        detach_req->model1_name = "ur"; detach_req->link1_name = "wrist_3_link";
        detach_req->model2_name = "box"; detach_req->link2_name = "link";
        detach_client_->async_send_request(detach_req); // Detach in Gazebo
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // Retreat from place pose
        ur_manipulator_->setPoseTarget(pre_place_pose);
        ur_manipulator_->move();

        RCLCPP_INFO(get_logger(), "Pick and place sequence finished successfully!");
        rclcpp::shutdown();
    }

private:
    // Member variables
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> ur_manipulator_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> robotiq_gripper_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client_;
    rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickAndPlaceNode>();
    
    if (!node->initialize()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize the pick and place node. Shutting down.");
        rclcpp::shutdown();
        return 1;
    }

    // Run the main logic in a background thread to not block the executor
    std::thread pick_place_thread([node]() {
        node->run_pick_place();
    });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    pick_place_thread.join();
    rclcpp::shutdown();
    return 0;
}


// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <linkattacher_msgs/srv/attach_link.hpp>
// #include <linkattacher_msgs/srv/detach_link.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <moveit_msgs/msg/collision_object.hpp>
// #include <moveit_msgs/msg/orientation_constraint.hpp>

// #include <memory>
// #include <string>
// #include <vector>
// #include <thread>

// // Define constants
// const std::string UR_MANIPULATOR_GROUP = "ur_manipulator";
// const std::string ROBOTIQ_GRIPPER_GROUP = "robotiq_gripper";
// const std::string ATTACH_SERVICE = "/ATTACHLINK";
// const std::string DETACH_SERVICE = "/DETACHLINK";

// class PickAndPlaceNode : public rclcpp::Node {
// public:
//     PickAndPlaceNode() : Node("ur5_pick_place_node") {}

//     bool initialize() {
//         RCLCPP_INFO(get_logger(), "Initializing Pick and Place Node...");
        
//         ur_manipulator_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), UR_MANIPULATOR_GROUP);
//         robotiq_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), ROBOTIQ_GRIPPER_GROUP);
//         planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

//         attach_client_ = create_client<linkattacher_msgs::srv::AttachLink>(ATTACH_SERVICE);
//         detach_client_ = create_client<linkattacher_msgs::srv::DetachLink>(DETACH_SERVICE);

//         if (!attach_client_->wait_for_service(std::chrono::seconds(5))) {
//             RCLCPP_ERROR(get_logger(), "Link Attacher service is not available.");
//             return false;
//         }
//         RCLCPP_INFO(get_logger(), "Initialization complete.");
//         return true;
//     }

//     void addBoxToScene() {
//         if (!rclcpp::ok()) return;
//         RCLCPP_INFO(get_logger(), "Adding box to the planning scene.");

//         moveit_msgs::msg::CollisionObject collision_object;
//         collision_object.header.frame_id = ur_manipulator_->getPlanningFrame();
//         collision_object.id = "box";

//         shape_msgs::msg::SolidPrimitive primitive;
//         primitive.type = primitive.BOX;
//         primitive.dimensions.resize(3);
//         primitive.dimensions[0] = 0.03;
//         primitive.dimensions[1] = 0.03;
//         primitive.dimensions[2] = 0.03;

//         geometry_msgs::msg::Pose box_pose;
//         box_pose.orientation.w = 1.0;
//         box_pose.position.x = 0.0;
//         box_pose.position.y = 0.0;
//         box_pose.position.z = 0.76;

//         collision_object.primitives.push_back(primitive);
//         collision_object.primitive_poses.push_back(box_pose);
//         collision_object.operation = collision_object.ADD;

//         std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
//         collision_objects.push_back(collision_object);

//         planning_scene_interface_->addCollisionObjects(collision_objects);
//         rclcpp::sleep_for(std::chrono::milliseconds(500));
//     }

//     void run_pick_place() {
//         RCLCPP_INFO(get_logger(), "Starting pick and place sequence.");
//         rclcpp::sleep_for(std::chrono::seconds(2));

//         addBoxToScene();
        
//         ur_manipulator_->setPlannerId("pilz_industrial_motion_planner");

//         tf2::Quaternion orientation;
//         orientation.setRPY(M_PI, 0, 0);
//         geometry_msgs::msg::Pose pre_pick_pose, pick_pose, pre_place_pose, place_pose;
//         pre_pick_pose.orientation = pick_pose.orientation = pre_place_pose.orientation = place_pose.orientation = tf2::toMsg(orientation);

//         pre_pick_pose.position.x = 0.0;
//         pre_pick_pose.position.y = 0.0;
//         pre_pick_pose.position.z = 0.86;

//         pick_pose.position.x = 0.0;
//         pick_pose.position.y = 0.0;
//         pick_pose.position.z = 0.76;

//         pre_place_pose.position.x = 0.0;
//         pre_place_pose.position.y = 0.2;
//         pre_place_pose.position.z = 0.86;

//         place_pose.position.x = 0.0;
//         place_pose.position.y = 0.2;
//         place_pose.position.z = 0.76;

//         setGripperState("open");
//         moveToPose(pre_pick_pose, "pre-pick");
//         moveToPose(pick_pose, "pick");
        
//         ur_manipulator_->attachObject("box", "wrist_3_link");
        
//         setGripperState("closed");
//         attach("ur", "wrist_3_link", "box", "link");
        
//         moveToPose(pre_pick_pose, "retreat from pick");
//         moveToPose(pre_place_pose, "pre-place");
//         moveToPose(place_pose, "place");
        
//         ur_manipulator_->detachObject("box");
        
//         setGripperState("open");
//         detach("ur", "wrist_3_link", "box", "link");
        
//         moveToPose(pre_place_pose, "retreat from place");

//         RCLCPP_INFO(get_logger(), "Pick and place sequence finished successfully!");
//         rclcpp::shutdown();
//     }

// private:
//     void moveToPose(const geometry_msgs::msg::Pose& target_pose, const std::string& pose_name) {
//         if (!rclcpp::ok()) return;
//         RCLCPP_INFO(get_logger(), "Moving to %s pose", pose_name.c_str());
//         ur_manipulator_->setPoseTarget(target_pose);
//         if (ur_manipulator_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
//              RCLCPP_ERROR(get_logger(), "Failed to move to %s pose", pose_name.c_str());
//         }
//     }

//     void setGripperState(const std::string& state) {
//         if (!rclcpp::ok()) return;
//         RCLCPP_INFO(get_logger(), "Setting gripper to '%s'", state.c_str());
//         robotiq_gripper_->setNamedTarget(state);
//         if (robotiq_gripper_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
//             RCLCPP_ERROR(get_logger(), "Failed to set gripper to %s", state.c_str());
//         }
//     }
    
//     void attach(const std::string& robot_model, const std::string& robot_link, const std::string& object_model, const std::string& object_link) {
//         if (!rclcpp::ok()) return;
//         RCLCPP_INFO(get_logger(), "Attaching '%s' to robot's '%s' in Gazebo", object_model.c_str(), robot_link.c_str());
//         auto req = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
        
//         req->model1_name = robot_model;
//         req->link1_name = robot_link;
//         req->model2_name = object_model;
//         req->link2_name = object_link;

//         attach_client_->async_send_request(req);
//         rclcpp::sleep_for(std::chrono::milliseconds(500));
//     }

//     void detach(const std::string& robot_model, const std::string& robot_link, const std::string& object_model, const std::string& object_link) {
//         if (!rclcpp::ok()) return;
//         RCLCPP_INFO(get_logger(), "Detaching '%s' from robot's '%s' in Gazebo", object_model.c_str(), robot_link.c_str());
//         auto req = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
        
//         req->model1_name = robot_model;
//         req->link1_name = robot_link;
//         req->model2_name = object_model;
//         req->link2_name = object_link;

//         detach_client_->async_send_request(req);
//         rclcpp::sleep_for(std::chrono::milliseconds(500));
//     }

//     // Member variables
//     std::shared_ptr<moveit::planning_interface::MoveGroupInterface> ur_manipulator_;
//     // ** THIS LINE IS NOW CORRECTED **
//     std::shared_ptr<moveit::planning_interface::MoveGroupInterface> robotiq_gripper_;
//     std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
//     rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client_;
//     rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client_;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<PickAndPlaceNode>();
    
//     if (!node->initialize()) {
//         RCLCPP_ERROR(node->get_logger(), "Failed to initialize the pick and place node. Shutting down.");
//         rclcpp::shutdown();
//         return 1;
//     }

//     std::thread pick_place_thread([node]() {
//         node->run_pick_place();
//     });

//     rclcpp::executors::SingleThreadedExecutor executor;
//     executor.add_node(node);
//     executor.spin();

//     pick_place_thread.join();
//     rclcpp::shutdown();
//     return 0;
// }