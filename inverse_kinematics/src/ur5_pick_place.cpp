#include <memory>
#include <chrono>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// MoveIt 2
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// Gazebo link-attacher services
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

// Messages & TF
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class BoxPickPlace : public rclcpp::Node
{
public:
  BoxPickPlace()
  : Node("box_pick_place"),
    mg_(std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), "ur_manipulator")),
    psi_(std::make_shared<moveit::planning_interface::PlanningSceneInterface>())
  {
    attach_client_ = create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
    detach_client_ = create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");

    addCollisionObjects();
    rclcpp::sleep_for(1s);

    pick();
    rclcpp::sleep_for(1s);
    attachObject();
    rclcpp::sleep_for(1s);
    place();
    rclcpp::sleep_for(1s);
    detachObject();
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>    mg_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;
  rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr      attach_client_;
  rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr      detach_client_;

  void addCollisionObjects()
  {
    using moveit_msgs::msg::CollisionObject;
    using shape_msgs::msg::SolidPrimitive;

    std::vector<CollisionObject> objs;

    // cube at (0,0,0.76)
    CollisionObject cube;
    cube.id              = "box";
    cube.header.frame_id = "base_link";
    SolidPrimitive p; p.type = p.BOX; p.dimensions = {0.03,0.03,0.03};
    cube.primitives      = {p};
    cube.primitive_poses.resize(1);
    cube.primitive_poses[0].position = {0.0,0.0,0.76};
    cube.primitive_poses[0].orientation.w = 1.0;
    cube.operation = cube.ADD;
    objs.push_back(cube);

    // belt underneath
    CollisionObject belt;
    belt.id              = "belt";
    belt.header.frame_id = "base_link";
    p.dimensions         = {0.60,2.00,1.00};
    belt.primitives      = {p};
    belt.primitive_poses.resize(1);
    belt.primitive_poses[0].position.z    = 0.50;
    belt.primitive_poses[0].orientation.w = 1.0;
    belt.operation = belt.ADD;
    objs.push_back(belt);

    psi_->applyCollisionObjects(objs);
    RCLCPP_INFO(get_logger(), "Scene: cube + belt added");
  }

  void pick()
  {
    geometry_msgs::msg::Pose T;
    tf2::Quaternion q; q.setRPY(M_PI,0,0);
    T.orientation = tf2::toMsg(q);

    T.position = {0.0,0.0,0.86};  planAndMove(T,"pre-pick");
    T.position.z = 0.76;          planAndMove(T,"pick");
  }

  void place()
  {
    geometry_msgs::msg::Pose T;
    tf2::Quaternion q; q.setRPY(M_PI,0,0);
    T.orientation = tf2::toMsg(q);

    T.position = {0.0,0.2,0.86}; planAndMove(T,"pre-place");
    T.position.z = 0.76;         planAndMove(T,"place");
  }

  void attachObject()
  {
    auto req = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
    req->model1_name = "cobot";
    req->link1_name  = "wrist_3_link";
    req->model2_name = "box";
    req->link2_name  = "box";

    waitForService(attach_client_, "ATTACHLINK");
    auto f = attach_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(shared_from_this(), f)
        == rclcpp::FutureReturnCode::SUCCESS)
      RCLCPP_INFO(get_logger(), "Object attached");
    else
      RCLCPP_ERROR(get_logger(), "Attach failed");
  }

  void detachObject()
  {
    auto req = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
    req->model1_name = "cobot";
    req->link1_name  = "wrist_3_link";
    req->model2_name = "box";
    req->link2_name  = "box";

    waitForService(detach_client_, "DETACHLINK");
    auto f = detach_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(shared_from_this(), f)
        == rclcpp::FutureReturnCode::SUCCESS)
      RCLCPP_INFO(get_logger(), "Object detached");
    else
      RCLCPP_ERROR(get_logger(), "Detach failed");
  }

  // — corrected template so ServiceT is deduced —
  template<typename ServiceT>
  void waitForService(
    const std::shared_ptr<rclcpp::Client<ServiceT>> &client,
    const std::string &name)
  {
    while (!client->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(), "Waiting for %s…", name.c_str());
    }
  }

  void planAndMove(const geometry_msgs::msg::Pose &T, const char *label)
  {
    mg_->setPoseTarget(T);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = mg_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    RCLCPP_INFO(get_logger(), "%s plan %s", label, ok?"OK":"FAILED");
    if (ok) mg_->execute(plan);
    mg_->clearPoseTargets();
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoxPickPlace>());
  rclcpp::shutdown();
  return 0;
}
