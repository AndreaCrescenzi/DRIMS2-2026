#pragma once

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_ros2/plugins.hpp>
#include "behaviortree_ros2/ros_node_params.hpp"

// Declares the gripper links as allowed to touch the object being held.
//
// /attach_object leaves touch_links=["tool0"], so MoveIt reads the grasp
// itself as a collision: every collision-checked IK fails as -31 and every
// Cartesian path is rejected at fraction 0 (measured: 20/20 poses solve with
// collision checking off, 0/198 with it on). Joint moves sometimes survive
// because the start-state adapter jiggles them, which is what made the
// failures look random. Fixed entirely on our side, as a planning scene diff.
//
// Uses its own node and executor: motion_server already spins the BT node.
class AllowAttachedContact : public BT::SyncActionNode
{
public:
  AllowAttachedContact(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    init();
  }

  AllowAttachedContact(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const BT::RosNodeParams &)
  : BT::SyncActionNode(name, config)
  {
    init();
  }

  static BT::PortsList providedPorts()
  {
    return {
      // Empty means "every attached object".
      BT::InputPort<std::string>("object_id"),
      // Semicolon-separated link names allowed to touch it.
      BT::InputPort<std::string>("touch_links"),
      BT::InputPort<double>("timeout_s"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string object_id = "dice";
    getInput("object_id", object_id);
    std::string links =
      "robotiq_hande_left_finger;robotiq_hande_right_finger;"
      "robotiq_hande_base_link;robotiq_hande_end;tool0;wrist_3_link;flange";
    getInput("touch_links", links);
    double timeout_s = 5.0;
    getInput("timeout_s", timeout_s);
    const auto timeout = std::chrono::duration<double>(timeout_s);

    if (!get_client_->wait_for_service(std::chrono::seconds(2)) ||
      !apply_client_->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_WARN(node_->get_logger(), "AllowAttachedContact: planning scene services missing");
      return BT::NodeStatus::FAILURE;
    }

    auto get_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    get_req->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
    auto get_future = get_client_->async_send_request(get_req);
    if (executor_->spin_until_future_complete(get_future, timeout) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(node_->get_logger(), "AllowAttachedContact: /get_planning_scene timed out");
      return BT::NodeStatus::FAILURE;
    }

    auto attached = get_future.get()->scene.robot_state.attached_collision_objects;
    if (attached.empty()) {
      RCLCPP_WARN(node_->get_logger(), "AllowAttachedContact: nothing is attached");
      return BT::NodeStatus::FAILURE;
    }

    const std::vector<std::string> touch = split(links);
    int patched = 0;
    for (auto & a : attached) {
      if (!object_id.empty() && a.object.id != object_id) {
        continue;
      }
      a.touch_links = touch;
      a.object.operation = a.object.ADD;
      ++patched;
      RCLCPP_INFO(
        node_->get_logger(), "AllowAttachedContact: '%s' on %s -> %zu touch links",
        a.object.id.c_str(), a.link_name.c_str(), touch.size());
    }
    if (patched == 0) {
      RCLCPP_WARN(
        node_->get_logger(), "AllowAttachedContact: no attached object named '%s'",
        object_id.c_str());
      return BT::NodeStatus::FAILURE;
    }

    auto apply_req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    apply_req->scene.is_diff = true;
    apply_req->scene.robot_state.is_diff = true;
    apply_req->scene.robot_state.attached_collision_objects = attached;
    auto apply_future = apply_client_->async_send_request(apply_req);
    if (executor_->spin_until_future_complete(apply_future, timeout) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(node_->get_logger(), "AllowAttachedContact: /apply_planning_scene timed out");
      return BT::NodeStatus::FAILURE;
    }
    return apply_future.get()->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  void init()
  {
    node_ = std::make_shared<rclcpp::Node>(
      "bt_allow_attached_contact_" + std::to_string(reinterpret_cast<uintptr_t>(this)));
    get_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    apply_client_ =
      node_->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
  }

  static std::vector<std::string> split(const std::string & s)
  {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ';')) {
      if (!item.empty()) {
        out.push_back(item);
      }
    }
    return out;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_client_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_client_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};
