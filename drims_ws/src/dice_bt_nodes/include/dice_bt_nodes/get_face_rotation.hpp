#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_ros2/plugins.hpp>
#include "behaviortree_ros2/ros_node_params.hpp"

// Given the die's current and target face, outputs the exact relative
// rotation (as a quaternion) that a grasped, rigidly-attached die needs in
// order to bring target_face's normal to where current_face's normal is
// now — i.e. the rotation to feed straight into MoveToPose's
// relative_motion=true "orientation" port.
//
// Relies on the simulator publishing one TF frame per face (face1_tf ...
// face6_tf, children of dice_com_tf, see drims_dice_simulator's
// dice_spawner.py), each frame's local Z aligned with that face's outward
// normal. Only valid in simulation for now: real vision will need its own
// way of resolving the die's full orientation (see the DRIMS2 challenge
// plan/memory for why that ambiguity belongs inside the identification
// skill, not here).
//
// Verified empirically against the simulator (see TESTING.md / the plan):
// the correct call is lookupTransform(target_frame=face{target}_tf,
// source_frame=face{current}_tf) while the caller sets the outgoing
// PoseStamped's frame_id to face{current}_tf — this is *not* the naive
// (current, target) argument order, see the plan for the derivation.
class GetFaceRotation : public BT::SyncActionNode
{
public:
  GetFaceRotation(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = config.blackboard->get<std::shared_ptr<rclcpp::Node>>("node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  }

  GetFaceRotation(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const BT::RosNodeParams & params)
  : BT::SyncActionNode(name, config), node_(params.nh)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("current_face"),
      BT::InputPort<int>("target_face"),
      BT::OutputPort<std::vector<double>>("orientation"),
    };
  }

  BT::NodeStatus tick() override
  {
    int current_face, target_face;

    if (!getInput("current_face", current_face)) {
      throw BT::RuntimeError("Missing parameter [current_face]");
    }
    if (!getInput("target_face", target_face)) {
      throw BT::RuntimeError("Missing parameter [target_face]");
    }

    const std::string current_frame = "face" + std::to_string(current_face) + "_tf";
    const std::string target_frame = "face" + std::to_string(target_face) + "_tf";

    geometry_msgs::msg::TransformStamped transform;
    try {
      // NOTE: target_frame/source_frame here are the *target die face* and
      // *current die face* respectively, not (current, target) — see the
      // class-level comment. Getting this backwards silently returns the
      // inverse rotation instead of failing loudly.
      transform = tf_buffer_->lookupTransform(target_frame, current_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        node_->get_logger(), "GetFaceRotation: TF lookup %s -> %s failed: %s",
        current_frame.c_str(), target_frame.c_str(), ex.what());
      return BT::NodeStatus::FAILURE;
    }

    const auto & q = transform.transform.rotation;
    RCLCPP_INFO(
      node_->get_logger(),
      "GetFaceRotation: %d -> %d => orientation [%.3f, %.3f, %.3f, %.3f]",
      current_face, target_face, q.x, q.y, q.z, q.w);
    setOutput("orientation", std::vector<double>{q.x, q.y, q.z, q.w});

    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
