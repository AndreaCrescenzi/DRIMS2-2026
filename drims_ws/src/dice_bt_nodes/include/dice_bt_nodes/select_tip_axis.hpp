#pragma once

#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_ros2/plugins.hpp>
#include "behaviortree_ros2/ros_node_params.hpp"

// Chooses which world axis to tip about, based on the closing axis the
// grasp ACTUALLY ended up with (measured from TF, not assumed).
//
// The tip must be parallel to the gripper's closing axis, otherwise it
// drives that axis vertical and the face it exposes is one the fingers are
// holding -- the camera then sees a finger instead of pips. Verified over
// all 6 faces x 4 yaw hypotheses (derive_occlusion_correct.py):
//
//   grasp parallel to tip axis       -> 0/24 occluded, still injective
//   grasp perpendicular to tip axis  -> 24/24 occluded
//
// Two ways to satisfy that: force the grasp to match a chosen tip axis, or
// choose the tip axis to match whatever grasp was achieved. This node does
// the latter, which keeps the existing, kinematically-proven grasp exactly
// as it is -- forcing a world-fixed grasp instead turned out to be either
// unreachable or unplannable at the same die position, and tuning it by
// trial and error is the escalation pattern this project deliberately
// avoids. Every one of the four candidate tip axes is equally informative
// (each has its own verified table in ResolveDieOrientation), so picking
// the one that matches costs nothing.
//
// Feed the output straight into GetTipRotation and ResolveDieOrientation,
// so the executed tip and the table used to read it stay in step.
class SelectTipAxis : public BT::SyncActionNode
{
public:
  SelectTipAxis(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = config.blackboard->get<std::shared_ptr<rclcpp::Node>>("node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  }

  SelectTipAxis(
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
      BT::InputPort<std::string>("world_frame"),
      BT::InputPort<std::string>("gripper_frame"),
      // Set true to prefer the negative sign of the chosen axis. The sign
      // doesn't affect occlusion (a tip either way keeps a parallel closing
      // axis horizontal) but it does change which wrist configurations the
      // tip passes through, so it is worth being able to flip when one
      // direction turns out unreachable.
      BT::InputPort<bool>("prefer_negative"),
      BT::OutputPort<std::string>("tip_axis"),
      // How parallel the closing axis is to the chosen tip axis: 1.0 is
      // perfectly parallel (no occlusion), 0.0 perpendicular (fully
      // occluded). Logged so a run can be judged at a glance.
      BT::OutputPort<double>("alignment"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string world_frame = "world";
    getInput("world_frame", world_frame);
    std::string gripper_frame = "tool0";
    getInput("gripper_frame", gripper_frame);
    bool prefer_negative = false;
    getInput("prefer_negative", prefer_negative);

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(world_frame, gripper_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        node_->get_logger(), "SelectTipAxis: TF %s <- %s failed: %s",
        world_frame.c_str(), gripper_frame.c_str(), ex.what());
      return BT::NodeStatus::FAILURE;
    }

    const tf2::Quaternion q(
      tf.transform.rotation.x, tf.transform.rotation.y,
      tf.transform.rotation.z, tf.transform.rotation.w);
    // The closing axis is the gripper frame's own X (established for this
    // cell's Hand-E; see check_closing_axis_vertical.hpp).
    const tf2::Vector3 closing = tf2::Matrix3x3(q).getColumn(0).normalized();

    const double along_x = std::abs(closing.x());
    const double along_y = std::abs(closing.y());
    const bool pick_x = along_x >= along_y;
    const double alignment = pick_x ? along_x : along_y;

    std::string axis = pick_x ? "world_X" : "world_Y";
    axis += prefer_negative ? "-" : "+";

    RCLCPP_INFO(
      node_->get_logger(),
      "SelectTipAxis: closing axis in %s = [%.3f, %.3f, %.3f] -> tip %s "
      "(alignment %.3f, 1.0 = no occlusion)",
      world_frame.c_str(), closing.x(), closing.y(), closing.z(),
      axis.c_str(), alignment);

    setOutput("tip_axis", axis);
    setOutput("alignment", alignment);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
