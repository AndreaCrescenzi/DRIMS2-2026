#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_ros2/plugins.hpp>
#include "behaviortree_ros2/ros_node_params.hpp"

// Computes the horizontal (x,y) correction needed to bring the die back
// over a fixed target point (target_x/target_y, expressed in "base_link"
// -- matching drims_dice_simulator's own spawn-bounds convention), without
// touching orientation.
//
// Why this exists: after picking, lifting and rotating the die to expose a
// different face, the die is no longer necessarily above where it was
// picked up (the pivot point of the rotation isn't the die's own center).
// Simply moving the gripper back to a fixed "pointing down" pose to place
// it would also undo the rotation we just carefully applied (gripper and
// die move rigidly together while attached) -- so the placement move must
// be a pure horizontal translation on top of whatever orientation the
// rotate step left us at, computed from a fresh DiceIdentification call
// after rotating.
//
// IMPORTANT: DiceIdentification reports current_pose in its own frame
// (typically "world"), which is *not* the same origin as "base_link" (the
// frame target_x/target_y are defined in). An earlier version of this node
// subtracted target_x/y directly from current_pose's raw coordinates,
// silently mixing the two frames -- the numbers looked plausible but the
// die landed outside the actual delimited zone. This version explicitly
// transforms current_pose into base_link first.
//
// Output is meant to feed a MoveToPose with relative_motion=true,
// frame_id="base_link", orientation="0;0;0;1" (identity delta -- keeps
// current orientation unchanged).
class ComputeXYCorrection : public BT::SyncActionNode
{
public:
  ComputeXYCorrection(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = config.blackboard->get<std::shared_ptr<rclcpp::Node>>("node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  }

  ComputeXYCorrection(
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
      BT::InputPort<geometry_msgs::msg::PoseStamped>("current_pose"),
      BT::InputPort<double>("target_x"),
      BT::InputPort<double>("target_y"),
      BT::InputPort<double>("target_z"),
      BT::InputPort<double>("z_offset"),
      BT::OutputPort<std::vector<double>>("xy_correction"),
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped current_pose;
    double target_x, target_y;

    if (!getInput("current_pose", current_pose)) {
      throw BT::RuntimeError("Missing parameter [current_pose]");
    }
    if (!getInput("target_x", target_x)) {
      throw BT::RuntimeError("Missing parameter [target_x]");
    }
    if (!getInput("target_y", target_y)) {
      throw BT::RuntimeError("Missing parameter [target_y]");
    }
    // Optional: when provided, also corrects height down to a known
    // resting z instead of leaving it at 0.0 (unchanged). Exists because
    // mirroring each intermediate lift with a matching descent (lift
    // before relocating away from an obstacle, lift again for the
    // rotation escalation, ...) drifts and is easy to under/over-count --
    // correcting straight to a known-good absolute height, the same way
    // target_x/target_y already do, is far more robust than bookkeeping
    // every relative delta by hand.
    double target_z = 0.0;
    const bool has_target_z = getInput("target_z", target_z).has_value();
    // Optional, mutually exclusive with target_z: a fixed RELATIVE z
    // delta instead of an absolute target -- for combining "translate
    // toward a target XY" with "lift by a fixed clearance amount" into a
    // single move (e.g. clearing the table while relocating away from an
    // obstacle), where there's no meaningful absolute height to aim for,
    // just "current height plus a margin".
    double z_offset = 0.0;
    getInput("z_offset", z_offset);

    geometry_msgs::msg::TransformStamped tf_to_base;
    try {
      tf_to_base = tf_buffer_->lookupTransform(
        "base_link", current_pose.header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        node_->get_logger(),
        "ComputeXYCorrection: TF lookup base_link <- %s failed: %s",
        current_pose.header.frame_id.c_str(), ex.what());
      return BT::NodeStatus::FAILURE;
    }

    const tf2::Transform T(
      tf2::Quaternion(
        tf_to_base.transform.rotation.x, tf_to_base.transform.rotation.y,
        tf_to_base.transform.rotation.z, tf_to_base.transform.rotation.w),
      tf2::Vector3(
        tf_to_base.transform.translation.x, tf_to_base.transform.translation.y,
        tf_to_base.transform.translation.z));

    const tf2::Vector3 p_in(
      current_pose.pose.position.x, current_pose.pose.position.y,
      current_pose.pose.position.z);
    const tf2::Vector3 p_base = T * p_in;

    const double dx = target_x - p_base.x();
    const double dy = target_y - p_base.y();
    const double dz = has_target_z ? (target_z - p_base.z()) : z_offset;

    RCLCPP_INFO(
      node_->get_logger(),
      "ComputeXYCorrection: current in base_link [%.3f, %.3f, %.3f] (from "
      "%s), target [%.3f, %.3f, %.3f] -> correction [%.3f, %.3f, %.3f]",
      p_base.x(), p_base.y(), p_base.z(), current_pose.header.frame_id.c_str(),
      target_x, target_y, has_target_z ? target_z : p_base.z(), dx, dy, dz);

    setOutput("xy_correction", std::vector<double>{dx, dy, dz});

    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
