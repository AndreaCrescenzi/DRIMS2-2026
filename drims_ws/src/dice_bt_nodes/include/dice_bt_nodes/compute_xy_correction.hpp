#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

// Computes the horizontal (x,y) correction needed to bring the die back
// over a fixed target point (typically the center of the delimited
// placement zone), without touching orientation.
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
// Output is meant to feed a MoveToPose with relative_motion=true,
// frame_id="base_link" (or "world"), orientation="0;0;0;1" (identity
// delta -- keeps current orientation unchanged).
class ComputeXYCorrection : public BT::SyncActionNode
{
public:
  ComputeXYCorrection(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("current_pose"),
      BT::InputPort<double>("target_x"),
      BT::InputPort<double>("target_y"),
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

    const double dx = target_x - current_pose.pose.position.x;
    const double dy = target_y - current_pose.pose.position.y;

    setOutput("xy_correction", std::vector<double>{dx, dy, 0.0});

    return BT::NodeStatus::SUCCESS;
  }
};
