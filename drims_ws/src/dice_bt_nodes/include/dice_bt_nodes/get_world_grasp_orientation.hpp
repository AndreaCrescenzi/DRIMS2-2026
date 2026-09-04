#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_ros2/plugins.hpp>
#include "behaviortree_ros2/ros_node_params.hpp"

// Produces a grasp orientation defined in the WORLD frame -- gripper
// pointing down, closing axis parallel to the tip axis -- and returns it
// expressed relative to the frame the pick is commanded in (dice_tf by
// default), so the existing pick move can stay as it is.
//
// Why world-fixed rather than relative to the die's reported frame
// -------------------------------------------------------------------
// Aligning the closing axis with the tip axis is what makes the second
// face readable. Verified numerically over all 6 faces and all 4 yaw
// hypotheses (derive_occlusion_correct.py):
//
//   grasp direction PARALLEL to tip axis  -> 0/24 occluded, still injective
//   grasp direction PERPENDICULAR to it   -> 24/24 occluded
//
// With a perpendicular grasp the tip always drives the closing axis
// vertical, so the face it exposes is one of the two the fingers are
// holding -- the camera sees a finger, not the pips. Parallel keeps the
// closing axis horizontal throughout, so the exposed face is always one of
// the two the fingers are NOT touching, and the reading stays informative.
//
// That guarantee only holds if the closing axis really points along the
// tip axis IN THE WORLD. Commanding the grasp relative to the die's
// reported frame (as before) makes it follow the REPORTED orientation
// instead, which on the real robot is a placeholder -- so the alignment,
// and with it the no-occlusion guarantee, would be down to luck. Taking
// only the position from vision (reliable) and fixing the orientation in
// world is also exactly how the real pipeline has to work.
class GetWorldGraspOrientation : public BT::SyncActionNode
{
public:
  GetWorldGraspOrientation(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = config.blackboard->get<std::shared_ptr<rclcpp::Node>>("node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  }

  GetWorldGraspOrientation(
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
      // Same axis name given to GetTipRotation: the closing axis is aligned
      // with it, which is what removes the occlusion.
      BT::InputPort<std::string>("tip_axis"),
      // Approach tilt about the closing axis. Rotating about that axis
      // leaves the closing direction untouched, so it can't break the
      // alignment. Defaults to the -45deg used elsewhere in this project.
      BT::InputPort<double>("tilt_deg"),
      // Frame the pick MoveToPose is commanded in.
      BT::InputPort<std::string>("reference_frame"),
      BT::InputPort<std::string>("world_frame"),
      // Adds 180deg of yaw. That pinches the SAME pair of faces (the
      // closing axis is a line, not an arrow) so it cannot break the
      // parallel-to-tip alignment, but it reaches the grasp through a
      // different wrist configuration -- the same trick
      // get_grasp_orientation.hpp's flip_grasp exists for, and the
      // documented fix when one approach is kinematically blocked.
      BT::InputPort<bool>("flip_grasp"),
      BT::OutputPort<std::vector<double>>("pick_orientation"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string tip_axis;
    if (!getInput("tip_axis", tip_axis)) {
      throw BT::RuntimeError("Missing parameter [tip_axis]");
    }
    double tilt_deg = -45.0;
    getInput("tilt_deg", tilt_deg);
    std::string reference_frame = "dice_tf";
    getInput("reference_frame", reference_frame);
    std::string world_frame = "world";
    getInput("world_frame", world_frame);

    bool flip_grasp = false;
    getInput("flip_grasp", flip_grasp);

    const bool along_x = (tip_axis == "world_X+" || tip_axis == "world_X-");
    const bool along_y = (tip_axis == "world_Y+" || tip_axis == "world_Y-");
    if (!along_x && !along_y) {
      throw BT::RuntimeError(
              "GetWorldGraspOrientation: unknown tip_axis '" + tip_axis +
              "' -- expected one of world_X+, world_X-, world_Y+, world_Y-");
    }

    // "Point straight down": 180deg about X. This leaves the gripper's own
    // X (its closing axis) along world X, so it already matches a world_X
    // tip; a world_Y tip needs the whole grasp yawed 90deg about world Z.
    tf2::Quaternion q_desired(1.0, 0.0, 0.0, 0.0);
    double yaw_rad = along_y ? M_PI / 2.0 : 0.0;
    if (flip_grasp) {
      yaw_rad += M_PI;
    }
    if (yaw_rad != 0.0) {
      tf2::Quaternion yaw;
      yaw.setRotation(tf2::Vector3(0, 0, 1), yaw_rad);
      q_desired = yaw * q_desired;
    }
    if (tilt_deg != 0.0) {
      tf2::Quaternion tilt;
      tilt.setRotation(tf2::Vector3(1, 0, 0), tilt_deg * M_PI / 180.0);
      q_desired = q_desired * tilt;  // post-multiply: about the local (closing) axis
    }
    q_desired.normalize();

    // MoveToPose with frame_id=reference_frame interprets the orientation
    // relative to that frame, so hand back inverse(ref) * desired.
    geometry_msgs::msg::TransformStamped tf_ref;
    try {
      tf_ref = tf_buffer_->lookupTransform(world_frame, reference_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        node_->get_logger(), "GetWorldGraspOrientation: TF %s <- %s failed: %s",
        world_frame.c_str(), reference_frame.c_str(), ex.what());
      return BT::NodeStatus::FAILURE;
    }
    const tf2::Quaternion q_ref(
      tf_ref.transform.rotation.x, tf_ref.transform.rotation.y,
      tf_ref.transform.rotation.z, tf_ref.transform.rotation.w);

    const tf2::Quaternion q_rel = (q_ref.inverse() * q_desired).normalized();

    RCLCPP_INFO(
      node_->get_logger(),
      "GetWorldGraspOrientation: closing axis along %s, tilt %.0f deg, flip %d -> "
      "orientation in %s [%.3f, %.3f, %.3f, %.3f]",
      along_x ? "world_X" : "world_Y", tilt_deg, static_cast<int>(flip_grasp),
      reference_frame.c_str(),
      q_rel.x(), q_rel.y(), q_rel.z(), q_rel.w());

    setOutput(
      "pick_orientation",
      std::vector<double>{q_rel.x(), q_rel.y(), q_rel.z(), q_rel.w()});
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
