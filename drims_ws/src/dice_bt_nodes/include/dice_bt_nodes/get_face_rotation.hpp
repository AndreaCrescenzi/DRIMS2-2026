#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

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

// Given the die's current and target face, outputs the *minimal* relative
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
// Why "minimal" matters: naively using the full transform between
// face{current}_tf and face{target}_tf (as an earlier version of this node
// did) also locks in whatever in-plane twist those frames happen to have
// baked into their orientation convention — for adjacent faces this can
// demand a ~120 degree rotation instead of the 90 degrees that's actually
// necessary, which is measurably harder (sometimes impossible) for the
// wrist to reach in one motion. We only care about which face ends up up,
// not the die's final in-plane yaw, so this computes the shortest rotation
// that aligns just the two face normals (classic axis = cross, angle =
// acos(dot) construction), then converts that into the ref_frame-relative
// delta the relative_motion mechanism expects (see the plan for the
// derivation of the conjugation below).
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
      // The frame_id that MoveToPose's relative_motion=true call must use
      // together with `orientation` above (BT.CPP XML can't build
      // "face{face}_tf" via string interpolation, only whole-value
      // blackboard substitution, hence this port instead of a template).
      BT::OutputPort<std::string>("ref_frame"),
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

    // Both looked up relative to the same fixed reference (base_link) so
    // their Z axes (= face normals) and rotations live in one common frame.
    geometry_msgs::msg::TransformStamped tf_current, tf_target;
    try {
      tf_current = tf_buffer_->lookupTransform("base_link", current_frame, tf2::TimePointZero);
      tf_target = tf_buffer_->lookupTransform("base_link", target_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        node_->get_logger(), "GetFaceRotation: TF lookup for %s/%s failed: %s",
        current_frame.c_str(), target_frame.c_str(), ex.what());
      return BT::NodeStatus::FAILURE;
    }

    const tf2::Quaternion C(
      tf_current.transform.rotation.x, tf_current.transform.rotation.y,
      tf_current.transform.rotation.z, tf_current.transform.rotation.w);
    const tf2::Quaternion T(
      tf_target.transform.rotation.x, tf_target.transform.rotation.y,
      tf_target.transform.rotation.z, tf_target.transform.rotation.w);

    tf2::Vector3 n_current = tf2::Matrix3x3(C).getColumn(2).normalized();
    tf2::Vector3 n_target = tf2::Matrix3x3(T).getColumn(2).normalized();

    // Minimal rotation (in base_link) that maps n_target onto n_current.
    tf2::Quaternion delta_min;
    tf2::Vector3 axis = n_target.cross(n_current);
    const double dot = std::max(-1.0, std::min(1.0, n_target.dot(n_current)));

    if (axis.length() < 1e-6) {
      if (dot > 0.0) {
        delta_min = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);  // already aligned
      } else {
        // 180 degrees: cross product is undefined, pick any perpendicular axis.
        tf2::Vector3 helper = std::abs(n_target.x()) < 0.9 ?
          tf2::Vector3(1, 0, 0) : tf2::Vector3(0, 1, 0);
        tf2::Vector3 perp = n_target.cross(helper).normalized();
        delta_min = tf2::Quaternion(perp, M_PI);
      }
    } else {
      axis.normalize();
      delta_min = tf2::Quaternion(axis, std::acos(dot));
    }
    delta_min.normalize();

    // relative_motion applies R_ref_delta as R_base_ref * R_ref_delta *
    // R_base_ref^-1 on top of the current tool orientation (see
    // motion_server.py::_apply_relative_offset). With ref_frame =
    // current_frame (R_base_ref = C), solving for R_ref_delta such that the
    // result equals delta_min gives this conjugation.
    const tf2::Quaternion ref_delta = (C.inverse() * delta_min * C).normalized();

    RCLCPP_INFO(
      node_->get_logger(),
      "GetFaceRotation: %d -> %d => minimal angle %.1f deg, orientation [%.3f, %.3f, %.3f, %.3f]",
      current_face, target_face, std::acos(dot) * 180.0 / M_PI,
      ref_delta.x(), ref_delta.y(), ref_delta.z(), ref_delta.w());

    setOutput(
      "orientation",
      std::vector<double>{ref_delta.x(), ref_delta.y(), ref_delta.z(), ref_delta.w()});
    setOutput("ref_frame", current_frame);

    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
