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

// Relative XYZ correction that brings the die from where it is now to a
// target, so a rotation that drifted it off the play area can be corrected
// in the same move that puts it down.
//
// Only X and Y go through target_frame; target_z stays a base_link height,
// because transforming it too once sent the die under the table.
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
      BT::InputPort<std::string>("target_frame"),
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

    std::string target_frame = "base_link";
    getInput("target_frame", target_frame);  // optional, defaults to base_link

    if (target_frame != "base_link") {
      geometry_msgs::msg::TransformStamped tf_target_to_base;
      try {
        tf_target_to_base = tf_buffer_->lookupTransform(
          "base_link", target_frame, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(
          node_->get_logger(),
          "ComputeXYCorrection: TF lookup base_link <- %s (target_frame) failed: %s",
          target_frame.c_str(), ex.what());
        return BT::NodeStatus::FAILURE;
      }
      const tf2::Transform T_target(
        tf2::Quaternion(
          tf_target_to_base.transform.rotation.x, tf_target_to_base.transform.rotation.y,
          tf_target_to_base.transform.rotation.z, tf_target_to_base.transform.rotation.w),
        tf2::Vector3(
          tf_target_to_base.transform.translation.x, tf_target_to_base.transform.translation.y,
          tf_target_to_base.transform.translation.z));
      // Only X/Y go through target_frame -- target_z (when given) stays a
      // plain base_link height regardless, since "how high above the
      // table" is a base_link/vertical fact independent of which XY
      // frame the safe zone is expressed in. Transforming Z too (an
      // earlier version of this did) silently shifted target_z by
      // target_frame's own Z offset from base_link (e.g.
      // camera_frame_floor sits ~0.05m below base_link's origin here),
      // sending the die's target resting height under the table and
      // making the final descend fail. Point transformed at local Z=0
      // (its own frame's floor height, not target_z) -- only .x()/.y()
      // of the result are used, .z() is discarded on purpose.
      const tf2::Vector3 target_in(target_x, target_y, 0.0);
      const tf2::Vector3 target_base = T_target * target_in;
      RCLCPP_INFO(
        node_->get_logger(),
        "ComputeXYCorrection: target [%.3f, %.3f] in %s -> [%.3f, %.3f] in base_link (z stays as given, in base_link)",
        target_x, target_y, target_frame.c_str(), target_base.x(), target_base.y());
      target_x = target_base.x();
      target_y = target_base.y();
    }

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
