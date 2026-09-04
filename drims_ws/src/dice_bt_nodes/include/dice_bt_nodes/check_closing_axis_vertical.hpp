#pragma once

#include <algorithm>
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

// Reports whether the gripper's closing axis is vertical, i.e. whether a
// finger sits on the face the camera needs to see. Diagnostic only.
class CheckClosingAxisVertical : public BT::SyncActionNode
{
public:
  CheckClosingAxisVertical(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = config.blackboard->get<std::shared_ptr<rclcpp::Node>>("node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  }

  CheckClosingAxisVertical(
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
      BT::InputPort<std::string>("base_frame"),
      BT::InputPort<std::string>("gripper_frame"),
      // |world Z component of the closing axis| above this is
      // considered "vertical". 0.7 ~= cos(45deg): a full 90deg tip
      // between hypotheses always lands near 0.0 (horizontal) or 1.0
      // (vertical) in practice, never close to this boundary -- see the
      // verification script's "worst_vert" column, which was always
      // exactly 0.0 or 1.0, never in between.
      BT::InputPort<double>("threshold"),
      BT::OutputPort<bool>("is_vertical"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string base_frame = "base_link";
    getInput("base_frame", base_frame);
    std::string gripper_frame = "tool0";
    getInput("gripper_frame", gripper_frame);
    double threshold = 0.7;
    getInput("threshold", threshold);

    geometry_msgs::msg::TransformStamped tf_gripper;
    try {
      tf_gripper = tf_buffer_->lookupTransform(base_frame, gripper_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        node_->get_logger(), "CheckClosingAxisVertical: TF lookup for %s failed: %s",
        gripper_frame.c_str(), ex.what());
      return BT::NodeStatus::FAILURE;
    }

    const tf2::Quaternion q(
      tf_gripper.transform.rotation.x, tf_gripper.transform.rotation.y,
      tf_gripper.transform.rotation.z, tf_gripper.transform.rotation.w);
    // Closing axis = gripper_frame's local X, expressed in base_frame.
    const tf2::Vector3 closing_axis = tf2::Matrix3x3(q).getColumn(0).normalized();
    const double z_component = std::abs(closing_axis.z());
    const bool is_vertical = z_component > threshold;

    RCLCPP_INFO(
      node_->get_logger(),
      "CheckClosingAxisVertical: closing axis z-component = %.3f -> %s",
      z_component, is_vertical ? "VERTICAL (unsafe)" : "horizontal (safe)");

    setOutput("is_vertical", is_vertical);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
