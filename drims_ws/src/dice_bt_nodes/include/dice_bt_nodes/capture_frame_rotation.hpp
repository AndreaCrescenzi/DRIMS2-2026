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

// Reads a frame's orientation from TF onto the blackboard, so the tree can
// MEASURE the rotation applied to the die between two moments instead of
// assuming it. Needed because the fixed carry configuration rotates the die
// by an amount only known at runtime.
class CaptureFrameRotation : public BT::SyncActionNode
{
public:
  CaptureFrameRotation(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = config.blackboard->get<std::shared_ptr<rclcpp::Node>>("node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  }

  CaptureFrameRotation(
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
      BT::InputPort<std::string>("parent_frame"),
      BT::InputPort<std::string>("child_frame"),
      BT::OutputPort<std::vector<double>>("orientation"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string parent_frame = "world";
    getInput("parent_frame", parent_frame);
    std::string child_frame = "tool0";
    getInput("child_frame", child_frame);

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        node_->get_logger(), "CaptureFrameRotation: TF %s <- %s failed: %s",
        parent_frame.c_str(), child_frame.c_str(), ex.what());
      return BT::NodeStatus::FAILURE;
    }
    const auto & q = tf.transform.rotation;
    RCLCPP_INFO(
      node_->get_logger(), "CaptureFrameRotation: %s in %s = [%.3f, %.3f, %.3f, %.3f]",
      child_frame.c_str(), parent_frame.c_str(), q.x, q.y, q.z, q.w);
    setOutput("orientation", std::vector<double>{q.x, q.y, q.z, q.w});
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
