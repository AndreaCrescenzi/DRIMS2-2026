#pragma once

#include <string>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

// Net rotation between two orientations of the same frame: delta = to *
// inverse(from). Paired with CaptureFrameRotation; the grasp is rigid, so the
// delta applies to the die too.
class ComputeRelativeRotation : public BT::SyncActionNode
{
public:
  ComputeRelativeRotation(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<double>>("from"),
      BT::InputPort<std::vector<double>>("to"),
      BT::OutputPort<std::vector<double>>("delta"),
      // Undoes it, for returning the die to how it was grasped.
      BT::OutputPort<std::vector<double>>("inverse_delta"),
      BT::OutputPort<double>("angle_deg"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<double> from, to;
    if (!getInput("from", from) || from.size() != 4) {
      throw BT::RuntimeError("Missing/!=4 parameter [from]");
    }
    if (!getInput("to", to) || to.size() != 4) {
      throw BT::RuntimeError("Missing/!=4 parameter [to]");
    }
    const tf2::Quaternion q_from(from[0], from[1], from[2], from[3]);
    const tf2::Quaternion q_to(to[0], to[1], to[2], to[3]);
    tf2::Quaternion d = (q_to * q_from.inverse()).normalized();
    if (d.w() < 0.0) {
      d = tf2::Quaternion(-d.x(), -d.y(), -d.z(), -d.w());  // shortest path
    }

    setOutput("delta", std::vector<double>{d.x(), d.y(), d.z(), d.w()});
    setOutput("inverse_delta", std::vector<double>{-d.x(), -d.y(), -d.z(), d.w()});
    setOutput(
      "angle_deg",
      2.0 * std::acos(std::min(1.0, std::abs(d.w()))) * 180.0 / M_PI);
    return BT::NodeStatus::SUCCESS;
  }
};
