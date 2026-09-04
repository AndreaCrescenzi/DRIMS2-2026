#pragma once

#include <cmath>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

// Turns a tip-axis NAME into the world-frame quaternion to command, so the
// executed tip and the resolution that follows are driven by one value.
// All four axes are equally informative; which one the wrist can reach
// depends on the configuration, so being able to switch matters.
class GetTipRotation : public BT::SyncActionNode
{
public:
  GetTipRotation(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      // One of: world_X+, world_X-, world_Y+, world_Y-
      BT::InputPort<std::string>("tip_axis"),
      BT::OutputPort<std::vector<double>>("orientation"),
      // The exact inverse, for undoing the tip after the second face has
      // been read. Undoing is reachable by construction (it returns the
      // wrist to the pose it was just in), which makes it a safe base for
      // the placement rotation that follows -- rotating straight out of
      // the tipped pose was observed to hit NO_IK_SOLUTION.
      BT::OutputPort<std::vector<double>>("inverse_orientation"),
      // The same rotations halved (45deg). Commanding the tip as two
      // Cartesian 45deg steps instead of one joint-space 90deg keeps the
      // grasp point still and just turns the wrist in place, instead of
      // letting the sampling planner swing the whole arm around -- smaller,
      // calmer and safer motion for the same result. A single 90deg step
      // does not survive straight-line interpolation; two 45deg ones do.
      BT::OutputPort<std::vector<double>>("half_orientation"),
      BT::OutputPort<std::vector<double>>("half_inverse_orientation"),
    };
  }

  // Shared with ResolveDieOrientation so both reject the same typos.
  static bool isKnownAxis(const std::string & axis)
  {
    return axis == "world_X+" || axis == "world_X-" ||
           axis == "world_Y+" || axis == "world_Y-";
  }

  BT::NodeStatus tick() override
  {
    std::string axis;
    if (!getInput("tip_axis", axis)) {
      throw BT::RuntimeError("Missing parameter [tip_axis]");
    }
    if (!isKnownAxis(axis)) {
      throw BT::RuntimeError(
              "GetTipRotation: unknown tip_axis '" + axis +
              "' -- expected one of world_X+, world_X-, world_Y+, world_Y-");
    }

    // 90deg about the named world axis, plus the 45deg half-step.
    constexpr double kS = 0.7071067811865476;              // sin/cos of 45deg
    const double kHs = std::sin(M_PI / 8.0);               // sin of 22.5deg
    const double kHc = std::cos(M_PI / 8.0);
    std::vector<double> q, qh;
    if (axis == "world_X+") {
      q = {kS, 0.0, 0.0, kS};
      qh = {kHs, 0.0, 0.0, kHc};
    } else if (axis == "world_X-") {
      q = {-kS, 0.0, 0.0, kS};
      qh = {-kHs, 0.0, 0.0, kHc};
    } else if (axis == "world_Y+") {
      q = {0.0, kS, 0.0, kS};
      qh = {0.0, kHs, 0.0, kHc};
    } else {
      q = {0.0, -kS, 0.0, kS};
      qh = {0.0, -kHs, 0.0, kHc};
    }

    setOutput("orientation", q);
    // Inverse of a unit quaternion: negate the vector part.
    setOutput("inverse_orientation", std::vector<double>{-q[0], -q[1], -q[2], q[3]});
    setOutput("half_orientation", qh);
    setOutput("half_inverse_orientation", std::vector<double>{-qh[0], -qh[1], -qh[2], qh[3]});
    return BT::NodeStatus::SUCCESS;
  }
};
