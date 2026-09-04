#pragma once

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

// Turns a tip-axis NAME into the world-frame quaternion to command, so the
// tip actually executed and the table ResolveDieOrientation resolves it
// with are driven by ONE value in the tree instead of two that can drift
// apart.
//
// Why that matters: the disambiguation table is derived per tip axis. All
// four candidates (world_X/world_Y, either sign) are equally valid
// information-theoretically -- each is injective, verified numerically --
// so switching axis is a legitimate fix when one is unreachable from the
// current wrist configuration (world_Y+90 was validated once, then became
// NO_IK_SOLUTION after the home pose and die spawn position changed).
// But editing the quaternion in the XML while leaving the table alone
// silently yields a WRONG resolved yaw for some face pairs rather than an
// error, which is exactly the kind of failure that is hard to spot. With
// this node the axis name is written once and both sides follow it.
//
// Feed the output straight into MoveToPose with relative_motion=true and
// frame_id="base_link" (that combination is a true extrinsic world
// rotation in this cell).
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

    // 90deg about the named world axis. sqrt(0.5) written out to match the
    // literals the derivation script printed.
    constexpr double kS = 0.7071067811865476;
    std::vector<double> q;
    if (axis == "world_X+") {
      q = {kS, 0.0, 0.0, kS};
    } else if (axis == "world_X-") {
      q = {-kS, 0.0, 0.0, kS};
    } else if (axis == "world_Y+") {
      q = {0.0, kS, 0.0, kS};
    } else {
      q = {0.0, -kS, 0.0, kS};
    }

    setOutput("orientation", q);
    // Inverse of a unit quaternion: negate the vector part.
    setOutput("inverse_orientation", std::vector<double>{-q[0], -q[1], -q[2], q[3]});
    return BT::NodeStatus::SUCCESS;
  }
};
