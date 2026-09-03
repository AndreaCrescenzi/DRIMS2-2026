#pragma once

#include <map>
#include <string>
#include <utility>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

// Resolves the true yaw of a die that was showing a 90deg-pip-symmetric
// face (1, 4, or 5 -- a single top-down 2D image cannot tell the true yaw
// from the 3 other 90deg-step hypotheses for these) from a SECOND
// face-count observation taken after tipping the die 90deg about a FIXED
// WORLD-frame axis while rigidly grasped.
//
// Both inputs are pure face-COUNT observations (never the ambiguous
// service's fake orientation reading) -- pip counting is always reliable,
// even on a symmetric face, so this never needs to "cheat" by reading a
// true orientation anywhere.
//
// The lookup table below was derived numerically (not by hand -- an
// earlier by-hand attempt using the grasp's own closing axis as the tip
// axis turned out to be information-theoretically useless, since that
// axis co-rotates with the very yaw ambiguity being resolved; verified
// with a script reimplementing drims_dice_simulator's dice_spawner.py
// FACE_NORMALS/get_quaternion_from_normal exactly) for tipping 90deg
// about world_Y (sign +1), matching the convention used by the "tip"
// MoveToPose in _test_disambiguation.xml -- world_X was tried first and
// is equally valid information-theoretically (the derivation showed
// EVERY fixed-world axis/sign choice is injective), but was NOT
// reachable (NO_IK_SOLUTION) from the wrist configuration this specific
// grasp/lift produces; world_Y worked. The map is injective for every
// ambiguous face (each of the 4 yaw hypotheses lands on a DIFFERENT
// second face), so a single tip always fully resolves it -- no case ever
// needs a second tip.
class ResolveAmbiguousFace : public BT::SyncActionNode
{
public:
  ResolveAmbiguousFace(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("ambiguous_face"),
      BT::InputPort<int>("second_face"),
      BT::OutputPort<int>("resolved_theta_deg"),
    };
  }

  BT::NodeStatus tick() override
  {
    int ambiguous_face, second_face;
    if (!getInput("ambiguous_face", ambiguous_face)) {
      throw BT::RuntimeError("Missing parameter [ambiguous_face]");
    }
    if (!getInput("second_face", second_face)) {
      throw BT::RuntimeError("Missing parameter [second_face]");
    }

    static const std::map<std::pair<int, int>, int> kTable = {
      // ambiguous_face=1 (opposite 6): tip about world_Y, sign=+1
      {{1, 2}, 0}, {{1, 4}, 90}, {{1, 5}, 180}, {{1, 3}, 270},
      // ambiguous_face=4 (opposite 3)
      {{4, 2}, 0}, {{4, 6}, 90}, {{4, 5}, 180}, {{4, 1}, 270},
      // ambiguous_face=5 (opposite 2)
      {{5, 6}, 0}, {{5, 3}, 90}, {{5, 1}, 180}, {{5, 4}, 270},
    };

    auto it = kTable.find({ambiguous_face, second_face});
    if (it == kTable.end()) {
      throw BT::RuntimeError(
              "ResolveAmbiguousFace: no entry for (ambiguous_face=" +
              std::to_string(ambiguous_face) + ", second_face=" +
              std::to_string(second_face) + ") -- either ambiguous_face "
              "wasn't one of {1,4,5}, or the tip motion didn't match the "
              "world_X/+1 convention this table was derived for.");
    }

    setOutput("resolved_theta_deg", it->second);
    return BT::NodeStatus::SUCCESS;
  }
};
