#pragma once

#include <map>
#include <string>
#include <utility>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

// Resolves the true yaw of a die from a SECOND face-count observation
// taken after tipping the die 90deg about a FIXED WORLD-frame axis while
// rigidly grasped.
//
// Formerly ResolveAmbiguousFace, restricted to the 3 faces (1, 4, 5) whose
// pip pattern has exact 90deg rotational symmetry. Renamed and extended to
// all 6 faces because the real vision service never determines orientation
// for ANY face -- it reports face number (pip count, reliable) and
// position, but a placeholder orientation -- so every starting face is
// "ambiguous" from a single read, not just the pip-symmetric ones.
//
// Both inputs are pure face-COUNT observations (never a reported
// orientation), so this never needs to "cheat" by reading a true
// orientation anywhere.
//
// Tables
// ------
// One table per tip axis, all derived numerically (never by hand -- an
// early by-hand attempt used the grasp's own closing axis, which is
// information-theoretically useless since it co-rotates with the very yaw
// being resolved). All four are injective for all 6 starting faces: each
// of the 4 yaw hypotheses lands on a DIFFERENT second face, so a single
// tip always fully resolves it and no case ever needs a second tip.
//
// Having all four means an unreachable tip can be swapped for another
// without a re-derivation: which axis the wrist can actually reach depends
// on the cell, the home pose and where the die sits (world_Y+ was
// validated once, then became NO_IK_SOLUTION after the home pose and spawn
// position changed). Pass the SAME tip_axis to GetTipRotation, which emits
// the matching quaternion -- that keeps the executed tip and the table
// selected here driven by one value, instead of two that can silently
// drift apart into a wrong-but-plausible resolved yaw.
//
// NOTE: when the target face is the geometric opposite of the current one,
// this resolution is unnecessary -- a 180deg flip about any horizontal
// axis reaches the opposite face regardless of yaw (verified: the
// resulting face is yaw-independent). Only route through this node when
// the target is NOT the opposite face.
class ResolveDieOrientation : public BT::SyncActionNode
{
public:
  ResolveDieOrientation(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("current_face"),
      BT::InputPort<int>("second_face"),
      // One of: world_X+, world_X-, world_Y+, world_Y- -- must be the same
      // value handed to GetTipRotation for the tip that was executed.
      BT::InputPort<std::string>("tip_axis"),
      BT::OutputPort<int>("resolved_theta_deg"),
    };
  }

  BT::NodeStatus tick() override
  {
    int current_face, second_face;
    if (!getInput("current_face", current_face)) {
      throw BT::RuntimeError("Missing parameter [current_face]");
    }
    if (!getInput("second_face", second_face)) {
      throw BT::RuntimeError("Missing parameter [second_face]");
    }
    std::string tip_axis;
    if (!getInput("tip_axis", tip_axis)) {
      throw BT::RuntimeError("Missing parameter [tip_axis]");
    }

    using Table = std::map<std::pair<int, int>, int>;

    static const Table kWorldXPlus = {
      {{1, 4}, 0}, {{1, 5}, 90}, {{1, 3}, 180}, {{1, 2}, 270},
      {{2, 3}, 0}, {{2, 6}, 90}, {{2, 4}, 180}, {{2, 1}, 270},
      {{3, 1}, 0}, {{3, 5}, 90}, {{3, 6}, 180}, {{3, 2}, 270},
      {{4, 6}, 0}, {{4, 5}, 90}, {{4, 1}, 180}, {{4, 2}, 270},
      {{5, 3}, 0}, {{5, 1}, 90}, {{5, 4}, 180}, {{5, 6}, 270},
      {{6, 3}, 0}, {{6, 5}, 90}, {{6, 4}, 180}, {{6, 2}, 270},
    };
    static const Table kWorldXMinus = {
      {{1, 3}, 0}, {{1, 2}, 90}, {{1, 4}, 180}, {{1, 5}, 270},
      {{2, 4}, 0}, {{2, 1}, 90}, {{2, 3}, 180}, {{2, 6}, 270},
      {{3, 6}, 0}, {{3, 2}, 90}, {{3, 1}, 180}, {{3, 5}, 270},
      {{4, 1}, 0}, {{4, 2}, 90}, {{4, 6}, 180}, {{4, 5}, 270},
      {{5, 4}, 0}, {{5, 6}, 90}, {{5, 3}, 180}, {{5, 1}, 270},
      {{6, 4}, 0}, {{6, 2}, 90}, {{6, 3}, 180}, {{6, 5}, 270},
    };
    static const Table kWorldYPlus = {
      {{1, 2}, 0}, {{1, 4}, 90}, {{1, 5}, 180}, {{1, 3}, 270},
      {{2, 1}, 0}, {{2, 3}, 90}, {{2, 6}, 180}, {{2, 4}, 270},
      {{3, 2}, 0}, {{3, 1}, 90}, {{3, 5}, 180}, {{3, 6}, 270},
      {{4, 2}, 0}, {{4, 6}, 90}, {{4, 5}, 180}, {{4, 1}, 270},
      {{5, 6}, 0}, {{5, 3}, 90}, {{5, 1}, 180}, {{5, 4}, 270},
      {{6, 2}, 0}, {{6, 3}, 90}, {{6, 5}, 180}, {{6, 4}, 270},
    };
    static const Table kWorldYMinus = {
      {{1, 5}, 0}, {{1, 3}, 90}, {{1, 2}, 180}, {{1, 4}, 270},
      {{2, 6}, 0}, {{2, 4}, 90}, {{2, 1}, 180}, {{2, 3}, 270},
      {{3, 5}, 0}, {{3, 6}, 90}, {{3, 2}, 180}, {{3, 1}, 270},
      {{4, 5}, 0}, {{4, 1}, 90}, {{4, 2}, 180}, {{4, 6}, 270},
      {{5, 1}, 0}, {{5, 4}, 90}, {{5, 6}, 180}, {{5, 3}, 270},
      {{6, 5}, 0}, {{6, 4}, 90}, {{6, 2}, 180}, {{6, 3}, 270},
    };

    const Table * table = nullptr;
    if (tip_axis == "world_X+") {
      table = &kWorldXPlus;
    } else if (tip_axis == "world_X-") {
      table = &kWorldXMinus;
    } else if (tip_axis == "world_Y+") {
      table = &kWorldYPlus;
    } else if (tip_axis == "world_Y-") {
      table = &kWorldYMinus;
    } else {
      throw BT::RuntimeError(
              "ResolveDieOrientation: unknown tip_axis '" + tip_axis +
              "' -- expected one of world_X+, world_X-, world_Y+, world_Y-");
    }

    auto it = table->find({current_face, second_face});
    if (it == table->end()) {
      throw BT::RuntimeError(
              "ResolveDieOrientation: no entry for (current_face=" +
              std::to_string(current_face) + ", second_face=" +
              std::to_string(second_face) + ") under tip_axis '" + tip_axis +
              "' -- second_face is the opposite of current_face (a tip can "
              "never produce that, so the reading or the executed tip "
              "doesn't match this axis).");
    }

    setOutput("resolved_theta_deg", it->second);
    return BT::NodeStatus::SUCCESS;
  }
};
