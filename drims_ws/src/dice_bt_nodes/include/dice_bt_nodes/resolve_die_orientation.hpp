#pragma once

#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

// Resolves the die's resting yaw from two face-COUNT observations taken
// before and after a known rotation. Never uses a reported orientation.
//
// Two modes: a per-tip-axis table, or -- preferred -- an analytic resolution
// from the rotation actually measured on the gripper, which also covers the
// arbitrary rotation the carry configuration adds. Verified: the analytic
// form reproduces the world_Y+ and world_X+ tables exactly, 24 entries each.
//
// A 180deg flip reaches the opposite face regardless of yaw, so this is only
// needed when the target is not the opposite face.
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
      // The rotation the die ACTUALLY underwent between the grasp and
      // the second reading, as a world-frame quaternion (measure it with
      // CaptureFrameRotation + ComputeRelativeRotation). When given it
      // replaces the per-axis tables entirely and tip_axis is ignored,
      // which is what lets the mission insert a fixed carry
      // configuration -- an arbitrary, run-dependent rotation no table
      // could cover -- between picking and tipping. Verified against
      // both derived tables (world_Y+ and world_X+): the analytic
      // resolution reproduces all 24 entries of each exactly.
      BT::InputPort<std::vector<double>>("total_rotation"),
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
    std::vector<double> total;
    if (getInput("total_rotation", total) && total.size() == 4) {
      const tf2::Quaternion q_total(total[0], total[1], total[2], total[3]);
      for (int theta = 0; theta < 360; theta += 90) {
        const tf2::Quaternion q_die = (q_total * restOrientation(current_face, theta))
          .normalized();
        if (faceUp(q_die) == second_face) {
          setOutput("resolved_theta_deg", theta);
          return BT::NodeStatus::SUCCESS;
        }
      }
      throw BT::RuntimeError(
              "ResolveDieOrientation: no yaw explains (current_face=" +
              std::to_string(current_face) + " -> second_face=" +
              std::to_string(second_face) + ") under the measured rotation -- "
              "the reading and the executed motion disagree.");
    }

    std::string tip_axis;
    if (!getInput("tip_axis", tip_axis)) {
      throw BT::RuntimeError(
              "Missing parameter [tip_axis] (and no [total_rotation] given)");
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

private:
  // FACE_NORMALS, matching drims_dice_simulator's dice_spawner.py:
  // 1<->-Z, 6<->+Z, 2<->-X, 5<->+X, 3<->+Y, 4<->-Y.
  static tf2::Vector3 faceNormal(int face)
  {
    switch (face) {
      case 1: return tf2::Vector3(0, 0, -1);
      case 2: return tf2::Vector3(-1, 0, 0);
      case 3: return tf2::Vector3(0, 1, 0);
      case 4: return tf2::Vector3(0, -1, 0);
      case 5: return tf2::Vector3(1, 0, 0);
      case 6: return tf2::Vector3(0, 0, 1);
      default:
        throw BT::RuntimeError("ResolveDieOrientation: face out of range 1..6");
    }
  }

  // Port of dice_spawner.py's get_quaternion_from_normal.
  static tf2::Quaternion quaternionFromNormal(const tf2::Vector3 & normal)
  {
    const tf2::Vector3 z_axis(0, 0, 1);
    const tf2::Vector3 v = z_axis.cross(normal);
    const double c = z_axis.dot(normal);
    if (v.length() < 1e-6) {
      return c > 0.0 ? tf2::Quaternion(0, 0, 0, 1) : tf2::Quaternion(1, 0, 0, 0);
    }
    return tf2::Quaternion(v.normalized(), std::acos(std::max(-1.0, std::min(1.0, c))));
  }

  // How a die rests with `face` up, spun `theta_deg` about the vertical.
  static tf2::Quaternion restOrientation(int face, int theta_deg)
  {
    const tf2::Quaternion base = quaternionFromNormal(faceNormal(face)).inverse();
    tf2::Quaternion yaw;
    yaw.setRotation(tf2::Vector3(0, 0, 1), theta_deg * M_PI / 180.0);
    return (yaw * base).normalized();
  }

  // Which face points most nearly straight up.
  static int faceUp(const tf2::Quaternion & q_die)
  {
    int best = 1;
    double best_z = -2.0;
    for (int f = 1; f <= 6; ++f) {
      const double z = tf2::quatRotate(q_die, faceNormal(f)).z();
      if (z > best_z) {
        best_z = z;
        best = f;
      }
    }
    return best;
  }
};
