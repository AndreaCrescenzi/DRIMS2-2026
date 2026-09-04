#pragma once

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

// Closes the "identify + normalize" phase: given the die's now-KNOWN true
// orientation (current_face from pip count + resolved_theta_deg from
// ResolveDieOrientation), outputs the world-frame rotation that puts the
// grasped die back on the table in a CANONICAL orientation (yaw = 0),
// plus which face that leaves facing up.
//
// Why this exists
// ---------------
// The first pick has to happen BEFORE the die's yaw is known (the tip that
// resolves it needs the die in hand), so that grasp is necessarily blind.
// GetGraspOrientation's yaw table assumes a canonical die, which a blind
// pick only satisfies half the time -- verified numerically over all
// (face, yaw, target) combinations: in 60/120 cases the fingers close on a
// DIFFERENT face pair than the table assumes, and in 48/120 they land
// directly on the target face (the "a finger is on the face we need"
// failure seen on the real robot). In those cases no choice of final
// rotation can save the release: the fingers are physically in the way.
//
// So the die is put back down in a canonical orientation first, and only
// then re-picked for the real reorientation. After that placement the
// existing GetGraspOrientation/ReorientCycle assumptions hold by
// construction -- re-verified numerically: 0/30 cases put a finger on the
// target face (down from 48/120), with no change needed to either.
//
// Selection rule
// --------------
// Among the 6 canonical (yaw = 0) placements, keep only those whose
// release is safe -- the gripper's closing axis (tool0's local X, carried
// rigidly by the grasp) must not end up vertical, or a finger would be
// trapped between die and table. Then take the smallest rotation. Verified
// exhaustively: every (current_face, theta) has such a placement, and in
// every case the closing axis comes out exactly horizontal (|z| = 0.00).
// Required rotations are 90deg or 120deg.
//
// Everything here is analytic -- no TF lookup, no reported orientation.
// That matters for the real robot, where the simulator's face{N}_tf frames
// don't exist and the vision service's orientation is a placeholder.
class ComputeCanonicalPlacement : public BT::SyncActionNode
{
public:
  ComputeCanonicalPlacement(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("current_face"),
      BT::InputPort<int>("resolved_theta_deg"),
      // The pick orientation actually commanded (straight from
      // GetGraspOrientation's output port) -- taken as an input rather
      // than re-derived here so the closing-axis check can never drift
      // out of sync with the grasp that was really performed.
      BT::InputPort<std::vector<double>>("pick_orientation"),
      // The tip actually applied after the pick, as a world-frame
      // quaternion. Defaults to the validated 90deg about world_Y.
      BT::InputPort<std::vector<double>>("tip_orientation"),
      // World-frame delta: feed to MoveToPose with relative_motion=true
      // and frame_id="base_link" (that combination is a true extrinsic
      // world rotation in this cell, so no conjugation is needed).
      BT::OutputPort<std::vector<double>>("orientation"),
      // Which face ends up up once placed -- lets the tree log/verify it.
      BT::OutputPort<int>("placed_face"),
      // The same rotation halved, so the tree can command it as two
      // Cartesian half-steps instead of one joint-space move. A Cartesian
      // rotation turns the wrist in place and leaves the grasp point where
      // it is; a joint-space one lets the sampling planner swing the whole
      // arm around to reach the same orientation, which looks alarming and
      // sweeps a lot of space. Rotations here reach 120deg, too much for a
      // single straight-line step, but fine in two.
      BT::OutputPort<std::vector<double>>("half_orientation"),
    };
  }

  BT::NodeStatus tick() override
  {
    int current_face = 0, theta_deg = 0;
    if (!getInput("current_face", current_face)) {
      throw BT::RuntimeError("Missing parameter [current_face]");
    }
    if (!getInput("resolved_theta_deg", theta_deg)) {
      throw BT::RuntimeError("Missing parameter [resolved_theta_deg]");
    }
    std::vector<double> pick_vec;
    if (!getInput("pick_orientation", pick_vec) || pick_vec.size() != 4) {
      throw BT::RuntimeError("Missing/!=4 parameter [pick_orientation]");
    }
    const tf2::Quaternion q_pick(pick_vec[0], pick_vec[1], pick_vec[2], pick_vec[3]);

    std::vector<double> tip_vec;
    tf2::Quaternion q_tip(0.0, std::sqrt(0.5), 0.0, std::sqrt(0.5));  // world_Y +90
    if (getInput("tip_orientation", tip_vec) && tip_vec.size() == 4) {
      q_tip = tf2::Quaternion(tip_vec[0], tip_vec[1], tip_vec[2], tip_vec[3]);
    }

    // True die orientation as it was resting, then as it is now (post-tip).
    const tf2::Quaternion q_die_rest = trueOrientation(current_face, theta_deg);
    const tf2::Quaternion q_die_now = (q_tip * q_die_rest).normalized();

    // The closing axis is tool0's local X, rigidly carried by the grasp:
    // express it once in the die's own body frame so it can be followed
    // through any candidate final orientation.
    const tf2::Quaternion q_tool_now = (q_tip * q_pick).normalized();
    const tf2::Vector3 closing_world_now = tf2::quatRotate(q_tool_now, tf2::Vector3(1, 0, 0));
    const tf2::Vector3 closing_body = tf2::quatRotate(q_die_now.inverse(), closing_world_now);

    int best_face = 0;
    double best_angle = 0.0;
    tf2::Quaternion best_delta(0, 0, 0, 1);
    bool found = false;

    for (int up_face = 1; up_face <= 6; ++up_face) {
      const tf2::Quaternion q_final = trueOrientation(up_face, 0);  // canonical: yaw 0
      const tf2::Quaternion delta = (q_final * q_die_now.inverse()).normalized();

      const tf2::Vector3 closing_world = tf2::quatRotate(q_final, closing_body);
      const double vertical = std::abs(closing_world.z());
      if (vertical > 0.7) {
        continue;  // a finger would be on the top or bottom face
      }
      const double angle = 2.0 * std::acos(std::min(1.0, std::abs(delta.w())));
      if (!found || angle < best_angle) {
        found = true;
        best_face = up_face;
        best_angle = angle;
        best_delta = delta;
      }
    }

    if (!found) {
      // Verified impossible for every (face, theta) with the standard tip,
      // so this means the inputs didn't match the real maneuver.
      throw BT::RuntimeError(
              "ComputeCanonicalPlacement: no safe canonical placement for (current_face=" +
              std::to_string(current_face) + ", theta=" + std::to_string(theta_deg) +
              ") -- check that pick_orientation/tip_orientation match the moves "
              "actually executed.");
    }

    setOutput(
      "orientation",
      std::vector<double>{best_delta.x(), best_delta.y(), best_delta.z(), best_delta.w()});
    // Halve the chosen rotation along the shortest path: averaging with
    // the identity and renormalising gives exactly half the angle about
    // the same axis.
    tf2::Quaternion d = best_delta;
    if (d.w() < 0.0) {
      d = tf2::Quaternion(-d.x(), -d.y(), -d.z(), -d.w());  // shortest path
    }
    const tf2::Quaternion half =
      tf2::Quaternion(d.x(), d.y(), d.z(), d.w() + 1.0).normalized();
    setOutput(
      "half_orientation",
      std::vector<double>{half.x(), half.y(), half.z(), half.w()});
    setOutput("placed_face", best_face);
    return BT::NodeStatus::SUCCESS;
  }

private:
  // FACE_NORMALS, matching drims_dice_simulator's dice_spawner.py exactly:
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
        throw BT::RuntimeError("ComputeCanonicalPlacement: face out of range 1..6");
    }
  }

  // Port of dice_spawner.py's get_quaternion_from_normal: the minimal
  // rotation taking +Z onto the given normal.
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

  // Orientation of a die resting with `face` up, rotated by `theta_deg`
  // about the vertical: R_yaw(theta) * inverse(quaternionFromNormal(face)).
  static tf2::Quaternion trueOrientation(int face, int theta_deg)
  {
    const tf2::Quaternion base = quaternionFromNormal(faceNormal(face)).inverse();
    tf2::Quaternion yaw;
    yaw.setRotation(tf2::Vector3(0, 0, 1), theta_deg * M_PI / 180.0);
    return (yaw * base).normalized();
  }
};
