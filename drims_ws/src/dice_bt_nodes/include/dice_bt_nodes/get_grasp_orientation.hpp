#pragma once

#include <array>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

// C++ port of drims_homework's dice_kinematics.py (grasp_yaw_for_target):
// given the die's current and target face, picks a grasp yaw (0 or 90 deg)
// so the gripper fingers never pinch target_face *or its opposite* -- the
// opposite matters because after rotating, the opposite of target_face
// becomes the new *bottom* face (against the table); if the fingers are
// still pinching that face, one of them ends up trapped between the die
// and the table at release time.
//
// Pure geometry, no ROS/TF needed (unlike GetFaceRotation): this only
// needs to know which face is currently up, not the die's full continuous
// orientation.
//
// Output is a full pick orientation quaternion, ready to feed MoveToPose's
// "orientation" port for the pick move (frame_id="dice_tf"): the fixed
// "point straight down" base orientation, additionally yawed by the
// computed angle around dice_tf's own Z axis.
//
// Optional input "flip_grasp" (default false): adds 180deg to the chosen
// yaw. A 180deg yaw flip pinches the *same* face pair (it's a rotation
// about the same closing axis, just approached from the opposite
// rotational direction), so it's exactly as safe w.r.t. target_face/its
// opposite as the unflipped yaw -- but it makes the arm reach the pick
// pose via a different wrist configuration. Exists because a specific
// rotation direction can be kinematically blocked (joint limit) for one
// grasp approach while the flipped approach, needing a different wrist
// path to reach the same physical grasp, is not -- observed empirically,
// not something the geometry alone predicts. See dice_challenge.xml's
// MainTree, which tries the flipped grasp as a fallback before resorting
// to a detour_face route.
//
// Optional input "tilt_deg": tilts the whole approach away from
// straight-down by this many degrees, applied as a rotation about the
// *local* X axis of the already-yawed pick orientation (post-
// multiplication, i.e. in the gripper's own body frame, not the world's)
// -- local X is the one axis BASE_PICK_ORIENTATION's 180deg-about-X flip
// leaves unchanged, and by construction it stays the gripper's closing
// axis after any yaw. Tilting about that same axis rotates the approach
// vector while leaving the closing axis -- and therefore which face pair
// gets pinched -- untouched: still grasps the same two side faces, just
// diagonally instead of from straight above.
//
// If NOT explicitly provided, the sign is derived from geometry instead
// of defaulting to a fixed value: the upcoming reorientation (see
// GetFaceRotation) is itself a rotation about this SAME closing axis --
// the die can only rotate, while staying gripped by the same two faces,
// about the line connecting them -- so the "rotation_quat" input (pass
// GetFaceRotation's own output straight through) directly tells us which
// way that rotation goes. Verified empirically (see dice_challenge.xml /
// project memory) that the natural, no-escalation-needed tilt sign is
// the OPPOSITE of the sign of the rotation's component along the closing
// axis: e.g. current_face=2,target_face=1 rotates +90deg about the
// closing axis (yaw=90 there, so that's rotation_quat's y component) and
// wants tilt=-45; current_face=1,target_face=3 rotates -90deg about its
// closing axis (yaw=0, x component) and wants tilt=+45 -- opposite
// rotation sign, opposite tilt sign, confirmed on both. Explicit
// "tilt_deg" (e.g. a fallback tier deliberately trying the other sign)
// always overrides this.

class GetGraspOrientation : public BT::SyncActionNode
{
public:
  GetGraspOrientation(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("current_face"),
      BT::InputPort<int>("target_face"),
      BT::InputPort<bool>("flip_grasp"),
      BT::InputPort<double>("tilt_deg"),
      BT::InputPort<std::vector<double>>("rotation_quat"),
      BT::InputPort<double>("yaw_offset_deg"),
      BT::OutputPort<std::vector<double>>("pick_orientation"),
      BT::OutputPort<int>("grasp_yaw_deg"),
    };
  }

  BT::NodeStatus tick() override
  {
    int current_face, target_face;

    if (!getInput("current_face", current_face)) {
      throw BT::RuntimeError("Missing parameter [current_face]");
    }
    if (!getInput("target_face", target_face)) {
      throw BT::RuntimeError("Missing parameter [target_face]");
    }
    bool flip_grasp = false;
    getInput("flip_grasp", flip_grasp);  // optional, defaults to false

    const int yaw_deg_unflipped = graspYawForTarget(current_face, target_face);

    double tilt_deg;
    if (!getInput("tilt_deg", tilt_deg)) {
      // Not explicitly set: derive the sign from the upcoming rotation
      // (see the class comment). yaw_deg_unflipped==0 means the closing
      // axis is local X (rotation_quat's x component carries the signed
      // angle about it); ==90 means local Y (y component). flip_grasp
      // doesn't change WHICH axis is the closing axis (0/180 and 90/270
      // are the same line, approached from opposite sides), so use the
      // unflipped yaw here regardless of flip_grasp.
      std::vector<double> rotation_quat;
      double component = 0.0;
      if (getInput("rotation_quat", rotation_quat) && rotation_quat.size() >= 2) {
        component = (yaw_deg_unflipped == 0) ? rotation_quat[0] : rotation_quat[1];
      }
      if (component > 1e-6) {
        tilt_deg = -45.0;
      } else if (component < -1e-6) {
        tilt_deg = 45.0;
      } else {
        // No rotation info, or ~zero rotation (current_face==target_face):
        // sign doesn't matter: fall back to the empirically-good default.
        tilt_deg = -45.0;
      }
    }

    int yaw_deg = yaw_deg_unflipped;
    if (flip_grasp) {
      yaw_deg = (yaw_deg + 180) % 360;
    }
    // Experimental/diagnostic: an explicit extra yaw rotation on top of
    // everything above, independent of flip_grasp (180) and tilt_deg.
    // Exists to test, on the real robot/sim, whether a specific
    // current_face/target_face pair needs a 90deg-rotated approach when
    // tilt is combined with it -- observed at least once (3->2) that the
    // tilt can bring a finger physically onto target_face's surface even
    // though the *untilted* pinch pair correctly avoids it (see
    // dice_challenge.xml's history). Not yet understood well enough to
    // derive automatically -- pass explicitly per case until it is.
    double yaw_offset_deg = 0.0;
    getInput("yaw_offset_deg", yaw_offset_deg);
    if (yaw_offset_deg != 0.0) {
      yaw_deg = static_cast<int>(
        std::lround(yaw_deg + yaw_offset_deg)) % 360;
    }

    // Same fixed "point straight down" pick orientation used throughout
    // (see demo_node.py / dice_reorient_node.py's BASE_PICK_ORIENTATION),
    // additionally yawed about dice_tf's own Z axis.
    const tf2::Quaternion base(1.0, 0.0, 0.0, 0.0);
    tf2::Quaternion yaw_q;
    yaw_q.setRPY(0.0, 0.0, yaw_deg * M_PI / 180.0);
    tf2::Quaternion pick_orientation = (yaw_q * base).normalized();

    if (tilt_deg != 0.0) {
      tf2::Quaternion tilt_q;
      tilt_q.setRPY(tilt_deg * M_PI / 180.0, 0.0, 0.0);
      // Post-multiply: apply in the gripper's own (already-yawed) local
      // frame, rotating about its closing axis rather than the world's.
      pick_orientation = (pick_orientation * tilt_q).normalized();
    }

    setOutput(
      "pick_orientation",
      std::vector<double>{
        pick_orientation.x(), pick_orientation.y(),
        pick_orientation.z(), pick_orientation.w()});
    setOutput("grasp_yaw_deg", yaw_deg);

    return BT::NodeStatus::SUCCESS;
  }

private:
  using FacePair = std::pair<int, int>;

  static int oppositeFace(int face) {return 7 - face;}

  // Which face pair does yaw=0 vs yaw=90 actually pinch? Derived from
  // dice_spawner.py's get_quaternion_from_normal (the rotation that
  // defines each face{N}_tf, and therefore "dice_tf" = face{current}_tf,
  // relative to dice_com_tf, the die's own fixed body frame where
  // FACE_NORMALS is defined: 1<->-Z, 6<->+Z, 2<->-X, 5<->+X, 3<->+Y,
  // 4<->-Y). yaw=0's closing axis is always dice_tf's own local X (the
  // axis BASE_PICK_ORIENTATION's flip leaves unchanged -- see tilt_deg's
  // comment); expressing that axis in dice_com_tf terms for each of the
  // 6 possible current_face values (by applying get_quaternion_from_
  // normal's actual rotation, not assuming a fixed relationship) gives:
  // yaw=0 pinches the {2,5} (local X) pair EXCEPT when current_face
  // itself is 2 or 5 (own axis, can't pinch it), in which case yaw=0
  // pinches {1,6} (local Z) instead. This is NOT the same as "whichever
  // pair comes first in a fixed table order after excluding current's
  // own axis" -- a previous version of this function assumed that,
  // which happens to agree with the derivation below for current_face
  // in {1,2,5,6} but is swapped (yaw=0 and yaw=90 reversed) for
  // current_face in {3,4} -- get_quaternion_from_normal's minimal-
  // rotation construction doesn't preserve a consistent axis labeling
  // across all 6 faces, only verified/assumed to for four of them.
  // Confirmed against real behavior: current_face=3 with the old
  // (wrong) mapping put a finger on the target face after rotating,
  // even though the *intended* pinch pair correctly excluded it -- the
  // grasp was pinching the pair the code THOUGHT it was avoiding.
  static std::array<FacePair, 2> sideFacePairs(int current_face)
  {
    static const FacePair kXPair = {2, 5};
    static const FacePair kYPair = {3, 4};
    static const FacePair kZPair = {1, 6};

    const bool current_on_x_axis = (current_face == 2 || current_face == 5);
    const FacePair pinched_at_yaw0 = current_on_x_axis ? kZPair : kXPair;

    const int opp = oppositeFace(current_face);
    FacePair pinched_at_yaw90 = kYPair;  // overwritten below unless current is on Y axis
    for (const auto & pair : {kXPair, kYPair, kZPair}) {
      const bool is_own_axis =
        (pair.first == current_face && pair.second == opp) ||
        (pair.first == opp && pair.second == current_face);
      const bool is_yaw0_pair =
        (pair.first == pinched_at_yaw0.first && pair.second == pinched_at_yaw0.second);
      if (!is_own_axis && !is_yaw0_pair) {
        pinched_at_yaw90 = pair;
        break;
      }
    }
    return {pinched_at_yaw0, pinched_at_yaw90};
  }

  static bool contains(const FacePair & pair, int face)
  {
    return pair.first == face || pair.second == face;
  }

  static int graspYawForTarget(int current_face, int target_face)
  {
    const auto sides = sideFacePairs(current_face);
    const FacePair & pinched_at_yaw0 = sides[0];
    const FacePair & pinched_at_yaw90 = sides[1];

    if (contains(pinched_at_yaw0, target_face)) {
      return 90;
    }
    if (contains(pinched_at_yaw90, target_face)) {
      return 0;
    }
    // target_face is current_face itself or its opposite: yaw doesn't matter.
    return 0;
  }
};
