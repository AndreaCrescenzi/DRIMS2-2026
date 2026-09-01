#pragma once

#include <array>
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

    const int yaw_deg = graspYawForTarget(current_face, target_face);

    // Same fixed "point straight down" pick orientation used throughout
    // (see demo_node.py / dice_reorient_node.py's BASE_PICK_ORIENTATION),
    // additionally yawed about dice_tf's own Z axis.
    const tf2::Quaternion base(1.0, 0.0, 0.0, 0.0);
    tf2::Quaternion yaw_q;
    yaw_q.setRPY(0.0, 0.0, yaw_deg * M_PI / 180.0);
    tf2::Quaternion pick_orientation = (yaw_q * base).normalized();

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

  static std::array<FacePair, 2> sideFacePairs(int current_face)
  {
    static const std::array<FacePair, 3> kAxisPairs = {{{1, 6}, {2, 5}, {3, 4}}};
    const int opp = oppositeFace(current_face);

    std::array<FacePair, 2> sides;
    std::size_t idx = 0;
    for (const auto & pair : kAxisPairs) {
      const bool is_own_axis =
        (pair.first == current_face && pair.second == opp) ||
        (pair.first == opp && pair.second == current_face);
      if (!is_own_axis) {
        sides[idx++] = pair;
      }
    }
    return sides;
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
