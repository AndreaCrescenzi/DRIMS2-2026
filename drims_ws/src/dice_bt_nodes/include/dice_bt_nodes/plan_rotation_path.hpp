#pragma once

#include <array>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

// Decides how many pick-rotate-place cycles are needed to bring
// target_face up, and which face(s) to pass through.
//
// - current_face == target_face            -> num_steps = 0
// - target_face == 7 - current_face (opp.) -> num_steps = 2, with a valid
//   intermediate side face (any side face works; deterministic pick: 1,
//   unless current_face is already on the (1,6) axis, then 2 -- both are
//   always guaranteed to differ from both current_face and target_face)
// - otherwise (adjacent)                    -> num_steps = 1
//
// Also always computes detour_face: a face adjacent to *both*
// current_face and target_face (there are always exactly two candidates
// for an adjacent pair; the smaller-numbered one is picked
// deterministically). This exists because a direct adjacent rotation can
// still be kinematically infeasible in one specific rotation direction
// even with unlimited lift clearance -- observed empirically: rotating
// the same ~90 degrees one way succeeds reliably, the other way fails
// consistently regardless of margin, i.e. a joint-limit issue, not a
// reachability/clearance one. Routing current_face -> detour_face ->
// target_face replaces the single problematic rotation with two
// differently-axed ones, as a fallback the tree can try when the direct
// path exhausts its own retries. Meaningless (left as 0) when num_steps
// is 0 or 2, since those cases already have their own designated path.
class PlanRotationPath : public BT::SyncActionNode
{
public:
  PlanRotationPath(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("current_face"),
      BT::InputPort<int>("target_face"),
      BT::OutputPort<int>("num_steps"),
      BT::OutputPort<int>("intermediate_face"),
      BT::OutputPort<int>("detour_face"),
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

    int num_steps;
    int intermediate = 0;
    int detour = 0;

    if (current_face == target_face) {
      num_steps = 0;
    } else if (target_face == 7 - current_face) {
      num_steps = 2;
      intermediate = (current_face == 1 || current_face == 6) ? 2 : 1;
    } else {
      num_steps = 1;
      const int opp_current = 7 - current_face;
      const int opp_target = 7 - target_face;
      for (int face = 1; face <= 6; ++face) {
        if (face != current_face && face != opp_current &&
          face != target_face && face != opp_target)
        {
          detour = face;
          break;
        }
      }
    }

    setOutput("num_steps", num_steps);
    setOutput("intermediate_face", intermediate);
    setOutput("detour_face", detour);

    return BT::NodeStatus::SUCCESS;
  }
};
