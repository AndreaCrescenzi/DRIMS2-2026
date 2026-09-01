#pragma once

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

// Decides how many pick-rotate-place cycles are needed to bring
// target_face up, and which intermediate face to pass through for the
// two-cycle (opposite-face) case.
//
// - current_face == target_face            -> num_steps = 0
// - target_face == 7 - current_face (opp.) -> num_steps = 2, with a valid
//   intermediate side face (any side face works; deterministic pick: 1,
//   unless current_face is already on the (1,6) axis, then 2 -- both are
//   always guaranteed to differ from both current_face and target_face)
// - otherwise (adjacent)                    -> num_steps = 1
//
// Pure geometry, no ROS/TF needed -- only needs which face is currently up.
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

    if (current_face == target_face) {
      num_steps = 0;
    } else if (target_face == 7 - current_face) {
      num_steps = 2;
      intermediate = (current_face == 1 || current_face == 6) ? 2 : 1;
    } else {
      num_steps = 1;
    }

    setOutput("num_steps", num_steps);
    setOutput("intermediate_face", intermediate);

    return BT::NodeStatus::SUCCESS;
  }
};
