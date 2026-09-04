#pragma once

#include <array>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

// Decides how many 90deg rotations reach the target face: none if it is
// already up, one if adjacent, two through an intermediate side face if
// opposite. A single 180deg flip has no unique axis and used to wedge
// motion_server, hence the two-step path.
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
