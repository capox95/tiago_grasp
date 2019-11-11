#include "../include/gripper_nodes.h"

using namespace BT;

NodeStatus OpenGripper::tick()
{
    bool success = _gripper_node->openGripper();
    if (!success)
        return NodeStatus::FAILURE;

    return NodeStatus::SUCCESS;
}

NodeStatus CloseGripper::tick()
{
    bool success = _gripper_node->closeGripper();
    if (!success)
        return NodeStatus::FAILURE;

    return NodeStatus::SUCCESS;
}

NodeStatus MoveGripperTo::tick()
{
    double value = 0.005;
    bool success = _gripper_node->moveToJointValue(value);
    if (!success)
        return NodeStatus::FAILURE;

    return NodeStatus::SUCCESS;
}
