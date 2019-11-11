#include <behaviortree_cpp/behavior_tree.h>
#include <ros/ros.h>
#include "gripper_class.h"

using namespace BT;

// MOVE ARM TO POSE ------------------------------------------------------------------------------------------------------//
class OpenGripper : public SyncActionNode
{
public:
    // additional arguments passed to the constructor
    OpenGripper(const std::string &name) : SyncActionNode(name, {}) {}

    void init(std::shared_ptr<GripperControl> ptr_gripper)
    {
        _gripper_node = ptr_gripper;
    }

    NodeStatus tick() override;

private:
    std::shared_ptr<GripperControl> _gripper_node;
};

// MOVE ARM PRE GRASP POSE------------------------------------------------------------------------------------------------//
class CloseGripper : public SyncActionNode
{
public:
    // additional arguments passed to the constructor
    CloseGripper(const std::string &name) : SyncActionNode(name, {}) {}

    void init(std::shared_ptr<GripperControl> ptr_gripper)
    {
        _gripper_node = ptr_gripper;
    }

    NodeStatus tick() override;

private:
    std::shared_ptr<GripperControl> _gripper_node;
};

// MOVE ARM POST GRASP POSE------------------------------------------------------------------------------------------------//
class MoveGripperTo : public SyncActionNode
{
public:
    // additional arguments passed to the constructor
    MoveGripperTo(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name, config) {}

    void init(std::shared_ptr<GripperControl> ptr_gripper)
    {
        _gripper_node = ptr_gripper;
    }

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        const char *description = "target value";
        return {InputPort<double>("target", description)};
    }

private:
    std::shared_ptr<GripperControl> _gripper_node;
};
