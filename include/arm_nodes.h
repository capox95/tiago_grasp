#include <behaviortree_cpp/behavior_tree.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include "arm_class.h"

using namespace BT;

// MOVE ARM TO POSE ------------------------------------------------------------------------------------------------------//
class MoveArmToPose : public SyncActionNode
{
public:
    // additional arguments passed to the constructor
    MoveArmToPose(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name, config) {}

    void init(std::shared_ptr<ArmControl> ptr_arm, float value_offset)
    {
        _arm_node = ptr_arm;
        _offset = value_offset;
    }

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        const char *description = "goal pose";
        return {InputPort<std::vector<double>>("target", description)};
    }

private:
    std::shared_ptr<ArmControl> _arm_node;
    float _offset;
};

// MOVE ARM PRE GRASP POSE------------------------------------------------------------------------------------------------//
class MoveArmPreGrasp : public SyncActionNode
{
public:
    // additional arguments passed to the constructor
    MoveArmPreGrasp(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name, config) {}

    void init(std::shared_ptr<ArmControl> ptr_arm, float value_offset, float distance)
    {
        _arm_node = ptr_arm;
        _offset = value_offset;
        _approaching_distance = distance;

    }

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        const char *description = "goal pose";
        return {InputPort<geometry_msgs::Pose>("target", description)};
    }

private:
    std::shared_ptr<ArmControl> _arm_node;
    float _offset, _approaching_distance;
};

// MOVE ARM POST GRASP POSE------------------------------------------------------------------------------------------------//
class MoveArmPostGrasp : public SyncActionNode
{
public:
    // additional arguments passed to the constructor
    MoveArmPostGrasp(const std::string &name) : SyncActionNode(name, {}) {}

    void init(std::shared_ptr<ArmControl> ptr_arm, float distance)
    {
        _arm_node = ptr_arm;
        _distance = distance;
    }

    NodeStatus tick() override;

private:
    std::shared_ptr<ArmControl> _arm_node;
    float _distance;
};
