#include "../include/arm_nodes.h"

// Template specialization to converts a string to geometry_msgs::Pose.
namespace BT
{
template <>
inline geometry_msgs::Pose convertFromString(StringView str)
{
    // The next line should be removed...
    printf("Converting string: \"%s\"\n", str.data());

    // We expect real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() != 7)
    {
        throw RuntimeError("invalid input)");
    }
    else
    {
        geometry_msgs::Pose output;
        output.position.x = convertFromString<double>(parts[0]);
        output.position.y = convertFromString<double>(parts[1]);
        output.position.z = convertFromString<double>(parts[2]);
        output.orientation.x = convertFromString<double>(parts[3]);
        output.orientation.y = convertFromString<double>(parts[4]);
        output.orientation.z = convertFromString<double>(parts[5]);
        output.orientation.w = convertFromString<double>(parts[6]);
        return output;
    }
}
} // end namespace BT

// Template specialization to converts a string to std::vector<double>.
namespace BT
{
template <>
inline std::vector<double> convertFromString(StringView str)
{
    // The next line should be removed...
    printf("Converting string: \"%s\"\n", str.data());

    // We expect real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() != 6)
    {
        throw RuntimeError("invalid input)");
    }
    else
    {
        std::vector<double> output;
        for (int i = 0; i < 6; i++)
            output.push_back(convertFromString<double>(parts[i]));
        return output;
    }
}
} // end namespace BT

using namespace BT;

NodeStatus MoveArmToPose::tick()
{
    auto res = getInput<std::vector<double>>("target");
    if (!res)
    {
        throw RuntimeError("error reading port [target]:", res.error());
    }
    std::vector<double> msg = res.value();

    geometry_msgs::Pose goal;
    goal.position.x = msg.at(0);
    goal.position.y = msg.at(1);
    goal.position.z = msg.at(2);

    tf2::Quaternion orientation;
    orientation.setRPY(msg.at(3), msg.at(4), msg.at(5));
    goal.orientation = tf2::toMsg(orientation);

    bool success = _arm_node->moveToPose(goal);
    if (!success)
        return NodeStatus::FAILURE;

    return NodeStatus::SUCCESS;
}

NodeStatus MoveArmPreGrasp::tick()
{

    auto res = getInput<geometry_msgs::Pose>("target");
    if (!res)
    {
        throw RuntimeError("error reading port [target]:", res.error());
    }
    geometry_msgs::Pose pose = res.value();

    pose.position.z += _offset;

    bool success = _arm_node->preGraspApproach(pose, _approaching_distance);
    if (!success)
        return NodeStatus::FAILURE;

    return NodeStatus::SUCCESS;
}

NodeStatus MoveArmPostGrasp::tick()
{
    geometry_msgs::Pose goal;
    goal.position.x = 0.46;
    goal.position.y = -0.56;
    goal.position.z = 1.11;

    goal.orientation.x = 0.1589;
    goal.orientation.y = -0.2806;
    goal.orientation.z = -0.0359;
    goal.orientation.w = 0.9459;

    bool success = _arm_node->postGraspApproach(goal, _distance);
    if (!success)
        return NodeStatus::FAILURE;

    return NodeStatus::SUCCESS;
}

NodeStatus MoveArmPreRecovery::tick()
{

    auto res = getInput<geometry_msgs::Pose>("target");
    if (!res)
    {
        throw RuntimeError("error reading port [target]:", res.error());
    }
    geometry_msgs::Pose pose = res.value();

    pose.position.x -= _offset;

    bool success = _arm_node->preRecoveryApproach(pose, _distance);
    if (!success)
        return NodeStatus::FAILURE;

    return NodeStatus::SUCCESS;
}

NodeStatus MoveArmPostRecovery::tick()
{
    geometry_msgs::Pose goal;
    goal.position.x = 0.59;
    goal.position.y = 0.05;
    goal.position.z = 0.5;

    goal.orientation.x = -0.660510;
    goal.orientation.y = -0.033602;
    goal.orientation.z = 0.019429;
    goal.orientation.w = 0.749814;

    bool success = _arm_node->postRecoveryApproach(goal, _distance);
    if (!success)
        return NodeStatus::FAILURE;

    return NodeStatus::SUCCESS;
}