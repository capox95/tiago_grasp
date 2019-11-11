#include <behaviortree_cpp/behavior_tree.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>

using namespace BT;

class SubscriberPose : public SyncActionNode
{
public:
    SubscriberPose(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name, config)
    {
    }

    void init(ros::NodeHandle &node, std::string &topic)
    {
        _node = node;
        _topic = topic;
    }

    static PortsList providedPorts()
    {
        return {OutputPort<geometry_msgs::Pose>("pose_msg")};
    }

    NodeStatus tick() override;

private:
    ros::NodeHandle _node;
    std::string _topic;
};

class SubscriberNextPose : public SyncActionNode
{
public:
    SubscriberNextPose(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name, config)
    {
    }

    void init(ros::NodeHandle &node, std::string &topic)
    {
        _node = node;
        _topic = topic;
    }

    static PortsList providedPorts()
    {
        return {OutputPort<geometry_msgs::Pose>("pose_msg")};
    }

    NodeStatus tick() override;

private:
    ros::NodeHandle _node;
    std::string _topic;
};

class TransformFrames : public SyncActionNode
{
public:
    TransformFrames(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name, config)
    {
    }

    void init(ros::NodeHandle &node, std::string target, std::string source)
    {
        _node = node;
        _target_frame = target;
        _source_frame = source;
    }

    static PortsList providedPorts()
    {
        return {InputPort<geometry_msgs::Pose>("pose_in_msg"), OutputPort<geometry_msgs::Pose>("pose_out_msg")};
    }

    NodeStatus tick() override;

private:
    ros::NodeHandle _node;
    std::string _target_frame;
    std::string _source_frame;
};

class SleepNode : public SyncActionNode
{
public:
    SleepNode(const std::string &name) : SyncActionNode(name, {})
    {
    }

    void init(double delay)
    {
        _value = delay;
    }

    NodeStatus tick() override;

private:
    double _value;
};

class BroadcastFrame : public SyncActionNode
{
public:
    BroadcastFrame(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name, config)
    {
    }

    void init(ros::NodeHandle &node, std::string worldframe, std::string newframe)
    {
        _node = node;
        _world_frame = worldframe;
        _new_frame = newframe;
    }

    static PortsList providedPorts()
    {
        return {InputPort<geometry_msgs::Pose>("pose_in_msg")};
    }

    NodeStatus tick() override;

private:
    ros::NodeHandle _node;
    std::string _world_frame;
    std::string _new_frame;
};