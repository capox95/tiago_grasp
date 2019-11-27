#include "../include/other_nodes.h"

NodeStatus SubscriberPose::tick()
{

    geometry_msgs::PoseConstPtr sharedPose;
    geometry_msgs::Pose pose;
    sharedPose = ros::topic::waitForMessage<geometry_msgs::Pose>(_topic, _node);
    if (sharedPose != NULL)
        pose = *sharedPose;

    //_sub = _node.subscribe("pose_goal_topic", 1, &Subscriber::callback, this);

    std::cout << "Pose goal from topic: " << pose.position.x << "," << pose.position.y << ", " << pose.position.z << std::endl;

    setOutput<geometry_msgs::Pose>("pose_msg", pose);
    return NodeStatus::SUCCESS;
}

NodeStatus SubscriberNextPose::tick()
{

    geometry_msgs::PoseConstPtr sharedPose;
    geometry_msgs::Pose pose;
    sharedPose = ros::topic::waitForMessage<geometry_msgs::Pose>(_topic, _node);
    if (sharedPose != NULL)
        pose = *sharedPose;

    //_sub = _node.subscribe("pose_goal_topic", 1, &Subscriber::callback, this);

    std::cout << "Pose goal from topic: " << pose.position.x << "," << pose.position.y << ", " << pose.position.z << std::endl;

    setOutput<geometry_msgs::Pose>("pose_msg", pose);
    return NodeStatus::SUCCESS;
}

NodeStatus TransformFrames::tick()
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = tfBuffer.lookupTransform(_target_frame, _source_frame, ros::Time(0), ros::Duration(3));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return NodeStatus::FAILURE;
    }

    auto res = getInput<geometry_msgs::Pose>("pose_in_msg");
    if (!res)
    {
        throw RuntimeError("error reading port [pose_in_msg]:", res.error());
    }
    geometry_msgs::Pose pose = res.value();

    geometry_msgs::Pose pose_out;
    tf2::doTransform(pose, pose_out, transform);

    setOutput("pose_out_msg", pose_out);

    ROS_INFO("TransformPose Node:");
    ROS_WARN("Position: %f, %f, %f", pose_out.position.x, pose_out.position.y, pose_out.position.z);
    ROS_WARN("Orientation: %f, %f, %f, %f",
             pose_out.orientation.x, pose_out.orientation.y, pose_out.orientation.z, pose_out.orientation.w);

    return NodeStatus::SUCCESS;
}

NodeStatus SleepNode::tick()
{
    ros::Duration(_value).sleep();
    return NodeStatus::SUCCESS;
}

NodeStatus BroadcastFrame::tick()
{
    auto res = getInput<geometry_msgs::Pose>("pose_in_msg");
    if (!res)
    {
        throw RuntimeError("error reading port [pose_in_msg]:", res.error());
    }
    geometry_msgs::Pose pose = res.value();

    // create message
    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped broadcast;
    broadcast.header.frame_id = _world_frame;
    broadcast.child_frame_id = _new_frame;

    broadcast.transform.translation.x = pose.position.x;
    broadcast.transform.translation.y = pose.position.y;
    broadcast.transform.translation.z = pose.position.z;

    broadcast.transform.rotation.x = pose.orientation.x;
    broadcast.transform.rotation.y = pose.orientation.y;
    broadcast.transform.rotation.z = pose.orientation.z;
    broadcast.transform.rotation.w = pose.orientation.w;

    std::cout << broadcast.header.frame_id << ", " << broadcast.child_frame_id << std::endl;

    static_broadcaster.sendTransform(broadcast);
    ROS_INFO("TF Broadcasted!");

    return NodeStatus::SUCCESS;
}