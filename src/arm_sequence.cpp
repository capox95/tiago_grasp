#include <ros/ros.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_zmq_publisher.h>
#include <memory>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "../include/arm_nodes.h"
#include "../include/other_nodes.h"
#include "../include/pointcloud_node.h"

using namespace BT;

// Simple tree, used to execute once each action.
static const char *xml_text = R"(
 <root >
     <BehaviorTree>
        <Sequence>
            <PointCloudPose pose_out_msg="{pose_camera_frame}" />
            <TransformFrames pose_in_msg="{pose_camera_frame}" pose_out_msg="{pose_world_frame}" />
            <MoveArmPreGrasp target="{pose_world_frame}" />
            <MoveArmPostGrasp />
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_sequence_test");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::shared_ptr<ArmControl> ptr_arm = std::make_shared<ArmControl>(
        "arm_torso", "kdl_kinematics_plugin/KDLKinematicsPlugin", "base_link", "arm_tool_link");

    std::shared_ptr<ArmControl> ptr_arm2 = std::make_shared<ArmControl>(
        "arm_torso", "kdl_kinematics_plugin/KDLKinematicsPlugin", "base_link", "arm_tool_link");

    BehaviorTreeFactory factory;

    factory.registerNodeType<MoveArmPreGrasp>("MoveArmPreGrasp");
    factory.registerNodeType<MoveArmPostGrasp>("MoveArmPostGrasp");

    factory.registerNodeType<PointCloudPose>("PointCloudPose");
    factory.registerNodeType<TransformFrames>("TransformFrames");

    auto tree = factory.createTreeFromText(xml_text);

    std::string topic_pc = "/xtion/depth_registered/points";
    std::string target_frame = "base_link";
    std::string source_frame = "xtion_optical_frame";
    for (auto &node : tree.nodes)
    {

        if (auto actionPreGrasp = dynamic_cast<MoveArmPreGrasp *>(node.get()))
        {
            actionPreGrasp->init(ptr_arm);
        }
        if (auto actionPostGrasp = dynamic_cast<MoveArmPostGrasp *>(node.get()))
        {
            actionPostGrasp->init(ptr_arm);
        }
        if (auto pointcloudNode = dynamic_cast<PointCloudPose *>(node.get()))
        {
            pointcloudNode->init(node_handle, topic_pc);
        }
        if (auto tfNode = dynamic_cast<TransformFrames *>(node.get()))
        {
            tfNode->init(node_handle, target_frame, source_frame);
        }
    }

    // This logger prints state changes on console
    StdCoutLogger logger_cout(tree);

    //Groot
    PublisherZMQ publisher_zmq(tree);

    tree.root_node->executeTick();

    ros::shutdown();
    return 0;
}