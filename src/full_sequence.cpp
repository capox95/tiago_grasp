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
#include "../include/gripper_nodes.h"

using namespace BT;

// Simple tree, used to execute once each action.
static const char *xml_text = R"(
 <root >
     <BehaviorTree>
            <Sequence>
                <OpenGripper />
                <PointCloudPose pose_out_msg="{pose_camera_frame}" margin_out_msg="{margin_value}" />
                <TransformFrames pose_in_msg="{pose_camera_frame}" pose_out_msg="{pose_world_frame}" />
                <MoveArmPreGrasp target="{pose_world_frame}" margin_in_msg="{margin_value}" />
                <CloseGripper />
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

    std::shared_ptr<ArmControl> ptr_arm = std::allocate_shared<ArmControl>(Eigen::aligned_allocator<ArmControl>());

    std::shared_ptr<GripperControl> ptr_gripper = std::make_shared<GripperControl>();

    BehaviorTreeFactory factory;

    factory.registerNodeType<MoveArmPreGrasp>("MoveArmPreGrasp");
    factory.registerNodeType<MoveArmPostGrasp>("MoveArmPostGrasp");

    factory.registerNodeType<PointCloudPose>("PointCloudPose");
    factory.registerNodeType<TransformFrames>("TransformFrames");
    factory.registerNodeType<SleepNode>("SleepNode");

    factory.registerNodeType<OpenGripper>("OpenGripper");
    factory.registerNodeType<CloseGripper>("CloseGripper");

    auto tree = factory.createTreeFromText(xml_text);

    std::string topic_pc = "/xtion/depth_registered/points";
    std::string target_frame = "base_footprint";
    std::string source_frame = "xtion_rgb_optical_frame";

    float offset_gripper_tf = 0.22; //offset between tip of gripper fingers and ee-effector frame ("arm_tool_link") = 0.22 m
    float distance = 0.3;           // distance travelled vertically (approach and departure)

    float sleep = 0.1;

    for (auto &node : tree.nodes)
    {
        // arm
        if (auto actionPreGrasp = dynamic_cast<MoveArmPreGrasp *>(node.get()))
        {
            actionPreGrasp->init(ptr_arm, offset_gripper_tf, distance);
        }
        if (auto actionPostGrasp = dynamic_cast<MoveArmPostGrasp *>(node.get()))
        {
            actionPostGrasp->init(ptr_arm, distance + 0.15);
        }

        // pointcloud and tfs
        if (auto pointcloudNode = dynamic_cast<PointCloudPose *>(node.get()))
        {
            pointcloudNode->init(node_handle, topic_pc);
        }
        if (auto tfNode = dynamic_cast<TransformFrames *>(node.get()))
        {
            tfNode->init(node_handle, target_frame, source_frame);
        }
        if (auto sleepNode = dynamic_cast<SleepNode *>(node.get()))
        {
            sleepNode->init(sleep);
        }

        // gripper
        if (auto openAction = dynamic_cast<OpenGripper *>(node.get()))
        {
            openAction->init(ptr_gripper);
        }
        if (auto closeAction = dynamic_cast<CloseGripper *>(node.get()))
        {
            closeAction->init(ptr_gripper);
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