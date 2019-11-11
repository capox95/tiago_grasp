#include <ros/ros.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_zmq_publisher.h>
#include <memory>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "../include/gripper_nodes.h"

using namespace BT;

// Simple tree, used to execute once each action.
static const char *xml_text = R"(
 <root >
     <BehaviorTree>
        <Sequence>
            <OpenGripper />
            <CloseGripper />
            <MoveGripperTo />
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_sequence_test");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::shared_ptr<GripperControl> ptr_gripper = std::make_shared<GripperControl>();

    BehaviorTreeFactory factory;

    factory.registerNodeType<OpenGripper>("OpenGripper");
    factory.registerNodeType<CloseGripper>("CloseGripper");
    factory.registerNodeType<MoveGripperTo>("MoveGripperTo");

    auto tree = factory.createTreeFromText(xml_text);

    for (auto &node : tree.nodes)
    {

        if (auto openAction = dynamic_cast<OpenGripper *>(node.get()))
        {
            openAction->init(ptr_gripper);
        }
        if (auto closeAction = dynamic_cast<CloseGripper *>(node.get()))
        {
            closeAction->init(ptr_gripper);
        }
        if (auto movetoAction = dynamic_cast<MoveGripperTo *>(node.get()))
        {
            movetoAction->init(ptr_gripper);
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