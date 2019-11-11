#include <ros/ros.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_zmq_publisher.h>
#include <memory>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "../include/arm_nodes.h"

using namespace BT;

// Simple tree, used to execute once each action.
static const char *xml_text = R"(
 <root >
     <BehaviorTree>
        <Sequence>
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

    BehaviorTreeFactory factory;

    factory.registerNodeType<MoveArmPreGrasp>("MoveArmPreGrasp");
    factory.registerNodeType<MoveArmPostGrasp>("MoveArmPostGrasp");

    auto tree = factory.createTreeFromText(xml_text);

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
    }

    // This logger prints state changes on console
    StdCoutLogger logger_cout(tree);

    //Groot
    PublisherZMQ publisher_zmq(tree);

    tree.root_node->executeTick();

    ros::shutdown();
    return 0;
}