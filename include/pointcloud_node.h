#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen_conversions/eigen_msg.h>

#include "../include/entropy.h"
#include "../include/binsegmentation.h"
#include "../include/pointpose.h"

#include <behaviortree_cpp/behavior_tree.h>

using namespace BT;

class PointCloudPose : public SyncActionNode
{
public:
    PointCloudPose(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name, config)
    {
    }

    void init(ros::NodeHandle &node, std::string &topic)
    {
        _node = node;
        _topic = topic;
    }

    static PortsList providedPorts()
    {
        return {OutputPort<geometry_msgs::Pose>("pose_out_msg"), OutputPort<float>("depth_msg")};
    }

    NodeStatus tick() override;

private:
    ros::NodeHandle _node;
    std::string _topic;
};