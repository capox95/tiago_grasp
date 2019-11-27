#include "../include/pointcloud_node.h"
#include <chrono>
#include <ctime>

bool grasp_point_callback(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source, Eigen::Affine3d &transformation)
{

    //BIN SEGMENTATION -----------------------------------------------------------------------
    BinSegmentation bin;
    bin.setInputCloud(source);
    bin.setNumberLines(4);
    bin.setPaddingDistance(0.12); // 5cm from the bin walls
    bin.setMaxBinHeight(0.3);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_grasp(new pcl::PointCloud<pcl::PointXYZRGB>);
    bool bin_result = bin.compute(cloud_grasp);
    if (bin_result == false)
        return false;

    pcl::ModelCoefficients::Ptr plane = bin.getPlaneGroundPoints();
    pcl::PointCloud<pcl::PointXYZ>::Ptr top_vertices = bin.getVerticesBinContour();

    // ENTROPY FILTER -----------------------------------------------------------------------
    //
    EntropyFilter ef;
    ef.setInputCloud(cloud_grasp);
    ef.setVerticesBinContour(top_vertices);
    ef.setDownsampleLeafSize(0.005);
    ef.setEntropyThreshold(0.65);
    ef.setKLocalSearch(500);        // Nearest Neighbour Local Search
    ef.setCurvatureThreshold(0.01); //Curvature Threshold for the computation of Entropy
    ef.setDepthThreshold(0.03);
    ef.setAngleThresholdForConvexity(5);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZ>);
    bool entropy_result = ef.compute(cloud_result);
    if (entropy_result == false)
    {
        PCL_WARN("fail at entropy stage\n");
        return false;
    }

    //depth = ef.getDepthValue();

    // GRASP POINT --------------------------------------------------------------------------
    PointPose pp;
    pp.setSourceCloud(source);
    pp.setRefPlane(plane);
    pp.setInputCloud(cloud_result);

    if (pp.computeGraspPoint(transformation))
    {
        //std::cout << transformation.matrix() << std::endl;
        return true;
    }
    else
        return false;
}

NodeStatus PointCloudPose::tick()
{

    sensor_msgs::PointCloud2ConstPtr sharedMsg;
    sensor_msgs::PointCloud2 msg;
    sharedMsg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(_topic, _node);
    if (sharedMsg != NULL)
        msg = *sharedMsg;

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*sharedMsg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    Eigen::Affine3d transformation;
    if (grasp_point_callback(cloud, transformation) == false)
    {
        ROS_WARN("ERROR GRASP POINT CALLBACK!");
        return NodeStatus::FAILURE;
    }

    geometry_msgs::Pose pose_msg;
    tf::poseEigenToMsg(transformation, pose_msg);

    setOutput<geometry_msgs::Pose>("pose_out_msg", pose_msg);

    ROS_INFO("PointCloudPose Node:");
    ROS_WARN("Position: %f, %f, %f", pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
    ROS_WARN("Orientation: %f, %f, %f, %f",
             pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w);

    return NodeStatus::SUCCESS;
}