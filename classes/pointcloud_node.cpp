#include "../include/pointcloud_node.h"
#include <chrono>
#include <ctime>

bool grasp_point_callback(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source, Eigen::Affine3d &transformation, float &margin)
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
    ef.useWeightedEntropy(true);
    ef.optimizeNumberOfClouds(true);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_result;
    bool entropy_result = ef.compute(clouds_result);
    if (entropy_result == false)
    {
        PCL_WARN("fail at entropy stage\n");
        return false;
    }

    pcl::ModelCoefficients::Ptr plane_ef = ef.getReferencePlane();

    // GRASP POINT --------------------------------------------------------------------------
    PointPose pp;
    pp.setSourceCloud(source);
    pp.setInputVectorClouds(clouds_result);
    pp.setRefPlane(plane_ef);

    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> transformation_vector;
    std::vector<float> margin_values;
    int number = pp.compute(transformation_vector, margin_values);

    //
    //
    //
    transformation = transformation_vector[0];
    margin = margin_values[0];

    return true;
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
    float margin;
    if (grasp_point_callback(cloud, transformation, margin) == false)
    {
        ROS_WARN("ERROR GRASP POINT CALLBACK!");
        return NodeStatus::FAILURE;
    }

    geometry_msgs::Pose pose_msg;
    tf::poseEigenToMsg(transformation, pose_msg);

    setOutput<geometry_msgs::Pose>("pose_out_msg", pose_msg);

    setOutput<float>("margin_out_msg", margin);

    ROS_INFO("PointCloudPose Node:");
    ROS_WARN("Position: %f, %f, %f", pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
    ROS_WARN("Orientation: %f, %f, %f, %f",
             pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w);

    ROS_WARN("Margin available: %f", margin);

    return NodeStatus::SUCCESS;
}

// --------------------------------------------------------------------------------------------------------

bool clothes_outside_callback(pcl::PointCloud<pcl::PointNormal>::Ptr &object,
                              pcl::PointCloud<pcl::PointNormal>::Ptr &scene,
                              Eigen::Affine3d &transformation, float &margin)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZRGB>);

    FindTarget ft;
    ft.object = object;
    ft.scene = scene;
    ft.apply_icp = false;
    bool alignment = ft.compute();
    if (alignment == false)
        return false;

    pcl::console::print_highlight("Starting processing to create difference map!\n");
    Processing proc;
    proc.setSceneCloud(scene);
    proc.setObjectCloud(ft.object_icp);
    proc.compute(cloud_seg);

    pcl::ModelCoefficients::Ptr plane_proc = proc.getPlaneUsed();

    pcl::console::print_highlight("Starting entropy...\n");
    // Entropy Filter
    EntropyFilter ef;
    ef.setInputCloud(cloud_seg);
    ef.setDownsampleLeafSize(0.005);     // size of the leaf for downsampling the cloud, value in meters. Default = 5 mm
    ef.setEntropyThreshold(0.6);         // Segmentation performed for all points with normalized entropy value above this
    ef.setKLocalSearch(500);             // Nearest Neighbour Local Search
    ef.setCurvatureThreshold(0.01);      // Curvature Threshold for the computation of Entropy
    ef.setDepthThreshold(0.03);          // if the segment region has a value of depth lower than this -> not graspable (value in meters)
    ef.setAngleThresholdForConvexity(5); // convexity check performed if the angle btw two normal vectors is larger than this
    ef.setReferencePlane(plane_proc);
    ef.useWeightedEntropy(true);
    ef.optimizeNumberOfClouds(true);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_result;
    bool entropy_result = ef.compute(clouds_result);
    if (entropy_result == false)
    {
        PCL_WARN("fail at entropy stage\n");
        return false;
    }
    //
    pcl::ModelCoefficients::Ptr plane_ef = ef.getReferencePlane();

    

    // GRASP POINT --------------------------------------------------------------------------
    PointPose pp;
    pp.setSourceCloud(cloud_seg);
    pp.setInputVectorClouds(clouds_result);
    pp.setRefPlane(plane_ef);

    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> transformation_vector;
    std::vector<float> margin_values;
    int number = pp.compute(transformation_vector, margin_values);

    transformation = transformation_vector[0];
    margin = margin_values[0];

    return true;
}

NodeStatus ClothesOutsidePose::tick()
{

    sensor_msgs::PointCloud2ConstPtr sharedMsg;
    sensor_msgs::PointCloud2 msg;
    sharedMsg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(_topic, _node);
    if (sharedMsg != NULL)
        msg = *sharedMsg;

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*sharedMsg, pcl_pc2);
    pcl::PointCloud<pcl::PointNormal>::Ptr scene(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromPCLPointCloud2(pcl_pc2, *scene);

    // loading object model
    pcl::PointCloud<pcl::PointNormal>::Ptr object(new pcl::PointCloud<pcl::PointNormal>);
    if (pcl::io::loadPCDFile<pcl::PointNormal>("model_today.pcd", *object) < 0)
    {
        pcl::console::print_error("Error loading object/scene file!\n");
        return NodeStatus::FAILURE;
    }

    Eigen::Affine3d transformation;
    float margin;
    if (clothes_outside_callback(object, scene, transformation, margin) == false)
    {
        ROS_WARN("ERROR GRASP POINT CALLBACK!");
        return NodeStatus::FAILURE;
    }

    geometry_msgs::Pose pose_msg;
    tf::poseEigenToMsg(transformation, pose_msg);

    setOutput<geometry_msgs::Pose>("pose_out_msg", pose_msg);

    setOutput<float>("margin_out_msg", margin);

    ROS_INFO("PointCloudPose Node:");
    ROS_WARN("Position: %f, %f, %f", pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
    ROS_WARN("Orientation: %f, %f, %f, %f",
             pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w);

    ROS_WARN("Margin available: %f", margin);

    return NodeStatus::SUCCESS;
}