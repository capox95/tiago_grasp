#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>

#include "../include/entropyOut.h"

void EntropyFilter::setVerticesBinContour(pcl::PointCloud<pcl::PointXYZ>::Ptr vertices)
{
    m_top_vertices = vertices;
    _flag_vertices = true;
}

pcl::ModelCoefficients::Ptr EntropyFilter::getReferencePlane() { return m_plane; }

bool EntropyFilter::compute(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds_out)
{
    downsample(m_source, m_leafsize, m_cloud_downsample);
    computePolyFitting(m_cloud_downsample, m_mls_points);
    divideCloudNormals(m_mls_points, m_mls_cloud, m_mls_normals);
    getSpherical(m_mls_normals, m_spherical);

    // depth interval estimation
    if (_flag_vertices == true && _flag_plane_ref == false)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>);
        vertices = m_top_vertices;
        computePlaneBottomBin(m_mls_cloud, vertices, m_plane);
    }
    else if (_flag_vertices == false && _flag_plane_ref == true)
    {
        std::cout << "Plane Reference already available" << std::endl;
        m_plane = m_plane_ref;
    }
    else
    {
        computePlaneReference(m_mls_cloud, m_plane);
    }

    m_depth_interval = getPointPlaneDistanceCloud(m_mls_cloud, m_cloud_depth, m_plane);
    if (m_depth_interval < m_depth_threshold) //less than 3 cm
    {
        PCL_WARN("Depth interval very small, grasp is not achievable! value: [%f mm]\n", m_depth_interval * 1000);
        return false;
    }
    //downsampleCloudAndNormals(m_mls_cloud, m_mls_normals, 0.001, m_convexity_ready);
    localSearchForConvexity(m_mls_cloud, m_mls_normals, m_cloud_convexity);

    combineConvexityAndCurvatureInfo(m_cloud_convexity, m_mls_normals, m_cloud_combined);

    local_search(m_mls_cloud, m_spherical, m_cloud_combined, m_cloud_depth);
    normalizeEntropy(m_spherical);

    if (_max_entropy < 1.0)
    {
        PCL_WARN("Entropy too small!\n");
        return false;
    }

    segmentCloudEntropy(m_mls_points, m_spherical, m_cloud_seg, m_entropy_threshold);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_connected;
    alternativeConnectedComponets(m_cloud_seg, clouds_connected);

    if (_flag_optimization)
        optimizeNumberOfClouds(clouds_connected, clouds_out);
    else
        clouds_out = clouds_connected;

    return true;
}

// -----------------------------------------------------------------------------------------------------------------------
//ColorMap functions
void EntropyFilter::colorMapEntropy(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_map)
{
    cloud_map->width = m_mls_cloud->width;
    cloud_map->height = m_mls_cloud->height;
    cloud_map->resize(m_mls_cloud->width * m_mls_cloud->height);

    for (int i = 0; i < m_mls_cloud->size(); i++)
    {
        cloud_map->points[i].x = m_mls_cloud->points[i].x;
        cloud_map->points[i].y = m_mls_cloud->points[i].y;
        cloud_map->points[i].z = m_mls_cloud->points[i].z;
        cloud_map->points[i].intensity = m_spherical->points[i].entropy;
    }
}

void EntropyFilter::colorMapCurvature(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_map)
{
    cloud_map->width = m_mls_cloud->width;
    cloud_map->height = m_mls_cloud->height;
    cloud_map->resize(m_mls_cloud->width * m_mls_cloud->height);

    for (int i = 0; i < m_mls_cloud->size(); i++)
    {
        cloud_map->points[i].x = m_mls_cloud->points[i].x;
        cloud_map->points[i].y = m_mls_cloud->points[i].y;
        cloud_map->points[i].z = m_mls_cloud->points[i].z;
        cloud_map->points[i].intensity = m_mls_normals->points[i].curvature;
    }
}

void EntropyFilter::colorMapInclination(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_map)
{
    cloud_map->width = m_mls_cloud->width;
    cloud_map->height = m_mls_cloud->height;
    cloud_map->resize(m_mls_cloud->width * m_mls_cloud->height);

    for (int i = 0; i < m_mls_cloud->size(); i++)
    {
        cloud_map->points[i].x = m_mls_cloud->points[i].x;
        cloud_map->points[i].y = m_mls_cloud->points[i].y;
        cloud_map->points[i].z = m_mls_cloud->points[i].z;
        cloud_map->points[i].intensity = m_spherical->points[i].inclination;
    }
}

void EntropyFilter::colorMapAzimuth(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_map)
{
    cloud_map->width = m_mls_cloud->width;
    cloud_map->height = m_mls_cloud->height;
    cloud_map->resize(m_mls_cloud->width * m_mls_cloud->height);

    for (int i = 0; i < m_mls_cloud->size(); i++)
    {
        cloud_map->points[i].x = m_mls_cloud->points[i].x;
        cloud_map->points[i].y = m_mls_cloud->points[i].y;
        cloud_map->points[i].z = m_mls_cloud->points[i].z;
        cloud_map->points[i].intensity = m_spherical->points[i].azimuth;
    }
}

void EntropyFilter::visualizeAll(bool flag)
{

    pcl::visualization::PCLVisualizer vizSource("PCL Source Cloud");
    vizSource.setBackgroundColor(1.0f, 1.0f, 1.0f);
    vizSource.addPointCloud<pcl::PointXYZRGB>(m_source, "m_source");
    vizSource.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "m_source");

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_curvature(new pcl::PointCloud<pcl::PointXYZI>);
    colorMapCurvature(cloud_curvature);
    pcl::visualization::PCLVisualizer vizC("PCL Curvature Map");
    vizC.setBackgroundColor(1.0f, 1.0f, 1.0f);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distributionCurvature(cloud_curvature, "intensity");
    vizC.addPointCloud<pcl::PointXYZI>(cloud_curvature, intensity_distributionCurvature, "cloud_mapCurvature");
    vizC.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_mapCurvature");

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_entropy(new pcl::PointCloud<pcl::PointXYZI>);
    colorMapEntropy(cloud_entropy);
    pcl::visualization::PCLVisualizer vizE("PCL Entropy Map");
    vizE.setBackgroundColor(1.0f, 1.0f, 1.0f);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distributionEntropy(cloud_entropy, "intensity");
    vizE.addPointCloud<pcl::PointXYZI>(cloud_entropy, intensity_distributionEntropy, "cloud_mapEntropy");
    vizE.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_mapEntropy");

    if (flag)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_inclination(new pcl::PointCloud<pcl::PointXYZI>);
        colorMapInclination(cloud_inclination);
        pcl::visualization::PCLVisualizer vizI("PCL Inclination Map");
        vizI.setBackgroundColor(1.0f, 1.0f, 1.0f);
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distributionInclination(cloud_inclination, "intensity");
        vizI.addPointCloud<pcl::PointXYZI>(cloud_inclination, intensity_distributionInclination, "sample cloud_mapInclination");

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_azimuth(new pcl::PointCloud<pcl::PointXYZI>);
        colorMapAzimuth(cloud_azimuth);
        pcl::visualization::PCLVisualizer vizA("PCL Azimuth Map");
        vizA.setBackgroundColor(1.0f, 1.0f, 1.0f);
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distributionAzimuth(cloud_azimuth, "intensity");
        vizA.addPointCloud<pcl::PointXYZI>(cloud_azimuth, intensity_distributionAzimuth, "sample cloud_mapAzimuth");
    }

    pcl::visualization::PCLVisualizer vizDepth("PCL Depth Map");
    vizDepth.setBackgroundColor(1.0f, 1.0f, 1.0f);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distributionDepth(m_cloud_depth, "intensity");
    vizDepth.addPointCloud<pcl::PointXYZI>(m_cloud_depth, intensity_distributionDepth, "cloud_depth");
    vizDepth.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_depth");
    //viz2.addPlane(*coefficients, "plane");

    pcl::visualization::PCLVisualizer vizConvexity("PCL Convexity Map");
    vizConvexity.setBackgroundColor(1.0f, 1.0f, 1.0f);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(m_cloud_convexity, "intensity");
    vizConvexity.addPointCloud<pcl::PointXYZI>(m_cloud_convexity, intensity_distribution, "cloud_convexity");
    vizConvexity.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_convexity");
    //viz.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_small, normals_small, 1, 0.01);

    pcl::visualization::PCLVisualizer vizCombined("PCL Combined Map");
    vizCombined.setBackgroundColor(1.0f, 1.0f, 1.0f);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distributionCombinede(m_cloud_combined, "intensity");
    vizCombined.addPointCloud<pcl::PointXYZI>(m_cloud_combined, intensity_distributionCombinede, "m_cloud_combined");
    vizCombined.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "m_cloud_combined");
    //viz.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_small, normals_small, 1, 0.01);

    while (!vizC.wasStopped())
    {
        vizC.spinOnce();
    }
}
// -----------------------------------------------------------------------------------------------------------------------

void EntropyFilter::downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, float leaf_size,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
    pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
    vox_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    vox_grid.setInputCloud(cloud_in);
    vox_grid.filter(*cloud_out);
}

void EntropyFilter::computePolyFitting(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointNormal> &mls_points)
{
    // Output has the PointNormal type in order to store the normals calculated by MLS

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    // Set parameters
    mls.setNumberOfThreads(4);
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);

    // Reconstruct
    mls.process(mls_points);
    std::cout << "mls_points: " << mls_points.height << ", " << mls_points.width << std::endl;
}

void EntropyFilter::divideCloudNormals(pcl::PointCloud<pcl::PointNormal> &input, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                       pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
    cloud->height = input.height;
    cloud->width = input.width;
    cloud->resize(input.size());
    normals->height = input.height;
    normals->width = input.width;
    normals->resize(input.size());
    for (int i = 0; i < input.size(); i++)
    {
        cloud->points[i].x = input.points[i].x;
        cloud->points[i].y = input.points[i].y;
        cloud->points[i].z = input.points[i].z;

        normals->points[i].normal_x = input.points[i].normal_x;
        normals->points[i].normal_y = input.points[i].normal_y;
        normals->points[i].normal_z = input.points[i].normal_z;
        normals->points[i].curvature = input.points[i].curvature;
    }
}

void EntropyFilter::getSpherical(pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals, pcl::PointCloud<Spherical>::Ptr &spherical)
{
    spherical->width = cloud_normals->width;
    spherical->height = cloud_normals->height;
    spherical->resize(spherical->width * spherical->height);

    pcl::Normal data;
    for (size_t i = 0; i < cloud_normals->points.size(); ++i)
    {
        data = cloud_normals->points[i];
        spherical->points[i].azimuth = atan2(data.normal_z, data.normal_y);
        spherical->points[i].inclination = atan2(sqrt(data.normal_y * data.normal_y + data.normal_z * data.normal_z), data.normal_x);
    }
}

void EntropyFilter::normalizeEntropy(pcl::PointCloud<Spherical>::Ptr &spherical)
{

    float max_entropy = 0;
    for (int i = 0; i < spherical->size(); i++)
    {
        if (spherical->points[i].entropy > max_entropy)
            max_entropy = spherical->points[i].entropy;
    }

    for (int i = 0; i < spherical->size(); i++)
    {
        spherical->points[i].entropy_normalized = spherical->points[i].entropy / max_entropy;
    }
    std::cout << "max entropy : " << max_entropy << std::endl;
    _max_entropy = max_entropy;
}

//LOCAL HISTOGRAM and entropy calculation at the end.
//param[in]: point cloud normals in spherical coordinates
//param[in]: current point index in the cloud
//param[in]: vector of indeces of neighborhood points of considered on
void EntropyFilter::histogram2D(pcl::PointCloud<Spherical>::Ptr &spherical, int id0, std::vector<int> indices)
{
    int Hist[64][64] = {0};
    float step_inc = M_PI / 63;
    float step_az = (2 * M_PI) / 63;
    long bin_inc, bin_az;
    for (int i = 0; i < indices.size(); i++)
    {
        bin_inc = std::lroundf(spherical->points[indices[i]].inclination / step_inc);
        bin_az = std::lroundf((spherical->points[indices[i]].azimuth + M_PI) / step_az);
        if (bin_az < 0)
            std::cout << "erorr bin_az negative" << std::endl;

        Hist[bin_inc][bin_az]++;
    }
    bin_inc = std::lroundf(spherical->points[id0].inclination / step_inc);
    bin_az = std::lroundf((spherical->points[id0].azimuth + M_PI) / step_az);
    if (bin_az < 0)
        std::cout << "erorr bin_az negative" << std::endl;

    Hist[bin_inc][bin_az]++;

    float HistNorm[64][64] = {0};
    float max_value = 0;
    for (int i = 0; i < 64; i++)
    {
        for (int j = 0; j < 64; j++)
        {
            if (Hist[i][j] > max_value)
                max_value = Hist[i][j];
        }
    }

    for (int i = 0; i < 64; i++)
        for (int j = 0; j < 64; j++)
            HistNorm[i][j] = Hist[i][j] / max_value;

    //entropy calculation
    float entropy_value = 0;
    float temp_entropy = 0;
    for (int i = 0; i < 64; i++)
    {
        for (int j = 0; j < 64; j++)
        {
            temp_entropy = HistNorm[i][j] * log2(HistNorm[i][j]);
            if (!std::isnan(temp_entropy))
                entropy_value += temp_entropy;
        }
    }

    spherical->points[id0].entropy = -(entropy_value);
    //std::cout << "entropy value: " << spherical->points[id0].entropy << std::endl;
}

// LOCAL SEARCH
void EntropyFilter::local_search(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<Spherical>::Ptr &spherical,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_combined,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_depth)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    pcl::PointXYZ searchPoint;

    for (int it = 0; it < cloud->points.size(); it++)
    {

        if (cloud_combined->points[it].intensity > 0)
        {

            searchPoint.x = cloud->points[it].x;
            searchPoint.y = cloud->points[it].y;
            searchPoint.z = cloud->points[it].z;

            // K nearest neighbor search
            std::vector<int> pointIdxNKNSearch(m_KNN);
            std::vector<float> pointNKNSquaredDistance(m_KNN);

            if (kdtree.nearestKSearch(searchPoint, m_KNN, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
            {
                histogram2D(spherical, it, pointIdxNKNSearch);
            }
            if (_flag_weight)
                spherical->points[it].entropy = -spherical->points[it].entropy * cloud_depth->points[it].intensity * 100;
        }
        else
        {
            spherical->points[it].entropy = 0;
        }
    }
}

void EntropyFilter::segmentCloudEntropy(pcl::PointCloud<pcl::PointNormal> &cloud, pcl::PointCloud<Spherical>::Ptr &spherical,
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr &output, float thresholdEntropy)
{
    pcl::PointXYZI p;
    for (int i = 0; i < spherical->size(); i++)
    {
        if (spherical->points[i].entropy_normalized > thresholdEntropy)
        {
            p.x = cloud.points[i].x;
            p.y = cloud.points[i].y;
            p.z = cloud.points[i].z;
            output->points.push_back(p);
        }
    }
    std::cout << "cloud segmented size: " << output->size() << std::endl;
}

void EntropyFilter::connectedComponets(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                       std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_clusters)
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
        cloud_clusters.push_back(cloud_cluster);
    }

    std::cout << "number of clusters found: " << cluster_indices.size() << std::endl;
}

float EntropyFilter::getPointPlaneDistanceCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out,
                                                pcl::ModelCoefficients::Ptr &plane)
{
    Eigen::Vector3f x0;
    Eigen::Vector3f n(plane->values[0], plane->values[1], plane->values[2]);
    float p = plane->values[3];
    float distance;

    cloud_out->width = cloud->width;
    cloud_out->height = cloud->height;
    cloud_out->resize(cloud_out->width * cloud_out->height);

    std::vector<float> v_distances;
    for (int i = 0; i < cloud->size(); i++)
    {
        // D=n·x_0+p

        if ((!std::isnan(cloud->points[i].x)) && (!std::isnan(cloud->points[i].y)) && (!std::isnan(cloud->points[i].z)))
        {

            x0 = cloud->points[i].getVector3fMap();
            distance = n.dot(x0) + p;

            if (distance > 0)
                distance = 0;

            cloud_out->points[i].x = cloud->points[i].x;
            cloud_out->points[i].y = cloud->points[i].y;
            cloud_out->points[i].z = cloud->points[i].z;

            cloud_out->points[i].intensity = distance;
            v_distances.push_back(distance);
        }
    }

    //float max_depth = *std::max_element(v_distances.begin(), v_distances.end());
    float min_depth = *std::min_element(v_distances.begin(), v_distances.end());

    float depth_interval = fabs(min_depth);
    //if (min_depth < 0)
    //    depth_interval = max_depth - min_depth;
    //else
    //    depth_interval = max_depth + min_depth;

    //std::cout << "max and min depth values: " << max_depth << ", " << min_depth << std::endl;
    std::cout << "depth interval: " << depth_interval << std::endl;
    return depth_interval;
}

void EntropyFilter::downsampleCloudAndNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                              float leaf_size, pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_out)
{

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    pcl::VoxelGrid<pcl::PointNormal> vox_grid;
    vox_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    vox_grid.setInputCloud(cloud_with_normals);
    vox_grid.filter(*cloud_out);
}

bool EntropyFilter::checkConvexity(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                   int id0, std::vector<int> ids)
{
    Eigen::Vector3f x1 = cloud->points[id0].getVector3fMap();
    Eigen::Vector3f n1 = -1 * normals->points[id0].getNormalVector3fMap();

    Eigen::Vector3f d;

    float result;
    int convex = 0, concave = 0;
    for (int i = 0; i < ids.size(); i++)
    {
        Eigen::Vector3f x2 = cloud->points[ids[i]].getVector3fMap();
        Eigen::Vector3f n2 = -1 * normals->points[ids[i]].getNormalVector3fMap();
        d = x1 - x2;

        float angle = acos(n1.dot(n2)) * (180 / M_PI);
        if (angle > m_angle_threshold || angle < -m_angle_threshold) // if normals are similar we do not check convexity/concavity
        {
            //std::cout << "angle " << angle << std::endl;

            result = n1.dot(d) - n2.dot(d);
            if (result > 0)
                convex++;
            else
                concave++;
        }
    }

    //std::cout << "convex: " << convex << "; concave: " << concave << std::endl;

    if (convex > concave)
        return true;
    else
        return false;
}

void EntropyFilter::splitPointNormal(pcl::PointCloud<pcl::PointNormal>::Ptr &input, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                     pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
    cloud->height = input->height;
    cloud->width = input->width;
    cloud->resize(input->size());
    normals->height = input->height;
    normals->width = input->width;
    normals->resize(input->size());
    for (int i = 0; i < input->size(); i++)
    {
        cloud->points[i].x = input->points[i].x;
        cloud->points[i].y = input->points[i].y;
        cloud->points[i].z = input->points[i].z;

        normals->points[i].normal_x = input->points[i].normal_x;
        normals->points[i].normal_y = input->points[i].normal_y;
        normals->points[i].normal_z = input->points[i].normal_z;
        normals->points[i].curvature = input->points[i].curvature;
    }
}

void EntropyFilter::localSearchForConvexity(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_map)
{

    /*
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    splitPointNormal(cloud_ready, cloud, normals);
    */
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    pcl::PointXYZ searchPoint;
    int K = 8;

    cloud_map->width = cloud->width;
    cloud_map->height = cloud->height;
    cloud_map->resize(cloud->width * cloud->height);

    for (size_t it = 0; it < cloud->points.size(); it++)
    {
        searchPoint.x = cloud->points[it].x;
        searchPoint.y = cloud->points[it].y;
        searchPoint.z = cloud->points[it].z;

        // K nearest neighbor search
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            if (checkConvexity(cloud, normals, it, pointIdxNKNSearch))
            {
                cloud_map->points[it].intensity = 1.0;
            }
            else
            {
                cloud_map->points[it].intensity = 0.0;
            }

            cloud_map->points[it].x = cloud->points[it].x;
            cloud_map->points[it].y = cloud->points[it].y;
            cloud_map->points[it].z = cloud->points[it].z;
        }
    }
}

void EntropyFilter::combineConvexityAndCurvatureInfo(pcl::PointCloud<pcl::PointXYZI>::Ptr &convexity,
                                                     pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                                     pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_map)
{
    cloud_map->width = convexity->width;
    cloud_map->height = convexity->height;
    cloud_map->resize(convexity->width * convexity->height);

    std::vector<bool> result;
    result.resize(normals->size());
    for (int i = 0; i < normals->size(); i++)
    {
        if (normals->points[i].curvature >= m_curvature_threshold && convexity->points[i].intensity == 1)
        {
            cloud_map->points[i].intensity = 1;
        }
        else
        {
            cloud_map->points[i].intensity = 0;
        }

        cloud_map->points[i].x = convexity->points[i].x;
        cloud_map->points[i].y = convexity->points[i].y;
        cloud_map->points[i].z = convexity->points[i].z;
    }
}

void EntropyFilter::computePlaneReference(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::ModelCoefficients::Ptr &plane)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *plane);
}

void EntropyFilter::computePlaneBottomBin(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr &vertices,
                                          pcl::ModelCoefficients::Ptr &plane)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(vertices);
    seg.segment(*inliers, *plane);

    Eigen::Vector3f x0;
    Eigen::Vector3f n(plane->values[0], plane->values[1], plane->values[2]);
    float p = plane->values[3];
    float distance, distance_max = 0;

    for (int i = 0; i < cloud->size(); i++)
    {
        // D=n·x_0+p

        if ((!std::isnan(cloud->points[i].x)) && (!std::isnan(cloud->points[i].y)) && (!std::isnan(cloud->points[i].z)))
        {

            x0 = cloud->points[i].getVector3fMap();
            distance = n.dot(x0) + p;

            if (distance > distance_max)
                distance_max = distance;
        }
    }

    Eigen::Vector3f bottom;
    pcl::PointXYZ point;
    pcl::PointCloud<pcl::PointXYZ>::Ptr bottom_vertices(new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << vertices->size() << std::endl;
    for (int i = 0; i < vertices->size(); i++)
    {
        bottom = vertices->points[i].getVector3fMap() + distance_max * n;

        point.getVector3fMap() = bottom;
        bottom_vertices->points.push_back(point);
    }

    std::cout << "distance max between top vertices and points in the bin: " << distance_max << std::endl;
    //std::cout << "new bottom_vertices size: " << bottom_vertices->size() << std::endl;

    // estimation new ground plane
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    inliers.reset(new pcl::PointIndices);
    plane.reset(new pcl::ModelCoefficients);

    seg.setInputCloud(bottom_vertices);
    seg.segment(*inliers, *plane);
}

void EntropyFilter::setReferencePlane(pcl::ModelCoefficients::Ptr &plane)
{
    m_plane_ref = plane;
    _flag_plane_ref = true;
}

void EntropyFilter::alternativeConnectedComponets(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                                  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_clusters)
{

    for (int j = 0; j < cloud->size(); j++)
    {
        float max = 0;
        int it;
        for (int i = 0; i < cloud->size(); i++)
        {
            if (cloud->points[i].intensity > max)
            {
                max = cloud->points[i].intensity;
                it = i;
            }
        }
        //std::cout << "max entropy: " << max << ", at indices: " << it << std::endl;

        pcl::PointXYZI searchPoint = cloud->points[it];

        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloud);
        pcl::Indices indices;
        std::vector<float> sqrDistances;
        float radius = 0.05;

        if (tree->radiusSearch(searchPoint, radius, indices, sqrDistances) > 0)
        {

            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            inliers->indices = indices;
            //std::cout << "number of neighborhood: " << indices.size() << std::endl;

            pcl::ExtractIndices<pcl::PointXYZI> extract;

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp2(new pcl::PointCloud<pcl::PointXYZ>);

            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cloud_temp);

            pcl::copyPointCloud(*cloud_temp, *cloud_temp2);
            if (inliers->indices.size() > 30) // 20
            {
                cloud_clusters.push_back(cloud_temp2);
                std::cout << "cluster number " << j << ", size: " << cloud_temp2->size() << std::endl;
            }
            else
            {
                std::cout << "cluster number " << j << ", size: " << cloud_temp2->size() << " -> to small! " << std::endl;
            }

            // remove points extracted above
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*cloud);
        }
    }

    std::cout << "number of clusters: " << cloud_clusters.size() << std::endl;
}

void EntropyFilter::optimizeNumberOfClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_clusters,
                                           std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds_out)
{

    std::vector<pcl::PointXYZ> results(cloud_clusters.size());
    for (int i = 0; i < cloud_clusters.size(); i++)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_clusters[i], centroid);
        results[i].getVector4fMap() = centroid;
    }

    std::vector<int> indices(results.size());
    for (int i = 0; i < results.size(); i++)
    {
        for (int j = 0; j < results.size(); j++)
        {
            if (i == j)
                continue;

            if (pcl::geometry::distance(results[i], results[j]) < 0.05)
            {
                indices[i] = j;
                break;
            }
        }
    }

    for (int i = 0; i < indices.size(); i++)
    {
        int id = indices[i];
        if (id > 0)
        {
            for (int k = 0; k < cloud_clusters[indices[i]]->size(); k++)
                cloud_clusters[i]->push_back(cloud_clusters[indices[i]]->points[k]);

            indices[id] = -1;
        }
        if (id < 0)
            continue;

        clouds_out.push_back(cloud_clusters[i]);
    }

    std::cout << std::endl;
    std::cout << "new clusters after optimization: " << std::endl;
    for (int k = 0; k < clouds_out.size(); k++)
        std::cout
            << "new cluster " << k << ", size: " << clouds_out[k]->size() << std::endl;
}
