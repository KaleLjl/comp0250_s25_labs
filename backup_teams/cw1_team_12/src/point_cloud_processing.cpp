#include <point_cloud_processing.h>
#include <pcl_conversions/pcl_conversions.h>

namespace cw1_pcl
{
// -------------------- Filtering functions --------------------
void applyVoxelGrid(PointCPtr &in_cloud, PointCPtr &out_cloud, double leaf_size)
{
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(in_cloud);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*out_cloud);
}

void applyPassThroughX(PointCPtr &in_cloud, PointCPtr &out_cloud, double min_x, double max_x)
{
    pcl::PassThrough<PointT> pt;
    pt.setInputCloud(in_cloud);
    pt.setFilterFieldName("x");
    pt.setFilterLimits(min_x, max_x);
    pt.filter(*out_cloud);
}

void applyPassThroughY(PointCPtr &in_cloud, PointCPtr &out_cloud, double min_y, double max_y)
{
    pcl::PassThrough<PointT> pt;
    pt.setInputCloud(in_cloud);
    pt.setFilterFieldName("y");
    pt.setFilterLimits(min_y, max_y);
    pt.filter(*out_cloud);
}

// -------------------- Normal estimation --------------------
void findNormals(PointCPtr &in_cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, int k)
{
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(in_cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    ne.setSearchMethod(tree);
    ne.setKSearch(k);
    ne.compute(*normals);
}

// -------------------- Segmentation functions --------------------
void segmentPlane(PointCPtr &in_cloud,
                  pcl::PointIndices::Ptr &inliers,
                  pcl::ModelCoefficients::Ptr &coefficients,
                  PointCPtr &plane_cloud,
                  PointCPtr &remaining_cloud,
                  pcl::PointCloud<pcl::Normal>::Ptr &normals,
                  pcl::PointCloud<pcl::Normal>::Ptr &remaining_normals,
                  double distance_threshold,
                  double normal_distance_weight,
                  int max_iterations)
{
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(normal_distance_weight);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iterations);
    seg.setDistanceThreshold(distance_threshold);
    seg.setInputCloud(in_cloud);
    seg.setInputNormals(normals);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(in_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane_cloud);

    extract.setNegative(true);
    extract.filter(*remaining_cloud);

    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setInputCloud(normals);
    extract_normals.setIndices(inliers);
    extract_normals.setNegative(true);
    extract_normals.filter(*remaining_normals);
}

void segmentCylinder(PointCPtr &in_cloud,
                     pcl::PointCloud<pcl::Normal>::Ptr &normals,
                     pcl::PointIndices::Ptr &inliers,
                     pcl::ModelCoefficients::Ptr &coefficients,
                     PointCPtr &cylinder_cloud,
                     double distance_threshold,
                     double normal_distance_weight,
                     int max_iterations)
{
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    seg.setNormalDistanceWeight(normal_distance_weight);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iterations);
    seg.setDistanceThreshold(distance_threshold);
    seg.setInputCloud(in_cloud);
    seg.setInputNormals(normals);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(in_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cylinder_cloud);
}

// -------------------- Pose estimation --------------------
geometry_msgs::Point findCylPose(PointCPtr &cloud, const std::string &target_frame, tf::TransformListener &listener)
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    geometry_msgs::PointStamped centroid_msg;
    // Use cloud header if set; otherwise, default frame.
    centroid_msg.header.frame_id = cloud->header.frame_id.empty() ? "panda_link0" : cloud->header.frame_id;
    centroid_msg.point.x = centroid[0];
    centroid_msg.point.y = centroid[1];
    centroid_msg.point.z = centroid[2];

    geometry_msgs::PointStamped transformed;
    try {
        listener.transformPoint(target_frame, centroid_msg, transformed);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("Transform error: %s", ex.what());
        transformed = centroid_msg;
    }
    return transformed.point;
}

// -------------------- Clustering --------------------
void clusterPointCloud(PointCPtr &in_cloud,
                       std::vector<pcl::PointIndices> &cluster_indices,
                       double cluster_tolerance,
                       int min_cluster_size,
                       int max_cluster_size)
{
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(in_cloud);

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(in_cloud);
    ec.extract(cluster_indices);
}

// -------------------- Publish functions --------------------
void publishFilteredPointCloud(ros::Publisher &pub, PointC &cloud)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    pub.publish(output);
}

void publishPose(ros::Publisher &pub, const geometry_msgs::Point &point)
{
    geometry_msgs::PointStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.point = point;
    pub.publish(msg);
}

// -------------------- Callback and utility functions --------------------
void targetPoseCallback(const geometry_msgs::PointStamped &msg, geometry_msgs::Point &target_point)
{
    target_point = msg.point;
}

std::vector<geometry_msgs::Point> discardRepeatPositions(const std::vector<geometry_msgs::Point> &positions, double threshold)
{
    std::vector<geometry_msgs::Point> unique_positions;
    for (size_t i = 0; i < positions.size(); i++) {
        bool duplicate = false;
        for (size_t j = 0; j < unique_positions.size(); j++) {
            if (std::abs(positions[i].x - unique_positions[j].x) < threshold &&
                std::abs(positions[i].y - unique_positions[j].y) < threshold) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate)
            unique_positions.push_back(positions[i]);
    }
    return unique_positions;
}

void printCubeBasket(const std::vector<geometry_msgs::Point> &red_cube,
                     const std::vector<geometry_msgs::Point> &blue_cube,
                     const std::vector<geometry_msgs::Point> &purple_cube,
                     const std::vector<geometry_msgs::Point> &red_basket,
                     const std::vector<geometry_msgs::Point> &blue_basket,
                     const std::vector<geometry_msgs::Point> &purple_basket)
{
    ROS_INFO("Red Cube Positions:");
    for (size_t i = 0; i < red_cube.size(); i++)
        ROS_INFO("  (%f, %f, %f)", red_cube[i].x, red_cube[i].y, red_cube[i].z);

    ROS_INFO("Blue Cube Positions:");
    for (size_t i = 0; i < blue_cube.size(); i++)
        ROS_INFO("  (%f, %f, %f)", blue_cube[i].x, blue_cube[i].y, blue_cube[i].z);

    ROS_INFO("Purple Cube Positions:");
    for (size_t i = 0; i < purple_cube.size(); i++)
        ROS_INFO("  (%f, %f, %f)", purple_cube[i].x, purple_cube[i].y, purple_cube[i].z);

    ROS_INFO("Red Basket Positions:");
    for (size_t i = 0; i < red_basket.size(); i++)
        ROS_INFO("  (%f, %f, %f)", red_basket[i].x, red_basket[i].y, red_basket[i].z);

    ROS_INFO("Blue Basket Positions:");
    for (size_t i = 0; i < blue_basket.size(); i++)
        ROS_INFO("  (%f, %f, %f)", blue_basket[i].x, blue_basket[i].y, blue_basket[i].z);

    ROS_INFO("Purple Basket Positions:");
    for (size_t i = 0; i < purple_basket.size(); i++)
        ROS_INFO("  (%f, %f, %f)", purple_basket[i].x, purple_basket[i].y, purple_basket[i].z);
}
}