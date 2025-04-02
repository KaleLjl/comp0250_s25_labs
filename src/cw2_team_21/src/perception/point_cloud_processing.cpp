#include "perception/point_cloud_processing.h"

namespace cw2 {

PointCloudProcessing::PointCloudProcessing() 
  : g_cloud_ptr(new PointC),
    g_cloud_filtered(new PointC),
    g_cloud_filtered2(new PointC),
    g_cloud_filtered_octomap(new PointC),
    g_octomap_ptr(new pcl::PointCloud<pcl::PointXYZ>),
    g_octomap_filtered(new pcl::PointCloud<pcl::PointXYZ>)
{
  // Default initialization
  g_vg_leaf_sz = 0.01;
  g_pt_thrs_min = 0.0;
  g_pt_thrs_max = 0.5;
  g_k_nn = 50;
}

void PointCloudProcessing::initialize(double vg_leaf_sz, 
                                    double pt_thrs_min, 
                                    double pt_thrs_max, 
                                    int k_nn) {
  g_vg_leaf_sz = vg_leaf_sz;
  g_pt_thrs_min = pt_thrs_min;
  g_pt_thrs_max = pt_thrs_max;
  g_k_nn = k_nn;
}

void PointCloudProcessing::processPointCloud(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg) {
  // Extract input point cloud info
  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;
    
  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2(g_pcl_pc, *g_cloud_ptr);
}

void PointCloudProcessing::setupNoughtFilter() {
  nought_filter.setMin(Eigen::Vector4f(-0.13, 0.05, 0.45, 1.0));
  nought_filter.setMax(Eigen::Vector4f(0.13, 0.4, 0.51, 1.0));
  nought_filter.setInputCloud(g_cloud_ptr);
  nought_filter.filter(*g_cloud_filtered);
}

void PointCloudProcessing::setupCrossFilter() {
  cross_filter.setInputCloud(g_cloud_ptr);
  cross_filter.setFilterFieldName("z");
  cross_filter.setFilterLimits(0.45, 0.51);
  cross_filter.filter(*g_cloud_filtered2);
}

void PointCloudProcessing::applyFilter(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr) {
  g_pt.setInputCloud(in_cloud_ptr);
  
  // Enlarging the x limits to include the whole plane
  g_pt.setFilterFieldName("x");
  g_pt.setFilterLimits(-1.0, 1.0);

  // Restricting the z limits to the top of the cubes
  g_pt.setFilterFieldName("z");
  g_pt.setFilterLimits(g_pt_thrs_min, g_pt_thrs_max);
  
  g_pt.filter(*out_cloud_ptr);
}

void PointCloudProcessing::pubFilteredPCMsg(ros::Publisher &pc_pub, PointC &pc) {
  // Publish the data
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  pc_pub.publish(g_cloud_filtered_msg);
}

void PointCloudProcessing::processOctomapPointCloud(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg) {
  // Extract octomap point cloud frame id
  g_octomap_frame_id_ = cloud_input_msg->header.frame_id;
  
  // Convert ROS message to PCL point cloud
  pcl_conversions::toPCL(*cloud_input_msg, g_octomap_pc);
  pcl::fromPCLPointCloud2(g_octomap_pc, *g_octomap_ptr);

  // Apply filtering to the octomap point cloud
  g_octomap_pt.setInputCloud(g_octomap_ptr);
  g_octomap_pt.setFilterFieldName("z");
  g_octomap_pt.setFilterLimits(0.04, 0.5);
  g_octomap_pt.filter(*g_octomap_filtered);
  
  ROS_INFO("Filtered octomap cloud has %lu points", g_octomap_filtered->size());
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> 
PointCloudProcessing::clusterPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  // Vector to store the clusters
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  
  // Check if cloud is empty
  if (cloud->empty()) {
    ROS_ERROR("Cannot cluster an empty point cloud!");
    return clusters;
  }

  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  // Create a set of indices to be used in the extraction
  std::vector<pcl::PointIndices> cluster_indices;
  // Create the extraction object for the clusters
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extraction;
  // Set the extraction parameters
  cluster_extraction.setClusterTolerance(0.04); // 4cm
  cluster_extraction.setMinClusterSize(50);  
  cluster_extraction.setMaxClusterSize(10000);
  cluster_extraction.setSearchMethod(tree);
  cluster_extraction.setInputCloud(cloud);
  cluster_extraction.extract(cluster_indices);

  // Loop through each cluster and store it in the vector
  for (const auto& cluster : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : cluster.indices) {
      cloud_cluster->push_back((*cloud)[idx]);
    }
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    ROS_INFO("PointCloud representing the Cluster: %lu data points.", cloud_cluster->size());
    clusters.push_back(cloud_cluster);
  }

  return clusters;
}

double PointCloudProcessing::getClusterWidth(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster) {
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(cluster);
  feature_extractor.compute();

  // Camera space bounding box
  pcl::PointXYZ min_point_AABB;
  pcl::PointXYZ max_point_AABB;
  
  feature_extractor.getAABB(min_point_AABB, max_point_AABB);
  return max_point_AABB.x - min_point_AABB.x; 
}

} // namespace cw2
