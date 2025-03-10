#ifndef POINT_CLOUD_PROCESSING_H
#define POINT_CLOUD_PROCESSING_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <string>
#include <vector>

// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>

// TF
#include <tf/transform_listener.h>

// Typedefs
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

namespace cw1_pcl
{
// --- Filtering functions ---
// Downsample point cloud using VoxelGrid filter.
void applyVoxelGrid(PointCPtr &in_cloud, PointCPtr &out_cloud, double leaf_size);

// PassThrough filter on x-axis.
void applyPassThroughX(PointCPtr &in_cloud, PointCPtr &out_cloud, double min_x, double max_x);

// PassThrough filter on y-axis.
void applyPassThroughY(PointCPtr &in_cloud, PointCPtr &out_cloud, double min_y, double max_y);

// --- Normal estimation ---
// Estimate surface normals with k nearest neighbors.
void findNormals(PointCPtr &in_cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, int k);

// --- Segmentation functions ---
// Segment a plane from the input cloud.
// Outputs: inliers, model coefficients, plane cloud and remaining cloud & normals.
void segmentPlane(PointCPtr &in_cloud,
                  pcl::PointIndices::Ptr &inliers,
                  pcl::ModelCoefficients::Ptr &coefficients,
                  PointCPtr &plane_cloud,
                  PointCPtr &remaining_cloud,
                  pcl::PointCloud<pcl::Normal>::Ptr &normals,
                  pcl::PointCloud<pcl::Normal>::Ptr &remaining_normals,
                  double distance_threshold,
                  double normal_distance_weight,
                  int max_iterations);

// Segment a cylinder (or parallel plane) from the input cloud.
void segmentCylinder(PointCPtr &in_cloud,
                     pcl::PointCloud<pcl::Normal>::Ptr &normals,
                     pcl::PointIndices::Ptr &inliers,
                     pcl::ModelCoefficients::Ptr &coefficients,
                     PointCPtr &cylinder_cloud,
                     double distance_threshold,
                     double normal_distance_weight,
                     int max_iterations);

// --- Pose estimation ---
// Compute the centroid of the cloud and transform it to target_frame.
geometry_msgs::Point findCylPose(PointCPtr &cloud, const std::string &target_frame, tf::TransformListener &listener);

// --- Clustering ---
// Cluster the input cloud and return cluster indices.
void clusterPointCloud(PointCPtr &in_cloud,
                       std::vector<pcl::PointIndices> &cluster_indices,
                       double cluster_tolerance,
                       int min_cluster_size,
                       int max_cluster_size);

// --- Publish functions ---
// Publish a PointCloud2 message.
void publishFilteredPointCloud(ros::Publisher &pub, PointC &cloud);

// Publish a point as PointStamped.
void publishPose(ros::Publisher &pub, const geometry_msgs::Point &point);

// --- Callback and utility functions ---
// Extract point from PointStamped message.
void targetPoseCallback(const geometry_msgs::PointStamped &msg, geometry_msgs::Point &target_point);

// Remove duplicate positions based on a threshold.
std::vector<geometry_msgs::Point> discardRepeatPositions(const std::vector<geometry_msgs::Point> &positions, double threshold);

// Print cube and basket positions for debugging.
void printCubeBasket(const std::vector<geometry_msgs::Point> &red_cube,
                     const std::vector<geometry_msgs::Point> &blue_cube,
                     const std::vector<geometry_msgs::Point> &purple_cube,
                     const std::vector<geometry_msgs::Point> &red_basket,
                     const std::vector<geometry_msgs::Point> &blue_basket,
                     const std::vector<geometry_msgs::Point> &purple_basket);
} // namespace cw1_pcl
#endif // POINT_CLOUD_PROCESSING_H
