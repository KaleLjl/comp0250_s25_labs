#ifndef POINT_CLOUD_PROCESSING_H_
#define POINT_CLOUD_PROCESSING_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/moment_of_inertia_estimation.h>

namespace cw2 {

// Set default pointcloud types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

class PointCloudProcessing {
public:
  /**
   * @brief Constructor for the PointCloudProcessing class
   */
  PointCloudProcessing();

  /**
   * @brief Initialize the point cloud processors with parameters
   * @param vg_leaf_sz Voxel grid leaf size
   * @param pt_thrs_min Pass through min threshold
   * @param pt_thrs_max Pass through max threshold
   * @param k_nn Normals neighborhood size
   */
  void initialize(double vg_leaf_sz = 0.01, 
                  double pt_thrs_min = 0.0, 
                  double pt_thrs_max = 0.5, 
                  int k_nn = 50);

  /**
   * @brief Process incoming point cloud message
   * @param cloud_input_msg Input point cloud message
   */
  void processPointCloud(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg);

  /**
   * @brief Apply pass through filter to the input point cloud
   * @param in_cloud_ptr Input point cloud
   * @param out_cloud_ptr Output filtered point cloud
   */
  void applyFilter(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);

  /**
   * @brief Publish filtered point cloud
   * @param pc_pub ROS publisher
   * @param pc Point cloud to publish
   */
  void pubFilteredPCMsg(ros::Publisher &pc_pub, PointC &pc);

  /**
   * @brief Cluster point clouds into separate objects
   * @param cloud Input point cloud
   * @return Vector of clustered point clouds
   */
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterPointClouds(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  /**
   * @brief Get width of a cluster
   * @param cluster Input point cloud cluster
   * @return Width of the cluster
   */
  double getClusterWidth(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster);

  // Getters for the point cloud data
  PointCPtr getCloudPtr() const { return g_cloud_ptr; }
  PointCPtr getFilteredCloudPtr() const { return g_cloud_filtered; }
  PointCPtr getFilteredCloud2Ptr() const { return g_cloud_filtered2; }
  PointCPtr getFilteredOctomapCloudPtr() const { return g_cloud_filtered_octomap; }
  pcl::PointCloud<pcl::PointXYZ>::Ptr getOctomapPtr() const { return g_octomap_ptr; }
  pcl::PointCloud<pcl::PointXYZ>::Ptr getFilteredOctomapPtr() const { return g_octomap_filtered; }
  std::string getInputFrameId() const { return g_input_pc_frame_id_; }
  std::string getOctomapFrameId() const { return g_octomap_frame_id_; }

  // Configure filters
  void setupCrossFilter();
  void setupNoughtFilter();

private:
  /** \brief The input point cloud frame id. */
  std::string g_input_pc_frame_id_;
  
  /** \brief The octomap point cloud frame id. */
  std::string g_octomap_frame_id_;

  /** \brief Point Cloud (input) pointer. */
  PointCPtr g_cloud_ptr;
  
  /** \brief Point Cloud (filtered) pointer. */
  PointCPtr g_cloud_filtered, g_cloud_filtered2, g_cloud_filtered_octomap;

  /** \brief The octomap point cloud. */
  pcl::PCLPointCloud2 g_octomap_pc;
  
  /** \brief The octomap point cloud pointer. */
  pcl::PointCloud<pcl::PointXYZ>::Ptr g_octomap_ptr;
  
  /** \brief The filtered octomap point cloud pointer. */
  pcl::PointCloud<pcl::PointXYZ>::Ptr g_octomap_filtered;

  /** \brief Point Cloud (input). */
  pcl::PCLPointCloud2 g_pcl_pc;
  
  /** \brief Voxel Grid filter. */
  pcl::VoxelGrid<PointT> g_vx;
  
  /** \brief Pass Through filter. */
  pcl::PassThrough<PointT> g_pt;
  
  /** \brief Crop box filter. */
  pcl::CropBox<pcl::PointXYZRGBA> nought_filter;
  
  /** \brief Pass Through filter. */
  pcl::PassThrough<pcl::PointXYZRGBA> cross_filter;

  /** \brief Octomap point cloud filter. */
  pcl::PassThrough<pcl::PointXYZ> g_octomap_pt;

  /** \brief Pass Through min and max threshold sizes. */
  double g_pt_thrs_min, g_pt_thrs_max;

  /** \brief Voxel Grid filter's leaf size. */
  double g_vg_leaf_sz;

  /** \brief Normals nn size. */
  int g_k_nn;

  /** \brief Point Cloud (filtered) sensros_msg for publ. */
  sensor_msgs::PointCloud2 g_cloud_filtered_msg;

  /** \brief The filtered octomap point cloud message. */
  sensor_msgs::PointCloud2 g_octomap_filtered_msg;
};

} // namespace cw2

#endif // POINT_CLOUD_PROCESSING_H_
