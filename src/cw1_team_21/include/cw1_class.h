/* Software License Agreement (MIT License)
 *
 *  Copyright (c) 2025 Jiale Li, Renkai Liu, Zhengyang Zhu
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the “Software”), to
 *  deal in the Software without restriction, including without limitation the
 *  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 *  sell copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 *  IN THE SOFTWARE.
 */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW1_CLASS_H_
#define CW1_CLASS_H_

// system includes
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <tf/transform_listener.h>

// include services from the spawner package - we will be responding to these
#include "cw1_world_spawner/Task1Service.h"
#include "cw1_world_spawner/Task2Service.h"
#include "cw1_world_spawner/Task3Service.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

/** \brief Coursework 1.
  *
  * \authors Jiale Li, Renkai Liu, Zhengyang Zhu
  */
class CW1
{
  public:

    /* ----- class member functions ----- */

    // constructor
    CW1(ros::NodeHandle &nh);

    // service callbacks for tasks 1, 2, and 3
    bool 
    task1Callback(cw1_world_spawner::Task1Service::Request &request,
      cw1_world_spawner::Task1Service::Response &response);
    bool 
    task2Callback(cw1_world_spawner::Task2Service::Request &request,
      cw1_world_spawner::Task2Service::Response &response);
    bool 
    task3Callback(cw1_world_spawner::Task3Service::Request &request,
      cw1_world_spawner::Task3Service::Response &response);

    /** \brief Set joint constraints for task 1 and task 3. */
    void
    setConstraint();

    /** \brief Set joint constraints for task 2. */
    void
    setConstraint2();

    /** \brief MoveIt function for moving the move_group to the target position
      *
      * \input[in] target_pose pose to move the arm to
      *
      * \return true if moved to target position 
      */
    bool 
    moveArm(geometry_msgs::Pose target_pose);

    /** \brief MoveIt function for moving the gripper fingers to a new position.
      *
      * \input[in] width desired gripper finger width
      *
      * \return true if gripper fingers are moved to the new position
      */
    bool 
    moveGripper(float width);


    /** \brief MoveIt function for adding a cuboid collision object in RViz
      * and the MoveIt planning scene
      *
      * \input[in] object_name name for the new object to be added
      * \input[in] centre point at which to add the new object
      * \input[in] dimensions dimensions of the cuboid to add in x,y,z
      * \input[in] orientation rotation to apply to the cuboid before adding
      */
    void
    addCollisionObject(std::string object_name, geometry_msgs::Point centre,
      geometry_msgs::Vector3 dimensions, geometry_msgs::Quaternion orientation);

    /** \brief MoveIt function for removing a cuboid collision object in RViz
      * and the MoveIt planning scene
      *
      * \input[in] object_name name for the object to be removed
      */
    void
    removeCollisionObject(std::string object_name);

    /** \brief MoveIt function for removing all collision objects in RViz
      * and the MoveIt planning scene
      */
    void
    clearCollisionObjects();

    /** \brief MoveIt function for adding multiple collision objects in RViz
      * and the MoveIt planning scene
      *
      * \input[in] in_vec vector of geometry_msgs::Point objects to add
      * \input[in] name name for the new object to be added
      */
    void
    addCollisionObjects(std::vector<geometry_msgs::Point> in_vec,
                        std::string name);

    /** \brief Point Cloud CallBack function for task 2.
      * 
      * \input[in] cloud_input a PointCloud2 sensor_msgs const pointer
      */
    void
    cloudCallBackOne(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);
    
    /** \brief Point Cloud CallBack function for task 3.
      * 
      * \input[in] cloud_input a PointCloud2 sensor_msgs const pointer
      */
    void
    cloudCallBackTwo(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);

    /** \brief Apply Pass Through filtering.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      */
    void
    applyPT(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);

    /** \brief Apply Voxel Grid filtering.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      */
    void
    applyVX(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);

    /** \brief Normal estimation.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    findNormals(PointCPtr &in_cloud_ptr);
     
    /** \brief Segment Plane from point cloud.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    segPlane(PointCPtr &in_cloud_ptr);
     
    /** \brief Segment Cylinder from point cloud.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    segCylind(PointCPtr &in_cloud_ptr);

    /** \brief Point Cloud publisher.
      * 
      *  \input pc_pub ROS publisher
      *  \input pc point cloud to be published
      */
    void
    pubFilteredPCMsg(ros::Publisher &pc_pub, PointC &pc);

    /** \brief Cluster extraction.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    cluster(PointCPtr &in_cloud_ptr);

    /** \brief Find the Pose of Cylinder.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
     void
     findCylPose(PointCPtr &in_cloud_ptr);

    /** \brief Remove overlapped objects.
      * 
      * \input[in] in_vector the input vector of geometry_msgs::Point
      * 
      * \return the output vector of geometry_msgs::Point
      */
    std::vector<geometry_msgs::Point>
    removeOverlappedObjects(std::vector<geometry_msgs::Point> in_vector);

    /** \brief Pick and place function.
      * 
      * \input[in] cube vector of geometry_msgs::Point for the cube
      * \input[in] basket vector of geometry_msgs::Point for the basket
      * \input[in] cube_name name of the cube
      */
    void
    pickAndPlace(std::vector<geometry_msgs::Point> cube,
                 std::vector<geometry_msgs::Point> basket,
                 std::string cube_name);

    /* ----- class member variables ----- */

    /** \brief Define some useful constant values. */
    std::string base_frame_ = "panda_link0";
    double gripper_open_ = 80e-3;
    double gripper_closed_ = 0.0;
    geometry_msgs::Quaternion hand_orientation_, object_orientation_;
    geometry_msgs::Point tile_position_, task_3_position_, cyl_pt_msg_point_;
    geometry_msgs::Vector3 tile_dimensions_, box_dimensions_,
    basket_dimensions_;

    /** \brief Node handle. */
    ros::NodeHandle nh_;

    /** \brief Servers for the three tasks. */
    ros::ServiceServer t1_service_;
    ros::ServiceServer t2_service_;
    ros::ServiceServer t3_service_;

    /** \brief MoveIt interface to move groups to seperate the arm and the
      * gripper, these are defined in urdf. */
    moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
    moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

    /** \brief MoveIt interface to interact with the moveit planning scene
      * (eg collision objects). */
    moveit::planning_interface::PlanningSceneInterface
    planning_scene_interface_;

    /** \brief The input point cloud frame id. */
    std::string input_pc_frame_id_;

    /** \brief ROS publishers. */
    ros::Publisher pub_cloud_;

    /** \brief Point Cloud (inputs). */
    pcl::PCLPointCloud2 pcl_pc_, pcl_pc2_;

    /** \brief Point Cloud pointers. */
    PointCPtr cloud_ptr_, cloud_ptr2_, cloud_filtered_, cloud_filtered2_,
    cloud_filtered3_, cloud_filtered4_, cloud_plane_, cloud_cylinder_;

    /** \brief Pass Through filter. */
    pcl::PassThrough<PointT> pt_;

    /** \brief Voxel Grid filter. */
    pcl::VoxelGrid<PointT> vx_;

    /** \brief SAC segmentation. */
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_;

    /** \brief Normal estimation. */
    pcl::NormalEstimation<PointT, pcl::Normal> ne_;

    /** \brief Extract point cloud indices. */
    pcl::ExtractIndices<PointT> extract_pc_;
  
    /** \brief Extract point cloud normal indices. */
    pcl::ExtractIndices<pcl::Normal> extract_normals_;

    /** \brief Point indices for plane and cylinder. */
    pcl::PointIndices::Ptr inliers_plane_, inliers_cylinder_;

    /** \brief Model coefficients for plane and cylinder. */
    pcl::ModelCoefficients::Ptr coeff_plane_, coeff_cylinder_;

    /** \brief Point Cloud (filtered) sensros_msg for publ. */
    sensor_msgs::PointCloud2 cloud_filtered_msg_;

    /** \brief Cloud of normals. */
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_, cloud_normals2_;

    /** \brief KDTree for nearest neighborhood search. */
    pcl::search::KdTree<PointT>::Ptr tree_ptr_;

    /** \brief RGB data. */
    uint32_t rgba_;

    /** \brief RGB of blue. */
    double blue_[3] = {0.1, 0.1, 0.8};

    /** \brief RGB of red. */
    double red_[3] = {0.8, 0.1, 0.1};

    /** \brief RGB of purple. */
    double purple_[3] = {0.8, 0.1, 0.8};

    std::vector<geometry_msgs::Point> red_cube_, blue_cube_, purple_cube_;
    std::vector<geometry_msgs::Point> red_basket_, blue_basket_, purple_basket_;

    bool is_triggered = false;
    bool is_clustering = false;
    bool is_clustered = false;
    bool is_moved = false;

    /** \brief Euclidean Cluster Extraction. */
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec_;

    /** \brief Extract Indices. */
    std::vector<pcl::PointIndices> cluster_indices_;

    /** \brief Single cluster point indices. */
    pcl::PointIndices::Ptr single_cluster_ptr_;
    pcl::PointCloud<PointT>::Ptr single_cluster_;

    /** \brief ROS geometry message point. */
    geometry_msgs::PointStamped cyl_pt_msg_;

    /** \brief TF listener. */
    tf::TransformListener listener_;

  protected:
    /** \brief Debug mode. */
    bool debug_;
};

#endif // end of include guard for CW1_CLASS_H_
