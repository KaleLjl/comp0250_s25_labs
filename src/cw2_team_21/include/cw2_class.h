#ifndef CW2_CLASS_H_
#define CW2_CLASS_H_

// system includes
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

// include services from the spawner package - we will be responding to these
#include "cw2_world_spawner/Task1Service.h"
#include "cw2_world_spawner/Task2Service.h"
#include "cw2_world_spawner/Task3Service.h"

// Include our modularized components
#include "robot/robot_control.h"
#include "perception/point_cloud_processing.h"
#include "perception/image_processing.h"
#include "task1/pick_and_place.h"
#include "task2/shape_detection.h"
#include "task3/environment_scanner.h"

namespace cw2 {

class cw2 {
public:
  /**
   * @brief Constructor for the cw2 class
   * @param nh ROS node handle
   */
  cw2(ros::NodeHandle nh);

  /**
   * @brief Load configuration values
   */
  void load_config();

private:
  // ROS node handle
  ros::NodeHandle nh_;

  // ROS service servers
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  // ROS publishers
  ros::Publisher g_pub_cloud;
  ros::Publisher g_pub_pose;
  ros::Publisher g_pub_cloud_octomap;
  ros::Publisher g_pub_octomap;

  // ROS subscribers
  ros::Subscriber image_sub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber octomap_pointcloud_sub_;

  // Flag for task-specific filtering
  bool task_1_filter = false;
  bool task_3_filter = false;

  // Module instances
  RobotControl robot_control_;
  PointCloudProcessing point_cloud_processing_;
  ImageProcessing image_processing_;
  PickAndPlace pick_and_place_;
  ShapeDetection shape_detection_;
  EnvironmentScanner environment_scanner_;

  // Constants
  double pi_ = 3.14159;
  double inspection_distance_;
  double angle_offset_;
  double drop_height_;
  double cross_pick_grid_y_offset_;
  double cross_pick_grid_x_offset_;
  double naught_pick_grid_x_offset_;
  double naught_pick_grid_y_offset_;
  double camera_offset_;
  geometry_msgs::Point scan_position_;

  // Callback functions for ROS services
  bool t1_callback(cw2_world_spawner::Task1Service::Request &request,
                  cw2_world_spawner::Task1Service::Response &response);
  bool t2_callback(cw2_world_spawner::Task2Service::Request &request,
                  cw2_world_spawner::Task2Service::Response &response);
  bool t3_callback(cw2_world_spawner::Task3Service::Request &request,
                  cw2_world_spawner::Task3Service::Response &response);

  // Callback functions for ROS subscribers
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg);
  void colorImageCallback(const sensor_msgs::Image& msg);
  void octomapCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg);

  // Task implementations
  bool task_1(geometry_msgs::Point object, 
             geometry_msgs::Point target, 
             std::string shape_type, 
             double width = 0.04);
  int64_t task_2(std::vector<geometry_msgs::PointStamped> ref, 
                geometry_msgs::PointStamped mystery);
  std::tuple<uint64_t, uint64_t> task_3();
};

} // namespace cw2

#endif // CW2_CLASS_H_
