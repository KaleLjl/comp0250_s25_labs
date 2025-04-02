#include <cw2_class.h>

////////////////////////////////////////////////////////////////////////////////
// Constructor
////////////////////////////////////////////////////////////////////////////////
cw2::cw2::cw2(ros::NodeHandle nh) :
  // Initialize module instances
  robot_control_(),
  point_cloud_processing_(),
  image_processing_(),
  // Initialize modules that depend on other modules
  pick_and_place_(robot_control_, point_cloud_processing_),
  shape_detection_(robot_control_, image_processing_),
  environment_scanner_(robot_control_, image_processing_, point_cloud_processing_, pick_and_place_)
{
  /* class constructor */
  nh_ = nh;

  // advertise solutions for coursework tasks
  t1_service_ = nh_.advertiseService("/task1_start", &cw2::t1_callback, this);
  t2_service_ = nh_.advertiseService("/task2_start", &cw2::t2_callback, this);
  t3_service_ = nh_.advertiseService("/task3_start", &cw2::t3_callback, this);

  // Define the publishers
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1, true);
  g_pub_pose = nh.advertise<geometry_msgs::PointStamped>("cyld_pt", 1, true);
  g_pub_cloud_octomap = nh.advertise<sensor_msgs::PointCloud2>("octomap_cloud", 1, true);
  g_pub_octomap = nh.advertise<sensor_msgs::PointCloud2>("filtered_octomap_cloud", 1, true);

  // Initialise ROS Subscribers //
  image_sub_ = nh_.subscribe("/r200/camera/color/image_raw", 1, &cw2::colorImageCallback, this);
  // Create a ROS subscriber for the input point cloud
  cloud_sub_ = nh_.subscribe("/r200/camera/depth_registered/points", 1, &cw2::pointCloudCallback, this);
  // Create a ROS subscriber for the octomap output cloud
  octomap_pointcloud_sub_ = nh_.subscribe("/octomap_point_cloud_centers", 1, &cw2::octomapCallback, this);

  // Load configuration parameters
  load_config();

  // Initialize PCL processing with configuration values
  point_cloud_processing_.initialize();

  ROS_INFO("cw2 class initialised");
}

void cw2::cw2::load_config()
{
  // Define constants identified experimentally 

  // Pick and place constants
  inspection_distance_ = 0.6;
  // Angle offset to align gripper with cube
  angle_offset_ = pi_ / 4.0;

  // Update robot control with angle offset
  robot_control_.setAngleOffset(angle_offset_);

  drop_height_ = 0.30;
  cross_pick_grid_y_offset_ = 0;
  cross_pick_grid_x_offset_ = 2;
  naught_pick_grid_x_offset_ = 2;
  naught_pick_grid_y_offset_ = 2;

  // Defining the Robot scanning position for task 3
  scan_position_.x = 0.3773;
  scan_position_.y = -0.0015;
  scan_position_.z = 0.8773;

  // Positions of the gripper for the different tasks 
  camera_offset_ = 0.0425;
}

///////////////////////////////////////////////////////////////////////////////

bool cw2::cw2::t1_callback(cw2_world_spawner::Task1Service::Request &request,
  cw2_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */

  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  bool success = task_1(request.object_point.point, request.goal_point.point, request.shape_type);

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool cw2::cw2::t2_callback(cw2_world_spawner::Task2Service::Request &request,
  cw2_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  int64_t mystery_object_num = task_2(request.ref_object_points, request.mystery_object_point);

  response.mystery_object_num = mystery_object_num;

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool cw2::cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
  cw2_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  std::tuple<uint64_t, uint64_t> responses = task_3();

  response.total_num_shapes = std::get<0>(responses);
  response.num_most_common_shape = std::get<1>(responses);

  return true;
}

///////////////////////////////////////////////////////////////////////////////

void cw2::cw2::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Forward to point cloud processing module
  point_cloud_processing_.processPointCloud(cloud_input_msg);

  // Apply task specific filter
  if (task_1_filter)
  {
    // Apply nought filter
    point_cloud_processing_.setupNoughtFilter();
    
    // Apply cross filter
    point_cloud_processing_.setupCrossFilter();

    // Publish filtered point cloud
    point_cloud_processing_.pubFilteredPCMsg(g_pub_cloud, *(point_cloud_processing_.getFilteredCloud2Ptr()));
  }
  else if (task_3_filter)
  {
    // Apply octomap filter
    auto cloud_ptr = point_cloud_processing_.getCloudPtr();
    auto filtered_cloud_ptr = point_cloud_processing_.getFilteredOctomapCloudPtr();
    point_cloud_processing_.applyFilter(cloud_ptr, filtered_cloud_ptr);
    
    // Publish filtered point cloud for octomap
    point_cloud_processing_.pubFilteredPCMsg(g_pub_cloud_octomap, 
                                          *(point_cloud_processing_.getFilteredOctomapCloudPtr()));
  }
  
  return;
}

///////////////////////////////////////////////////////////////////////////////

void cw2::cw2::colorImageCallback(const sensor_msgs::Image& msg)
{
  // Forward to image processing module
  image_processing_.processColorImage(msg);
  
  return;
}

///////////////////////////////////////////////////////////////////////////////

void cw2::cw2::octomapCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Forward to point cloud processing module
  point_cloud_processing_.processOctomapPointCloud(cloud_input_msg);
  
  // Publish the filtered octomap
  sensor_msgs::PointCloud2 octomap_filtered_msg;
  pcl::toROSMsg(*(point_cloud_processing_.getFilteredOctomapPtr()), octomap_filtered_msg);
  g_pub_octomap.publish(octomap_filtered_msg);
}

////////////////////////////////////////////////////////////////////////////////

bool cw2::cw2::task_1(geometry_msgs::Point object, 
                    geometry_msgs::Point target, 
                    std::string shape_type, 
                    double width)
{
  // Enable task 1 filtering in point cloud callback
  task_1_filter = true;

  // Delegate to pick and place module
  bool success = pick_and_place_.executeTask(object, target, shape_type, width);
  
  // Disable task 1 filtering
  task_1_filter = false;

  return success;
}

////////////////////////////////////////////////////////////////////////////////

int64_t cw2::cw2::task_2(std::vector<geometry_msgs::PointStamped> ref, 
                        geometry_msgs::PointStamped mystery)
{
  // Delegate to shape detection module
  return shape_detection_.executeTask(ref, mystery);
}

///////////////////////////////////////////////////////////////////////////////

std::tuple<uint64_t, uint64_t> cw2::cw2::task_3()
{
  // Enable task 3 filtering in point cloud callback
  task_3_filter = true;
  
  // Delegate to environment scanner module
  std::tuple<uint64_t, uint64_t> result = environment_scanner_.executeTask();
  
  // Disable task 3 filtering
  task_3_filter = false;
  
  return result;
}
