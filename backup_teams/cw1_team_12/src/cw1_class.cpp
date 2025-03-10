#include "cw1_class.h"
#include "publish_aux.h"
#include "point_cloud_processing.h"
#include "move_control.h"
#include "collision_control.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

///////////////////////////////////////////////////////////////////////////////

cw1::cw1(ros::NodeHandle nh):
  g_cloud_ptr (new PointC), // input point cloud
  g_cloud_ptr_task_3 (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  g_cloud_filtered2 (new PointC), // filtered point cloud
  g_cloud_filtered_task_3 (new PointC), // filtered point cloud
  g_cloud_filtered2_task_3 (new PointC), // filtered point cloud
  g_cloud_plane (new PointC), // plane point cloud
  g_cloud_cylinder (new PointC), // cylinder point cloud
  g_tree_ptr (new pcl::search::KdTree<PointT>), // KdTree

  singal_cluster_ptr (new pcl::PointIndices), //cluster
  singal_cluster(new pcl::PointCloud<PointT>), //singal cluster

  g_cloud_normals (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_cloud_normals2 (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_inliers_plane (new pcl::PointIndices), // plane seg
  g_inliers_cylinder (new pcl::PointIndices), // cylidenr seg
  g_coeff_plane (new pcl::ModelCoefficients), // plane coeff
  g_coeff_cylinder (new pcl::ModelCoefficients) // cylinder coeff
{
  /* class constructor */

  nh_ = nh;

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw1::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw1::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw1::t3_callback, this);

  ros::Subscriber sub_target_pose = 
                  nh_.subscribe ("/target_pos",
                  10,
                  &cw1::target_pose_callback,
                  this);

  // define the publisher
  g_pub_task_2 = nh_.advertise<sensor_msgs::PointCloud2> ("filtered_task_2", 1, true);
  g_pub_task_3 = nh_.advertise<sensor_msgs::PointCloud2> ("filtered_task_3", 1, true);
  g_pub_pose = nh_.advertise<geometry_msgs::PointStamped> ("target_pos", 1, true);

  // publisher for cluster testing
  pub_cluster = nh_.advertise<sensor_msgs::PointCloud2> ("cluster_testing", 1, true);

  // define the constants for PCL
  g_vg_leaf_sz = 0.0005; // VoxelGrid leaf size: Better in a config file
  g_pt_thrs_min_x = -0.05; // PassThrough min thres: Better in a config file
  g_pt_thrs_max_x = 0.05; // PassThrough max thres: Better in a config file
  g_pt_thrs_min_y = -0.01; // PassThrough min thres: Better in a config file
  g_pt_thrs_max_y = 0.09; // PassThrough max thres: Better in a config file
  g_k_nn = 50; // Normals nn size: Better in a config file

  ROS_INFO("cw1 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */

  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  // add the collision in the RViz
  geometry_msgs::Point plate_position;
  plate_position.x = 0;
  plate_position.y = 0;
  plate_position.z = 0;

  geometry_msgs::Vector3 plate_dimension;
  plate_dimension.x = 5;
  plate_dimension.y = 5;
  plate_dimension.z = 0.03;

  geometry_msgs::Quaternion plate_orientation;
  plate_orientation.w = 1;
  plate_orientation.x = 0;
  plate_orientation.y = 0;
  plate_orientation.z = 0;

  cw1_collision::addCollisionObject(planning_scene_interface_, plate_name, plate_position, plate_dimension, plate_orientation);

  geometry_msgs::Vector3 cube_dimension;
  cube_dimension.x = 0.07;
  cube_dimension.y = 0.07;
  cube_dimension.z = 0.08;

  geometry_msgs::Quaternion cube_orientation;
  cube_orientation.w = 1;
  cube_orientation.x = 0;
  cube_orientation.y = 0;
  cube_orientation.z = 0;

  // add the cube
  cw1_collision::addCollisionObject(planning_scene_interface_, cube_name, request.object_loc.pose.position, cube_dimension, cube_orientation);

  geometry_msgs::Vector3 basket_dimension;
  basket_dimension.x = 0.12;
  basket_dimension.y = 0.12;
  basket_dimension.z = 0.16;

  geometry_msgs::Quaternion basket_orientation;
  basket_orientation.w = 1;
  basket_orientation.x = 0;
  basket_orientation.y = 0;
  basket_orientation.z = 0;

  // add the basket
  cw1_collision::addCollisionObject(planning_scene_interface_, basket_name, request.goal_loc.point, basket_dimension, basket_orientation);

  // define the constraint to joints
  cw1_move::set_constraint(arm_group_, constraints);

  // move the arm to achieve pick and place
  geometry_msgs::Pose target_cube, target_cube_ahead, target_goal, intermedia;

  // define the position and orientation of the cube
  target_cube.position = request.object_loc.pose.position;
  target_cube.orientation.x = 0.9239;
  target_cube.orientation.y = -0.3827;
  target_cube.orientation.z = 0;
  target_cube.orientation.w = 0;
  target_cube.position.z = target_cube.position.z + 0.12;

  // define the target_cube_ahead
  target_cube_ahead = target_cube;
  target_cube_ahead.position.z = 0.18;
  target_cube_ahead.orientation.x = 0.9239;
  target_cube_ahead.orientation.y = -0.3827;
  target_cube_ahead.orientation.z = 0;
  target_cube_ahead.orientation.w = 0;

  // define the position and orientation of the goal
  target_goal.position = request.goal_loc.point;
  target_goal.orientation = target_cube.orientation;
  target_goal.orientation.x = 1;
  target_goal.orientation.y = 0;
  target_goal.position.z = 0.35;

  // define the position and orientation of the intermedia point
  intermedia = target_cube_ahead;
  intermedia.position.z = target_cube_ahead.position.z;

  cw1_move::moveGripper(hand_group_, 1);
  cw1_move::moveArm(arm_group_, target_cube_ahead);
  cw1_move::moveArm(arm_group_, target_cube);
  cw1_collision::removeCollisionObject(planning_scene_interface_, cube_name);
  cw1_move::moveGripper(hand_group_, 0.01);
  cw1_move::moveArm(arm_group_, target_goal);
  cw1_move::moveGripper(hand_group_, 1);
  cw1_collision::removeCollisionObject(planning_scene_interface_, basket_name);
  cw1_collision::removeCollisionObject(planning_scene_interface_, plate_name);
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t2_callback(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  // get the number of target
  int numb_targets;
  numb_targets = request.basket_locs.size();

  // moving the arm to the targets
  // define the pose of manipulator
  geometry_msgs::Pose target;
  target.orientation.x = 0.9239;
  target.orientation.y = -0.3827;
  target.orientation.z = 0;
  target.orientation.w = 0;

  // define the constraint to joints
  moveit_msgs::Constraints constraints;
  moveit_msgs::JointConstraint joint_3, joint_5;
  joint_5.joint_name = "panda_joint5";
  joint_5.position = 0.0;
  joint_5.tolerance_above = 0.5;
  joint_5.tolerance_below = 0.5;
  joint_5.weight = 1.0;

  joint_3.joint_name = "panda_joint3";
  joint_3.position = 0.0;
  joint_3.tolerance_above = 0.5;
  joint_3.tolerance_below = 0.5;
  joint_3.weight = 1.0;

  constraints.joint_constraints.push_back(joint_3);
  constraints.joint_constraints.push_back(joint_5);
  arm_group_.setPathConstraints(constraints);

  // initialize the output
  std::vector<std::string> output;
  output.resize(numb_targets);

  // loop the location of the baskets
  for (int i=0; i<numb_targets; i++){
    target.position = request.basket_locs[i].point;
    // adjust the camera ahead of the basket
    target.position.z = 0.5;
    cw1_move::moveArm(arm_group_, target);

    // initialize the r, g, b data
    int r = 0;
    int g = 0;
    int b = 0;
    // retrieve the RGB data
    for (int j=0; j<(*g_cloud_filtered2).points.size(); j++){
      rgba = (*g_cloud_filtered2).points[j].rgba;
      uint8_t uint8_r = (rgba >> 16) & 0x0000ff;
      uint8_t uint8_g = (rgba >> 8)  & 0x0000ff;
      uint8_t uint8_b = (rgba)       & 0x0000ff;
      uint8_t uint8_a = (rgba >> 24) & 0x000000ff;

      r = r + uint8_r;
      g = g + uint8_g;
      b = b + uint8_b;
    }
    // take the average number of rgb of the image area
    r = r / (*g_cloud_filtered2).points.size();
    g = g / (*g_cloud_filtered2).points.size();
    b = b / (*g_cloud_filtered2).points.size();

    if (r/255.0 <= red[0] + 0.1 && g/255.0 <= red[1] + 0.1 && b/255.0 <= red[2] + 0.1){
      output[i] = "red";
    }
    else if (r/255.0 <= blue[0] + 0.1 && g/255.0 <= blue[1] + 0.1 && b/255.0 <= blue[2] + 0.1){
      output[i] = "blue";
    }
    else if (r/255.0 <= purple[0] + 0.1 && g/255.0 <= purple[1] + 0.1 && b/255.0 <= purple[2] + 0.1){
      output[i] = "purple";
    }
    else {
      output[i] = "empty";
    }
  }

  // assign the output to the response
  response.basket_colours = output;

  for (const auto &s : output) {
    std::cout << s << std::endl;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  cw1_collision::removeAllCollisions(planning_scene_interface_);

  // initialise the flags
  cluster_or_not = false;
  task_3_triger = false;
  cluster_done = false;

  // clear the vectors
  red_cube.clear();
  blue_cube.clear();
  purple_cube.clear();
  red_basket.clear();
  blue_basket.clear();
  purple_basket.clear();

  // define the constraint to joints
  cw1_move::set_constraint(arm_group_, constraints);

  // move the arm to the target pose
  geometry_msgs::Pose target_pose_1, target_pose_2;
  target_pose_1.position.x = 0.41;
  target_pose_1.position.y = 0.3;
  target_pose_1.position.z = 0.7;

  target_pose_1.orientation.x = 0.9239;
  target_pose_1.orientation.y = -0.3827;
  target_pose_1.orientation.z = 0;
  target_pose_1.orientation.w = 0;

  target_pose_2 = target_pose_1;
  target_pose_2.position.y = -target_pose_1.position.y;

  // move to the first detection position
  cw1_move::moveArm(arm_group_, target_pose_1);

  cluster_or_not = true;
  task_3_triger = true;

  // hand on until the first clustering is finished
  while(!cluster_done)
  {}

  // move to the second detection position
  cluster_or_not = false;
  task_3_triger = false;
  cluster_done = false;
  cw1_move::moveArm(arm_group_, target_pose_2);

  cluster_or_not = true;
  task_3_triger = true;

  // hand on until the second clustering is finished
  while(!cluster_done)
  {}
  cluster_or_not = false;
  task_3_triger = false;

  // discard the repeated position
  red_cube = cw1_publish::discardRepeatPositions(red_cube, 0.01);
  blue_cube = cw1_publish::discardRepeatPositions(blue_cube, 0.01);
  purple_cube = cw1_publish::discardRepeatPositions(purple_cube, 0.01);
  red_basket = cw1_publish::discardRepeatPositions(red_basket, 0.01);
  blue_basket = cw1_publish::discardRepeatPositions(blue_basket, 0.01);
  purple_basket = cw1_publish::discardRepeatPositions(purple_basket, 0.01);
  ROS_INFO("Discard the repeated cube and basket finished");

  // Print cube and basket positions for debugging if needed
  cw1_publish::printCubeBasket(red_cube, blue_cube, purple_cube, red_basket, blue_basket, purple_basket);

  // add the collision for path planning
  cw1_collision::addCollisions(planning_scene_interface_, red_cube, blue_cube, purple_cube, 
                red_basket, blue_basket, purple_basket);

  ROS_INFO("Pick and place starts");
  cw1_move::pick_and_place(arm_group_, hand_group_, planning_scene_interface_, red_cube, red_basket, "red_cube");
  cw1_move::pick_and_place(arm_group_, hand_group_, planning_scene_interface_, blue_cube, blue_basket, "blue_cube");
  cw1_move::pick_and_place(arm_group_, hand_group_, planning_scene_interface_, purple_cube, purple_basket, "purple_cube");

  return true;
}

void
cw1::cloudCallBackOne(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Extract input point cloud info
  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;
    
  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2(g_pcl_pc, *g_cloud_ptr);

  // apply the filter to filter specific area
  cw1_pcl::applyPassThroughX(g_cloud_ptr, g_cloud_filtered, g_pt_thrs_min_x, g_pt_thrs_max_x);
  cw1_pcl::applyPassThroughY(g_cloud_filtered, g_cloud_filtered2, g_pt_thrs_min_y, g_pt_thrs_max_y);

  cw1_pcl::publishFilteredPointCloud(g_pub_task_2, *g_cloud_filtered2);

  return;
}

void cw1::cloudCallBackTwo(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  while(task_3_triger)
  {
    g_input_pc_frame_id_ = cloud_msg->header.frame_id;
    pcl_conversions::toPCL(*cloud_msg, g_pcl_pc_task_3);
    pcl::fromPCLPointCloud2(g_pcl_pc_task_3, *g_cloud_ptr_task_3);

    // Downsample
    cw1_pcl::applyVoxelGrid(g_cloud_ptr_task_3, g_cloud_filtered_task_3, g_vg_leaf_sz);

    // Estimate normals
    cw1_pcl::findNormals(g_cloud_filtered_task_3, g_cloud_normals, g_k_nn);

    // Segment plane
    cw1_pcl::segmentPlane(g_cloud_filtered_task_3,
                 g_inliers_plane, g_coeff_plane,
                 g_cloud_plane, g_cloud_filtered2_task_3,
                 g_cloud_normals, g_cloud_normals2,
                 0.03, 0.0001, 1000);

    // Segment cylinder
    cw1_pcl::segmentCylinder(g_cloud_filtered2_task_3,
                    g_cloud_normals2,
                    g_inliers_cylinder, g_coeff_cylinder,
                    g_cloud_cylinder,
                    0.1, 0.0001, 10000);

    // Clustering if triggered
    if(cluster_or_not)
    {
      cluster_or_not = false;

      std::vector<pcl::PointIndices> cluster_inds;
      cw1_pcl::clusterPointCloud(g_cloud_cylinder, cluster_inds, 0.02, 100, 25000);

      // For each cluster, compute color & centroid
      for(const auto &inds : cluster_inds)
      {
        pcl::PointIndices::Ptr single(new pcl::PointIndices(inds));
        PointCPtr single_cluster(new PointC);

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(g_cloud_cylinder);
        extract.setIndices(single);
        extract.setNegative(false);
        extract.filter(*single_cluster);

        // Compute average RGB
        int rr = 0, gg = 0, bb = 0;
        for(const auto &pt : single_cluster->points)
        {
          uint32_t val = pt.rgba;
          rr += (val >> 16) & 0x0000ff;
          gg += (val >> 8)  & 0x0000ff;
          bb += (val)       & 0x0000ff;
        }
        int size = single_cluster->points.size();
        if(size > 0)
        {
          rr /= size; gg /= size; bb /= size;
        }
        double r_ = rr/255.0, g_ = gg/255.0, b_ = bb/255.0;

        // Find centroid in base frame
        geometry_msgs::Point cpos = cw1_pcl::findCylPose(single_cluster, "panda_link0", g_listener_);

        // Distinguish cube vs basket by point size
        if(r_ <= red[0]+0.05 && g_ <= red[1]+0.05 && b_ <= red[2]+0.05)
        {
          if(size > 4500) red_basket.push_back(cpos);
          else            red_cube.push_back(cpos);
        }
        else if(r_ <= blue[0]+0.05 && g_ <= blue[1]+0.05 && b_ <= blue[2]+0.05)
        {
          if(size > 4500) blue_basket.push_back(cpos);
          else            blue_cube.push_back(cpos);
        }
        else if(r_ <= purple[0]+0.05 && g_ <= purple[1]+0.05 && b_ <= purple[2]+0.05)
        {
          if(size > 4500) purple_basket.push_back(cpos);
          else            purple_cube.push_back(cpos);
        }
      }
      cluster_done = true;
    }

    // Publish for debug
    sensor_msgs::PointCloud2 out;
    pcl::toROSMsg(*g_cloud_cylinder, out);
    cw1_publish::publishFilteredPointCloud(g_pub_task_3, out);
  }
}

void cw1::target_pose_callback(const geometry_msgs::PointStamped &msg)
{
  cyl_pt_msg_header = msg.header;
  cyl_pt_msg_point  = msg.point;
}