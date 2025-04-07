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

#include <cw1_class.h>

///////////////////////////////////////////////////////////////////////////////

CW1::CW1(ros::NodeHandle &nh):
debug_(false),
cloud_ptr_(new PointC),
cloud_ptr2_(new PointC),
cloud_filtered_(new PointC),
cloud_filtered2_(new PointC),
cloud_filtered3_(new PointC),
cloud_filtered4_(new PointC),
cloud_plane_(new PointC),
cloud_cylinder_(new PointC),
tree_ptr_(new pcl::search::KdTree<PointT>),
single_cluster_ptr_(new pcl::PointIndices),
single_cluster_(new PointC),
cloud_normals_(new pcl::PointCloud<pcl::Normal>),
cloud_normals2_(new pcl::PointCloud<pcl::Normal>),
inliers_plane_(new pcl::PointIndices),
inliers_cylinder_(new pcl::PointIndices),
coeff_plane_(new pcl::ModelCoefficients),
coeff_cylinder_(new pcl::ModelCoefficients)
{
  /* class constructor */

  nh_ = nh;

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &CW1::task1Callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &CW1::task2Callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &CW1::task3Callback, this);

  hand_orientation_ = tf2::toMsg(tf2::Quaternion(0.9239, -0.3827, 0, 0));
  object_orientation_ = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));
  tile_position_.x = 0.45;
  tile_position_.y = 0;
  tile_position_.z = 0;
  tile_dimensions_ = tf2::toMsg(tf2::Vector3(0.5, 0.9, 0.02));
  box_dimensions_ = tf2::toMsg(tf2::Vector3(0.04, 0.04, 0.04));
  basket_dimensions_ = tf2::toMsg(tf2::Vector3(0.1, 0.1, 0.1));


  ROS_INFO("cw1 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

bool
CW1::task1Callback(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */

  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  // set the constraints for the arm
  setConstraint();

  // define the object pose and goal pose
  geometry_msgs::Pose object_pose, above_object_pose, goal_pose;

  object_pose = request.object_loc.pose;
  object_pose.position.z += 0.12;
  object_pose.orientation = hand_orientation_;

  above_object_pose = object_pose;
  above_object_pose.position.z = 0.18;

  goal_pose.position = request.goal_loc.point;
  goal_pose.orientation = hand_orientation_;
  goal_pose.position.z = 0.35;

  // add the collision objects to the scene
  addCollisionObject("tile", tile_position_, tile_dimensions_,
    object_orientation_);
  addCollisionObject("box", request.object_loc.pose.position, box_dimensions_,
    request.object_loc.pose.orientation);
  addCollisionObject("basket", request.goal_loc.point, basket_dimensions_,
    request.object_loc.pose.orientation);

  // perform the pick and place operation
  moveGripper(gripper_open_);
  moveArm(above_object_pose);
  moveArm(object_pose);
  removeCollisionObject("box");
  moveGripper(gripper_closed_);
  moveArm(goal_pose);
  moveGripper(gripper_open_);
  clearCollisionObjects();

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW1::task2Callback(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  int num_baskets = request.basket_locs.size();

  geometry_msgs::Pose target;
  target.orientation = hand_orientation_;

  // set the constraint to joints
  setConstraint2();

  // initialize the output
  std::vector<std::string> output;
  output.resize(num_baskets);

  // loop the location of the baskets
  for (int i=0; i<num_baskets; i++){
    target.position = request.basket_locs[i].point;
    // adjust the camera ahead of the basket
    target.position.z = 0.5;
    moveArm(target);

    // initialize the r, g, b data
    int r = 0;
    int g = 0;
    int b = 0;
    // retrieve the RGB data
    for (int j=0; j<(*cloud_filtered_).points.size(); j++){
      rgba_ = (*cloud_filtered_).points[j].rgba;
      uint8_t uint8_r = (rgba_ >> 16) & 0x0000ff;
      uint8_t uint8_g = (rgba_ >> 8)  & 0x0000ff;
      uint8_t uint8_b = (rgba_)       & 0x0000ff;

      r += uint8_r;
      g += uint8_g;
      b += uint8_b;
    }
    // take the average number of rgb of the image area
    r = r / (*cloud_filtered_).points.size();
    g = g / (*cloud_filtered_).points.size();
    b = b / (*cloud_filtered_).points.size();

    if (abs(r / 255.0 - blue_[0]) <= 0.1 && abs(g / 255.0 - blue_[1]) <= 0.1 &&
        abs(b / 255.0 - blue_[2]) <= 0.1)
    {
      output[i] = "blue";
    }
    else if (abs(r / 255.0 - red_[0]) <= 0.1 && abs(g / 255.0 - red_[1]) <= 0.1
             && abs(b / 255.0 - red_[2]) <= 0.1)
    {
      output[i] = "red";
    }
    else if (abs(r / 255.0 - purple_[0]) <= 0.1 &&
      abs(g / 255.0 - purple_[1]) <= 0.1 && abs(b / 255.0 - purple_[2]) <= 0.1)
    {
      output[i] = "purple";
    }
    else
    {
      output[i] = "none";
    }
  }

  // assign the output to the response
  response.basket_colours = output;

  // print the output
  for (int i=0; i<num_baskets; i++){
    ROS_INFO("Basket %d: %s", i + 1, response.basket_colours[i].c_str());
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW1::task3Callback(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  // initialise the flags
  is_triggered = false;
  is_clustering = false;
  is_clustered = false;

  // clear the vectors
  red_cube_.clear();
  blue_cube_.clear();
  purple_cube_.clear();
  red_basket_.clear();
  blue_basket_.clear();
  purple_basket_.clear();

  // define the constraint to joints
  setConstraint();

  // move the arm to the target pose
  geometry_msgs::Pose target_pose_1, target_pose_2;
  target_pose_1.position.x = 0.41;
  target_pose_1.position.y = 0.3;
  target_pose_1.position.z = 0.7;

  target_pose_1.orientation = hand_orientation_;

  target_pose_2 = target_pose_1;
  target_pose_2.position.y = -target_pose_1.position.y;

  // move to the first detection position
  moveArm(target_pose_1);

  is_triggered = true;
  is_clustering = true;

  // hand on until the first clustering is finished
  while(!is_clustered)
  {}

  // move to the second detection position
  is_triggered = false;
  is_clustering = false;
  is_clustered = false;
  moveArm(target_pose_2);

  is_triggered = true;
  is_clustering = true;

  // hand on until the second clustering is finished
  while(!is_clustered)
  {}
  is_triggered = true;
  is_clustering = true;

  // discard the repeated position
  red_cube_ = removeOverlappedObjects(red_cube_);
  blue_cube_ = removeOverlappedObjects(blue_cube_);
  purple_cube_ = removeOverlappedObjects(purple_cube_);
  red_basket_ = removeOverlappedObjects(red_basket_);
  blue_basket_ = removeOverlappedObjects(blue_basket_);
  purple_basket_ = removeOverlappedObjects(purple_basket_);
  ROS_INFO("Removed overlapped objects");

  // add the collision for path planning
  addCollisionObject("tile", tile_position_, tile_dimensions_,
                     object_orientation_);
  addCollisionObjects(blue_cube_, "blue_cube");
  addCollisionObjects(red_cube_, "red_cube");
  addCollisionObjects(purple_cube_, "purple_cube");
  addCollisionObjects(blue_basket_, "blue_basket");
  addCollisionObjects(red_basket_, "red_basket");
  addCollisionObjects(purple_basket_, "purple_basket");

  ROS_INFO("Pick and place starts");
  pickAndPlace(blue_cube_, blue_basket_, "blue_cube");
  pickAndPlace(red_cube_, red_basket_, "red_cube");
  pickAndPlace(purple_cube_, purple_basket_, "purple_cube");
  ROS_INFO("Pick and place finishes");

  return true;
}

////////////////////////////////////////////////////////////////////////////////

void
CW1::setConstraint()
{
  moveit_msgs::Constraints constraints;
  moveit_msgs::JointConstraint joint_2, joint_3, joint_4, joint_5;

  joint_2.joint_name = "panda_joint2";
  joint_2.position = 0.4;
  joint_2.tolerance_above = M_PI * 0.8;
  joint_2.tolerance_below = M_PI * 0.8;
  joint_2.weight = 1.0;

  joint_3.joint_name = "panda_joint3";
  joint_3.position = 0.0;
  joint_3.tolerance_above = M_PI * 0.25;
  joint_3.tolerance_below = M_PI * 0.25;
  joint_3.weight = 1.0;

  joint_4.joint_name = "panda_joint4";
  joint_4.position = -2.13;
  joint_4.tolerance_above = M_PI * 0.5;
  joint_4.tolerance_below = M_PI * 0.5;
  joint_4.weight = 1.0;

  joint_5.joint_name = "panda_joint5";
  joint_5.position = 0.0;
  joint_5.tolerance_above = M_PI * 0.3;
  joint_5.tolerance_below = M_PI * 0.3;
  joint_5.weight = 1.0;

  constraints.joint_constraints.push_back(joint_2);
  constraints.joint_constraints.push_back(joint_3);
  constraints.joint_constraints.push_back(joint_4);
  constraints.joint_constraints.push_back(joint_5);
  arm_group_.setPathConstraints(constraints);

  return;
}

////////////////////////////////////////////////////////////////////////////////

void
CW1::setConstraint2()
{
  moveit_msgs::Constraints constraints;
  moveit_msgs::JointConstraint joint_3, joint_5;

  joint_3.joint_name = "panda_joint3";
  joint_3.position = 0.0;
  joint_3.tolerance_above = 0.5;
  joint_3.tolerance_below = 0.5;
  joint_3.weight = 1.0;

  joint_5.joint_name = "panda_joint5";
  joint_5.position = 0.0;
  joint_5.tolerance_above = 0.5;
  joint_5.tolerance_below = 0.5;
  joint_5.weight = 1.0;

  constraints.joint_constraints.push_back(joint_3);
  constraints.joint_constraints.push_back(joint_5);
  arm_group_.setPathConstraints(constraints);

  return;
}

////////////////////////////////////////////////////////////////////////////////

bool 
CW1::moveArm(geometry_msgs::Pose target_pose)
{
  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // google 'c++ conditional operator' to understand this line
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();

  return success;
}

////////////////////////////////////////////////////////////////////////////////

bool 
CW1::moveGripper(float width)
{
  // safety checks in case width exceeds safe values
  if (width > gripper_open_) 
    width = gripper_open_;
  if (width < gripper_closed_) 
    width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // move the gripper joints
  hand_group_.move();

  return success;
}

////////////////////////////////////////////////////////////////////////////////

void 
CW1::addCollisionObject(std::string object_name,
  geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
  geometry_msgs::Quaternion orientation)
{
  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input header information
  collision_object.id = object_name;
  collision_object.header.frame_id = base_frame_;

  // define the primitive and its dimensions
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dimensions.x;
  collision_object.primitives[0].dimensions[1] = dimensions.y;
  collision_object.primitives[0].dimensions[2] = dimensions.z;

  // define the pose of the collision object
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x = centre.x;
  collision_object.primitive_poses[0].position.y = centre.y;
  collision_object.primitive_poses[0].position.z = centre.z;
  collision_object.primitive_poses[0].orientation = orientation;

  // define that we will be adding this collision object 
  // Hint: what about collision_object.REMOVE?
  collision_object.operation = collision_object.ADD;

  // add the collision object to the vector, then apply to planning scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);

  return;
}

////////////////////////////////////////////////////////////////////////////////

void 
CW1::removeCollisionObject(std::string object_name)
{
  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input header information
  collision_object.id = object_name;
  collision_object.header.frame_id = base_frame_;

  // define that we will be removing this collision object 
  collision_object.operation = collision_object.REMOVE;

  // add the collision object to the vector, then apply to planning scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);

  return;
}

////////////////////////////////////////////////////////////////////////////////

void
CW1::clearCollisionObjects()
{
  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;

  // define that we will be removing this collision object 
  collision_object.operation = collision_object.REMOVE;

  // add the collision object to the vector, then apply to planning scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);

  return;
}

////////////////////////////////////////////////////////////////////////////////

void
CW1::addCollisionObjects(std::vector<geometry_msgs::Point> in_vec, std::string name)
{
  geometry_msgs::Vector3 dimen;
  if (name == "red_cube" || name == "blue_cube" || name == "purple_cube")
  {
    dimen = box_dimensions_;
  }
  else
  {
    dimen = basket_dimensions_;
  }
  if (in_vec.size()>0)
  {
    for (int i=0; i<in_vec.size(); i++)
    {
      addCollisionObject(
        name + std::to_string(i + 1),
        in_vec[i],
        dimen,
        object_orientation_
      );
    }
  }

  return;
}

////////////////////////////////////////////////////////////////////////////////

void
CW1::cloudCallBackOne(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Extract inout point cloud info
  input_pc_frame_id_ = cloud_input_msg->header.frame_id;
    
  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_input_msg, pcl_pc_);
  pcl::fromPCLPointCloud2(pcl_pc_, *cloud_ptr_);

  // Perform the filtering
  applyPT(cloud_ptr_, cloud_filtered_);
    
  // Publish the data
  pubFilteredPCMsg(pub_cloud_, *cloud_filtered_);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
CW1::cloudCallBackTwo(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  while(is_triggered)
  {
    input_pc_frame_id_ = cloud_input_msg->header.frame_id;
    pcl_conversions::toPCL(*cloud_input_msg, pcl_pc2_);
    pcl::fromPCLPointCloud2(pcl_pc2_, *cloud_ptr2_);

    // Downsample
    applyVX(cloud_ptr2_, cloud_filtered3_);

    // Estimate normals
    findNormals(cloud_filtered3_);

    // Segment plane
    segPlane(cloud_filtered3_);

    // Segment cylinder
    segCylind(cloud_filtered3_);

    // Clustering if triggered
    if(is_clustering)
    {
      is_clustering = false;
      cluster(cloud_cylinder_);
    }

    // Publish the data
    pubFilteredPCMsg(pub_cloud_, *cloud_cylinder_);
  }
}

////////////////////////////////////////////////////////////////////////////////

void
CW1::applyPT(PointCPtr &in_cloud_ptr,
             PointCPtr &out_cloud_ptr)
{
  pt_.setInputCloud(in_cloud_ptr);
  pt_.setFilterFieldName("x");
  pt_.setFilterLimits(-0.05, 0.05);
  pt_.filter(*out_cloud_ptr);
  pt_.setInputCloud(out_cloud_ptr);
  pt_.setFilterFieldName("y");
  pt_.setFilterLimits(-0.01, 0.09);
  pt_.filter(*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
CW1::applyVX(PointCPtr &in_cloud_ptr,
             PointCPtr &out_cloud_ptr)
{
  vx_.setInputCloud(in_cloud_ptr);
  vx_.setLeafSize(0.01, 0.01, 0.01);
  vx_.filter(*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
CW1::findNormals(PointCPtr &in_cloud_ptr)
{
  // Estimate point normals
  ne_.setInputCloud(in_cloud_ptr);
  ne_.setSearchMethod(tree_ptr_);
  ne_.setKSearch(50);
  ne_.compute(*cloud_normals_);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
CW1::segPlane (PointCPtr &in_cloud_ptr)
{
  // Create the segmentation object for the planar model
  // and set all the params
  seg_.setOptimizeCoefficients(true);
  seg_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  seg_.setNormalDistanceWeight(0.1); //bad style
  seg_.setMethodType(pcl::SAC_RANSAC);
  seg_.setMaxIterations(100); //bad style
  seg_.setDistanceThreshold(0.03); //bad style
  seg_.setInputCloud(in_cloud_ptr);
  seg_.setInputNormals(cloud_normals_);
  // Obtain the plane inliers and coefficients
  seg_.segment(*inliers_plane_, *coeff_plane_);
  
  // Extract the planar inliers from the input cloud
  extract_pc_.setInputCloud(in_cloud_ptr);
  extract_pc_.setIndices(inliers_plane_);
  extract_pc_.setNegative(false);
  
  // Write the planar inliers to disk
  extract_pc_.filter(*cloud_plane_);
  
  // Remove the planar inliers, extract the rest
  extract_pc_.setNegative(true);
  extract_pc_.filter(*cloud_filtered4_);
  extract_normals_.setNegative(true);
  extract_normals_.setInputCloud(cloud_normals_);
  extract_normals_.setIndices(inliers_plane_);
  extract_normals_.filter(*cloud_normals2_);

  //ROS_INFO_STREAM ("Plane coefficients: " << *g_coeff_plane);
  ROS_INFO_STREAM("PointCloud representing the planar component: "
                  << cloud_plane_->size ()
                  << " data points.");
}
    
////////////////////////////////////////////////////////////////////////////////

void
CW1::segCylind (PointCPtr &in_cloud_ptr)
{
  // Create the segmentation object for cylinder segmentation
  // and set all the parameters
  seg_.setOptimizeCoefficients (true);
  seg_.setModelType(pcl::SACMODEL_CYLINDER);
  seg_.setMethodType(pcl::SAC_RANSAC);
  seg_.setNormalDistanceWeight(0.1); //bad style
  seg_.setMaxIterations(10000); //bad style
  seg_.setDistanceThreshold(0.05); //bad style
  seg_.setRadiusLimits(0, 0.1); //bad style
  seg_.setInputCloud(cloud_filtered4_);
  seg_.setInputNormals(cloud_normals2_);

  // Obtain the cylinder inliers and coefficients
  seg_.segment (*inliers_cylinder_, *coeff_cylinder_);
  
  // Write the cylinder inliers to disk
  extract_pc_.setInputCloud (cloud_filtered4_);
  extract_pc_.setIndices (inliers_cylinder_);
  extract_pc_.setNegative (false);
  extract_pc_.filter (*cloud_cylinder_);
  
  ROS_INFO_STREAM ("PointCloud representing the cylinder component: "
                   << cloud_cylinder_->size ()
                   << " data points.");
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
CW1::pubFilteredPCMsg(ros::Publisher &pc_pub, PointC &pc)
{
  // Publish the data
  pcl::toROSMsg(pc, cloud_filtered_msg_);
  pc_pub.publish(cloud_filtered_msg_);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
CW1::cluster(PointCPtr &in_cloud_ptr)
{

  ROS_INFO("start clustering");

  // use k-means to cluster the filtered point cloud, and get the indices
  tree_ptr_->setInputCloud(in_cloud_ptr);
  
  ec_.setClusterTolerance(0.02);
  ec_.setMinClusterSize(100);
  ec_.setMaxClusterSize(25000);

  ec_.setSearchMethod(tree_ptr_);

  ec_.setInputCloud(in_cloud_ptr);

  cluster_indices_.clear();
  ec_.extract(cluster_indices_);


  // extract the data from each cluster
  for (int i = 0; i < cluster_indices_.size(); i++)
  {
    *single_cluster_ptr_ = cluster_indices_[i];
    extract_pc_.setInputCloud(in_cloud_ptr);
    extract_pc_.setIndices(single_cluster_ptr_);
    extract_pc_.filter(*single_cluster_);

    // initialize the r, g, b data
    int r = 0;
    int g = 0;
    int b = 0;
    // retrieve the RGB data
    for (int j = 0; j < (*single_cluster_).points.size(); j++)
    {
      rgba_ = (*single_cluster_).points[j].rgba;
      uint8_t uint8_r = (rgba_ >> 16) & 0x0000ff;
      uint8_t uint8_g = (rgba_ >> 8) & 0x0000ff;
      uint8_t uint8_b = (rgba_) & 0x0000ff;

      r = r + uint8_r;
      g = g + uint8_g;
      b = b + uint8_b;
    }
    // take the average number of rgb of the image area
    r = r / (*single_cluster_).points.size();
    g = g / (*single_cluster_).points.size();
    b = b / (*single_cluster_).points.size();

    if (abs(r / 255.0 - red_[0]) <= 0.1 && abs(g / 255.0 - red_[1]) <= 0.1 &&
        abs(b / 255.0 - red_[2]) <= 0.1)
    {
      findCylPose(single_cluster_);
      if((*single_cluster_).points.size()>4500)
      {
        red_basket_.push_back(task_3_position_);
      }
      else
      {
        red_cube_.push_back(task_3_position_);
      }
    }
    else if (abs(r / 255.0 - blue_[0]) <= 0.1 &&
      abs(g / 255.0 - blue_[1]) <= 0.1 && abs(b / 255.0 - blue_[2]) <= 0.1)
    {
      findCylPose(single_cluster_);
      if((*single_cluster_).points.size()>4500)
      {
        blue_basket_.push_back(task_3_position_);
      }
      else
      {
        blue_cube_.push_back(task_3_position_);
      }
    }
    else if (abs(r / 255.0 - purple_[0]) <= 0.1 &&
      abs(g / 255.0 - purple_[1]) <= 0.1 && abs(b / 255.0 - purple_[2]) <= 0.1)
    {
      findCylPose(single_cluster_);
      if((*single_cluster_).points.size()>4500)
      {
        purple_basket_.push_back(task_3_position_);
      }
      else
      {
        purple_cube_.push_back(task_3_position_);
      }
    }

    is_moved = false;
  }

  ROS_INFO("Clustering finished");

  is_clustered = true;

  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::findCylPose(PointCPtr &in_cloud_ptr)
{
  task_3_position_.x = 0;
  task_3_position_.y = 0;
  task_3_position_.z = 0;

  Eigen::Vector4f centroid_in;
  pcl::compute3DCentroid(*in_cloud_ptr, centroid_in);
  
  cyl_pt_msg_.header.frame_id = input_pc_frame_id_;
  cyl_pt_msg_.header.stamp = ros::Time (0);
  cyl_pt_msg_.point.x = centroid_in[0];
  cyl_pt_msg_.point.y = centroid_in[1];
  cyl_pt_msg_.point.z = centroid_in[2];
  
  // Transform the point to new frame
  geometry_msgs::PointStamped cyl_pt_msg_out;
  try
  {
    listener_.transformPoint("panda_link0",
                             cyl_pt_msg_,
                             cyl_pt_msg_out);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("Received a trasnformation exception: %s", ex.what());
  }
  
  task_3_position_ = cyl_pt_msg_out.point;
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<geometry_msgs::Point>
CW1::removeOverlappedObjects(std::vector<geometry_msgs::Point> in_vector)
{
  std::vector<geometry_msgs::Point> temp_vec;
  geometry_msgs::Point temp_pos;
  bool same;

  if (in_vector.size() > 1)
  {
    for (int i = 0; i < in_vector.size()-1; i++)
    {
      same = false;
      temp_pos = in_vector[i];
      for (int j = i+1; j < in_vector.size(); j++)
      {
        if (abs(temp_pos.x-in_vector[j].x)<0.01 && abs(temp_pos.y-in_vector[j].y)<0.01)
        {
          same = true;
        }
      }
      if(!same)
      {
        temp_vec.push_back(temp_pos);
      }
    }
    temp_vec.push_back(in_vector[in_vector.size()-1]);

    // update the order 
    int n=temp_vec.size();
    bool swap;
    do{
      swap = false;
      for (int k=1; k<n; ++k)
      {
        if (temp_vec[k-1].x<temp_vec[k].x)
        {
          std::swap(temp_vec[k-1], temp_vec[k]);
          swap = true;
        }
      }
      --n;
    }while(swap);
  }
  else
  {
    temp_vec = in_vector;
  }

  return temp_vec;
}

////////////////////////////////////////////////////////////////////////////////

void
CW1::pickAndPlace(std::vector<geometry_msgs::Point> cube, 
                  std::vector<geometry_msgs::Point> basket,
                  std::string cube_name)
{
  if (cube.size()>0 && basket.size()>0)
  {
    geometry_msgs::Pose tar_cube, tar_basket;

    tar_basket.position = basket[0];
    tar_basket.position.z =  0.35;
    tar_basket.orientation = hand_orientation_;

    for (int i=0; i<cube.size(); i++)
    {
      std::string name = cube_name + std::to_string(i+1);
      tar_cube.position = cube[i];
      tar_cube.orientation = hand_orientation_;

      moveGripper(gripper_open_);
      tar_cube.position.z = 0.175;
      moveArm(tar_cube);
      tar_cube.position.z = 0.155;
      moveArm(tar_cube);
      removeCollisionObject(name);
      moveGripper(gripper_closed_);
      tar_cube.position.z = 0.2;
      moveArm(tar_basket);
      moveGripper(gripper_open_);
    }

  }

  return;
}