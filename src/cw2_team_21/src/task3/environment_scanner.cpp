#include "task3/environment_scanner.h"

namespace cw2 {

EnvironmentScanner::EnvironmentScanner(RobotControl& robot_control, 
                                     ImageProcessing& image_proc,
                                     PointCloudProcessing& point_cloud_proc,
                                     PickAndPlace& pick_and_place)
  : robot_control_(robot_control),
    image_proc_(image_proc),
    point_cloud_proc_(point_cloud_proc),
    pick_and_place_(pick_and_place)
{
  // Constructor
}

std::tuple<uint64_t, uint64_t> EnvironmentScanner::executeTask() {
  // Scan the environment for objects and obstacles
  scanEnvironment();

  ROS_INFO("Starting to cluster");

  // Create snapshot of point cloud for processing
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr octomap_filtered = point_cloud_proc_.getFilteredOctomapPtr();
  
  // Check if the filtered octomap is empty
  if (octomap_filtered->empty()) {
    ROS_ERROR("Filtered octomap point cloud is empty! Cannot proceed with clustering.");
    return std::make_tuple(0, 0);
  }
  
  pcl::copyPointCloud(*octomap_filtered, *cloud);
  ROS_INFO("Copied point cloud with %lu points for clustering", cloud->size());

  // Cluster the point cloud into separate objects
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = 
    point_cloud_proc_.clusterPointClouds(cloud);

  ROS_INFO("Finished clustering");

  // Initializing the object positions vectors
  std::vector<geometry_msgs::Point> object_positions;
  std::vector<geometry_msgs::Point> obstacle_positions;
  std::vector<geometry_msgs::Point> cross_positions;
  std::vector<geometry_msgs::Point> nought_positions;
  geometry_msgs::Point basket_position;

  for (auto cluster : clusters) {
    // Compute the centroid of the cluster
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);

    // Create a point to store the centroid position
    geometry_msgs::Point centroid_position;

    // Round to 3 decimal places
    centroid_position.x = std::round(centroid[0] * position_precision_) / position_precision_;
    centroid_position.y = std::round(centroid[1] * position_precision_) / position_precision_;
    centroid_position.z = 0;

    // Ignore the centroid if it is at the origin
    if (centroid_position.x < 0.1 && centroid_position.x > -0.1 && 
        centroid_position.y < 0.1 && centroid_position.y > -0.1) {
      continue;
    }

    ROS_INFO("Object centroid: (%f, %f, %f)", 
             centroid_position.x, centroid_position.y, centroid_position.z);

    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D(*cluster, min_point, max_point);
    ROS_INFO("Object dimensions: (%f, %f, %f)", 
             max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z);

    // Identify the basket
    if (max_point.x - min_point.x > 0.25 && max_point.y - min_point.y > 0.25) {
      basket_position = centroid_position;
    } else {
      // Compute the width for pick and place
      double width = point_cloud_proc_.getClusterWidth(cluster) / 5.0;

      // Add the centroid position to the vector
      object_positions.push_back(centroid_position);
    }
  }

  // Sort the object positions
  for (auto object : object_positions) {
    // Move the arm to the object
    geometry_msgs::Point target = object;
    target.x = target.x - camera_offset_; // Offset of camera from end-effector
    target.y = target.y;
    target.z = 0.5;
    geometry_msgs::Pose target_pose = robot_control_.point2Pose(target);
    robot_control_.moveArm(target_pose);

    // Identify the object
    Object target_obj = image_proc_.checkObject();

    if (target_obj == Object::obstacle) {
      obstacle_positions.push_back(object);
    } else if (target_obj == Object::basket) {
      basket_position = object;
    } else if (target_obj == Object::cross) {
      cross_positions.push_back(object);
    } else if (target_obj == Object::nought) {
      nought_positions.push_back(object);
    }

    // add delay
    ros::Duration(1.0).sleep();
  }

  if (cross_positions.size() > nought_positions.size()) {
    geometry_msgs::Point target = cross_positions.back();
    bool success = pick_and_place_.executeTask(target, basket_position, "cross");

    ROS_INFO("/////////////////////////////////////////////////////////////////////");
    ROS_INFO_STREAM("Number of objects: " << cross_positions.size() + nought_positions.size());
    ROS_INFO_STREAM("Number of crosses: " << cross_positions.size());
    ROS_INFO("/////////////////////////////////////////////////////////////////////");

    return std::make_tuple(cross_positions.size() + nought_positions.size(), cross_positions.size());
  } else {
    geometry_msgs::Point target = nought_positions.back();
    bool success = pick_and_place_.executeTask(target, basket_position, "nought");

    ROS_INFO("/////////////////////////////////////////////////////////////////////");
    ROS_INFO_STREAM("Number of objects: " << cross_positions.size() + nought_positions.size());
    ROS_INFO_STREAM("Number of noughts: " << nought_positions.size());
    ROS_INFO("/////////////////////////////////////////////////////////////////////");

    return std::make_tuple(cross_positions.size() + nought_positions.size(), nought_positions.size());
  }
}

void EnvironmentScanner::scanEnvironment() {
  geometry_msgs::Point reset_point;
  reset_point.x = 0.5;
  reset_point.y = 0.0;
  reset_point.z = 0.5;
  geometry_msgs::Pose reset_pose = robot_control_.point2Pose(reset_point);
  bool reset_success = robot_control_.moveArm(reset_pose);

  // set speed of arm
  // Note: This would need to be exposed in the RobotControl class
  // arm_group_.setMaxVelocityScalingFactor(0.05);

  // Create corners of scan area
  geometry_msgs::Point corner1;
  corner1.x = -0.50;
  corner1.y = -0.40;
  corner1.z = 0.6;
  geometry_msgs::Point corner2;
  corner2.x = 0.50;
  corner2.y = -0.40;
  corner2.z = 0.65;
  geometry_msgs::Point corner3;
  corner3.x = 0.50;
  corner3.y = 0.30;
  corner3.z = 0.6;
  geometry_msgs::Point corner4;
  corner4.x = -0.50;
  corner4.y = 0.30;
  corner4.z = 0.6;

  // Add corners to a vector
  std::vector<geometry_msgs::Point> corners;
  corners.push_back(corner1);
  corners.push_back(corner2);
  corners.push_back(corner3);
  corners.push_back(corner4);
  corners.push_back(corner1);

  // Set constant gripper angle
  double angle = -1.5708;
  int num_steps = 4;

  geometry_msgs::Pose pose;
  geometry_msgs::Point distance;
  geometry_msgs::Point step;
  bool success;

  for (int i = 0; i < corners.size() - 1; i++) { 
    // Move arm to corner position
    pose = robot_control_.point2Pose(corners.at(i), angle);
    success = robot_control_.moveArm(pose);

    // Enable filter when arm is in position
    // Note: This would need to be exposed in the main class
    // if (i == 0)
    //   task_3_filter = true;

    // Initialise variable to store distance between points
    distance.x = corners.at(i).x - corners.at(i+1).x;
    distance.y = corners.at(i).y - corners.at(i+1).y;
    distance.z = 0;
    
    for (int j = 1; j < num_steps - 1; j++) {
      // Calculate step distance
      step.x = corners.at(i).x - (j * distance.x / num_steps);
      step.y = corners.at(i).y - (j * distance.y / num_steps);
      step.z = corners.at(i).z;

      ROS_INFO("Step: (%f, %f, %f)", step.x, step.y, step.z);

      // Move arm to step position
      pose = robot_control_.point2Pose(step, angle);
      success = robot_control_.moveArm(pose);

      if (i == 3 && j == 2)
        j++;
    }
  }
  ros::Duration(1.0).sleep();

  // Disable filter when scan is complete
  // Note: This would need to be exposed in the main class
  // task_3_filter = false;
  
  // Reset the arm velocity
  // Note: This would need to be exposed in the RobotControl class
  // arm_group_.setMaxVelocityScalingFactor(0.1);
}

} // namespace cw2
