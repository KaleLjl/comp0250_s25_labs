#include "task1/pick_and_place.h"

namespace cw2 {

PickAndPlace::PickAndPlace(RobotControl& robot_control, PointCloudProcessing& point_cloud_proc)
  : robot_control_(robot_control),
    point_cloud_proc_(point_cloud_proc)
{
  // Constructor
}

bool PickAndPlace::executeTask(geometry_msgs::Point object, 
                              geometry_msgs::Point target, 
                              std::string shape_type,
                              double width) {
  // Position camera above object
  geometry_msgs::Pose view_pose = robot_control_.point2Pose(object);
  view_pose.position.z = 0.6; // Distance above object
  view_pose.position.x -= 0.04; // Camera offset
  
  // Move arm
  bool success = robot_control_.moveArm(view_pose);

  // Create snapshot of point cloud for processing
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  if (shape_type == "cross") {
    pcl::copyPointCloud(*(point_cloud_proc_.getFilteredCloud2Ptr()), *cloud);
  } else {
    pcl::copyPointCloud(*(point_cloud_proc_.getFilteredCloudPtr()), *cloud);
  }

  // PCL feature extractor to obtain object orientation
  pcl::MomentOfInertiaEstimation<pcl::PointXYZRGBA> feature_extractor;
  feature_extractor.setInputCloud(cloud);
  feature_extractor.compute();
  
  // Determine Object Axis
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);

  // Compute angle between object local y-axis and global y-axis
  double angle;
  // Angle between vectors (2D - x/y plane only)
  double dot = middle_vector(1);
  double det = middle_vector(0);
  angle = atan2(det, dot);

  // Remove obtuse angles
  if (angle > pi_/2) {
    angle -= pi_;
  } else if (angle < -pi_/2) {
    angle += pi_;
  }
  
  // Minimise rotation to +- 45 degrees
  if (angle > pi_/4) {
    angle -= pi_/2;
  } else if (angle < -pi_/4) {
    angle += pi_/2;
  }

  ROS_INFO("CALCULATED ANGLE: (%f)", angle * (180/pi_));

  // Positional offsets for pick and place
  if (shape_type == "cross") {
    object.x += width * cos(angle);
    object.y += width * sin(angle);
    target.x += width;
  } else {
    object.x += 2 * width * sin(angle);
    object.y += 2 * width * cos(angle);
    target.y += 2 * width;
  }

  success *= robot_control_.pickAndPlace(object, target, -angle);

  return success;
}

} // namespace cw2
