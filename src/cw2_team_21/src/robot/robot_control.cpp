#include "robot/robot_control.h"

namespace cw2 {

RobotControl::RobotControl() {
  // Initialize MoveIt groups
}

bool RobotControl::moveArm(geometry_msgs::Pose target_pose) {
  // Setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // Create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // Execute the planned path
  arm_group_.move();

  return success;
}

bool RobotControl::moveGripper(float width) {
  // Safety checks
  if (width > gripper_open_) width = gripper_open_;
  if (width < gripper_closed_) width = gripper_closed_;

  // Calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // Create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // Apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // Move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  hand_group_.move();

  return success;
}

geometry_msgs::Pose RobotControl::point2Pose(geometry_msgs::Point point, double rotation) {
  // Position gripper above point
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  // Gripper Orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_ + rotation);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion orientation = tf2::toMsg(q_result);

  // Set the desired Pose
  geometry_msgs::Pose pose;
  pose.position = point;
  pose.orientation = orientation;

  return pose;
}

bool RobotControl::pickAndPlace(geometry_msgs::Point objectPoint, 
                              geometry_msgs::Point objectGoal,
                              double rotation) {
  // Generate Picking Pose:
  geometry_msgs::Pose grasp_pose = point2Pose(objectPoint, rotation);
  grasp_pose.position.z += 0.14; // Align shape with gripper
  // Approach and Takeaway:
  geometry_msgs::Pose offset_pose = grasp_pose;
  offset_pose.position.z += 0.125;
  // Releasing Object:
  geometry_msgs::Pose release_pose = point2Pose(objectGoal);
  release_pose.position.z += 0.35; // Position gripper above basket

  // Perform Pick
  bool success = true;
  // Approach
  success *= moveArm(offset_pose);
  success *= moveGripper(gripper_open_);
  success *= moveArm(grasp_pose);
  success *= moveGripper(gripper_closed_);
  // Takeaway
  success *= moveArm(offset_pose);
  offset_pose.position.z += 0.125;
  success *= moveArm(offset_pose);
  // Place
  success *= moveArm(release_pose);
  release_pose.position.z -= 0.15;
  success *= moveArm(release_pose);

  // Open gripper
  success *= moveGripper(gripper_open_);
  
  return success;
}

} // namespace cw2
