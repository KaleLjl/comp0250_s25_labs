#ifndef ROBOT_CONTROL_H_
#define ROBOT_CONTROL_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace cw2 {

class RobotControl {
public:
  /**
   * @brief Constructor for the RobotControl class
   */
  RobotControl();

  /**
   * @brief Moves the robot arm to a target pose
   * @param target_pose The target pose for the arm
   * @return true if the arm moved successfully
   */
  bool moveArm(geometry_msgs::Pose target_pose);

  /**
   * @brief Moves the robot gripper to a specified width
   * @param width The width to open/close the gripper
   * @return true if the gripper moved successfully
   */
  bool moveGripper(float width);

  /**
   * @brief Converts a point to a pose with an optional rotation
   * @param point The point to convert
   * @param rotation Optional rotation around z-axis
   * @return The resulting pose
   */
  geometry_msgs::Pose point2Pose(geometry_msgs::Point point, double rotation = 0.0);

  /**
   * @brief Performs a pick and place operation
   * @param objectPoint The position of the object to pick
   * @param objectGoal The target position to place the object
   * @param rotation Optional rotation for picking the object
   * @return true if the operation was successful
   */
  bool pickAndPlace(geometry_msgs::Point objectPoint, 
                  geometry_msgs::Point objectGoal, 
                  double rotation = 0.0);

  // Getters for gripper state
  float getGripperOpen() const { return gripper_open_; }
  float getGripperClosed() const { return gripper_closed_; }
  double getAngleOffset() const { return angle_offset_; }

  // Setters for configuration values
  void setAngleOffset(double angle_offset) { angle_offset_ = angle_offset; }

private:
  /** \brief MoveIt interface to move groups to separate the arm and the gripper */
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

  /** \brief Gripper constants */
  double angle_offset_ = M_PI / 4.0;
  float gripper_open_ = 80e-3;
  float gripper_closed_ = 0.0;
};

} // namespace cw2

#endif // ROBOT_CONTROL_H_
