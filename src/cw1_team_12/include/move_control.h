#ifndef MOVE_CONTROL_H
#define MOVE_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <vector>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace cw1_move {
// Moves the arm to a target pose.
bool moveArm(moveit::planning_interface::MoveGroupInterface &arm_group,
             const geometry_msgs::Pose &target_pose);

// Controls the gripper with specified width.
bool moveGripper(moveit::planning_interface::MoveGroupInterface &hand_group,
                 float width);

// Sets joint constraints for the arm.
void set_constraint(moveit::planning_interface::MoveGroupInterface &arm_group,
                    moveit_msgs::Constraints &constraints);

// Pick and place operation.
// 'cubes' and 'baskets' are vectors of positions.
// 'base_cube_name' is used to generate collision object names (e.g., "red_cube1").
void pick_and_place(moveit::planning_interface::MoveGroupInterface &arm_group,
                    moveit::planning_interface::MoveGroupInterface &hand_group,
                    moveit::planning_interface::PlanningSceneInterface &psi,
                    const std::vector<geometry_msgs::Point> &cubes,
                    const std::vector<geometry_msgs::Point> &baskets,
                    const std::string &base_cube_name);
}
#endif // MOVE_CONTROL_H
