#include "move_control.h"
#include "collision_control.h"
#include <cmath>

namespace cw1_move
{
bool moveArm(moveit::planning_interface::MoveGroupInterface &arm_group,
             const geometry_msgs::Pose &target_pose)
{
    ROS_INFO("Setting pose target");
    arm_group.setPoseTarget(target_pose);

    ROS_INFO("Planning arm motion");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (arm_group.plan(my_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Arm plan %s", success ? "SUCCESS" : "FAILED");

    if(success)
    {
        arm_group.move();
    }
    return success;
}

bool moveGripper(moveit::planning_interface::MoveGroupInterface &hand_group,
                 float width)
{
    double eachJoint = width / 2.0;
    std::vector<double> gripperJointTargets(2, eachJoint);
    hand_group.setJointValueTarget(gripperJointTargets);

    ROS_INFO("Planning gripper motion");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (hand_group.plan(my_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Gripper plan %s", success ? "SUCCESS" : "FAILED");

    if(success)
    {
        hand_group.move();
    }
    return success;
}

void set_constraint(moveit::planning_interface::MoveGroupInterface &arm_group,
                    moveit_msgs::Constraints &constraints)
{
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

    constraints.joint_constraints.clear();
    constraints.joint_constraints.push_back(joint_2);
    constraints.joint_constraints.push_back(joint_3);
    constraints.joint_constraints.push_back(joint_4);
    constraints.joint_constraints.push_back(joint_5);

    arm_group.setPathConstraints(constraints);
}

void pick_and_place(moveit::planning_interface::MoveGroupInterface &arm_group,
                    moveit::planning_interface::MoveGroupInterface &hand_group,
                    moveit::planning_interface::PlanningSceneInterface &psi,
                    const std::vector<geometry_msgs::Point> &cubes,
                    const std::vector<geometry_msgs::Point> &baskets,
                    const std::string &base_cube_name)
{
    if(cubes.empty() || baskets.empty())
    {
        ROS_WARN("No cubes or baskets provided for pick and place.");
        return;
    }

    geometry_msgs::Pose target_cube, target_basket;
    // Set basket target pose (using first basket position).
    target_basket.position = baskets[0];
    target_basket.position.z = 0.35;
    target_basket.orientation.x = 0.9239;
    target_basket.orientation.y = -0.3827;
    target_basket.orientation.z = 0;
    target_basket.orientation.w = 0;

    for (size_t i = 0; i < cubes.size(); i++)
    {
        // Compose cube collision object name, e.g., "red_cube1"
        std::string cube_name = base_cube_name + std::to_string(i + 1);
        target_cube.position = cubes[i];
        target_cube.orientation.x = 0.9239;
        target_cube.orientation.y = -0.3827;
        target_cube.orientation.z = 0;
        target_cube.orientation.w = 0;

        // Open gripper before picking.
        moveGripper(hand_group, 1);

        // Move above cube.
        target_cube.position.z = 0.175;
        moveArm(arm_group, target_cube);

        // Lower to pick cube.
        target_cube.position.z = 0.155;
        moveArm(arm_group, target_cube);

        // (Optionally remove collision object for cube here.)
        cw1_collision::removeCollisionObject(psi,cube_name);

        // Close gripper to grasp cube.
        moveGripper(hand_group, 0.01);

        // Lift the cube.
        target_cube.position.z = 0.2;
        moveArm(arm_group, target_cube);

        // Move to basket position.
        moveArm(arm_group, target_basket);

        // Open gripper to release cube.
        moveGripper(hand_group, 1);

        ROS_INFO("Pick and place finished for cube: %s", cube_name.c_str());
    }
}

} // namespace cw1
