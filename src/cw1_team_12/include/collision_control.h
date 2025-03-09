#ifndef COLLISION_CONTROL_H
#define COLLISION_CONTROL_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

namespace cw1_collision {
// Adds a collision object (box) to the planning scene.
void addCollisionObject(moveit::planning_interface::PlanningSceneInterface &psi,
                          const std::string &object_name,
                          const geometry_msgs::Point &centre,
                          const geometry_msgs::Vector3 &dimensions,
                          const geometry_msgs::Quaternion &orientation);

// Removes a collision object from the planning scene.
void removeCollisionObject(moveit::planning_interface::PlanningSceneInterface &psi,
                             const std::string &object_name);

// Removes all collision objects from the planning scene.
void removeAllCollisions(moveit::planning_interface::PlanningSceneInterface &psi);

// Adds a plate collision object.
void addPlate(moveit::planning_interface::PlanningSceneInterface &psi,
              const std::string &plate_name);

// Adds cube collision objects from given positions.
void addCube(moveit::planning_interface::PlanningSceneInterface &psi,
             const std::vector<geometry_msgs::Point> &positions,
             const std::string &base_name);

// Adds basket collision objects from given positions.
void addBasket(moveit::planning_interface::PlanningSceneInterface &psi,
               const std::vector<geometry_msgs::Point> &positions,
               const std::string &base_name);

// Adds all collision objects (plate, cubes, baskets) to the scene.
void addCollisions(moveit::planning_interface::PlanningSceneInterface &psi,
                   const std::vector<geometry_msgs::Point> &red_cubes,
                   const std::vector<geometry_msgs::Point> &blue_cubes,
                   const std::vector<geometry_msgs::Point> &purple_cubes,
                   const std::vector<geometry_msgs::Point> &red_baskets,
                   const std::vector<geometry_msgs::Point> &blue_baskets,
                   const std::vector<geometry_msgs::Point> &purple_baskets);
}
#endif // COLLISION_CONTROL_H
