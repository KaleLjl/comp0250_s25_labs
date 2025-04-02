#ifndef PICK_AND_PLACE_H_
#define PICK_AND_PLACE_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include "robot/robot_control.h"
#include "perception/point_cloud_processing.h"

namespace cw2 {

class PickAndPlace {
public:
  /**
   * @brief Constructor for the PickAndPlace class
   * @param robot_control Reference to the robot control module
   * @param point_cloud_proc Reference to the point cloud processing module
   */
  PickAndPlace(RobotControl& robot_control, PointCloudProcessing& point_cloud_proc);

  /**
   * @brief Execute Task 1 - Pick and place object
   * @param object Position of the object to pick
   * @param target Position to place the object
   * @param shape_type Type of shape ("cross" or "nought")
   * @param width Optional width of the object
   * @return true if the operation was successful
   */
  bool executeTask(geometry_msgs::Point object, 
                  geometry_msgs::Point target, 
                  std::string shape_type,
                  double width = 0.04);

private:
  /** \brief Reference to the robot control module */
  RobotControl& robot_control_;
  
  /** \brief Reference to the point cloud processing module */
  PointCloudProcessing& point_cloud_proc_;

  /** \brief Pi constant */
  const double pi_ = 3.14159;
};

} // namespace cw2

#endif // PICK_AND_PLACE_H_
