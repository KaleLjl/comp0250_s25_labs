#ifndef SHAPE_DETECTION_H_
#define SHAPE_DETECTION_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <string>
#include <vector>

#include "robot/robot_control.h"
#include "perception/image_processing.h"

namespace cw2 {

class ShapeDetection {
public:
  /**
   * @brief Constructor for the ShapeDetection class
   * @param robot_control Reference to the robot control module
   * @param image_proc Reference to the image processing module
   */
  ShapeDetection(RobotControl& robot_control, ImageProcessing& image_proc);

  /**
   * @brief Execute Task 2 - Detect and identify shapes
   * @param ref Vector of reference object positions
   * @param mystery Position of the mystery object
   * @return Identifier for the mystery object (1 or 2)
   */
  int64_t executeTask(std::vector<geometry_msgs::PointStamped> ref, 
                     geometry_msgs::PointStamped mystery);

  /**
   * @brief Check shape at a given position
   * @param point Location to check
   * @return String describing the shape ("cross" or "nought")
   */
  std::string checkShape(geometry_msgs::Point point);

private:
  /** \brief Reference to the robot control module */
  RobotControl& robot_control_;
  
  /** \brief Reference to the image processing module */
  ImageProcessing& image_proc_;
};

} // namespace cw2

#endif // SHAPE_DETECTION_H_
