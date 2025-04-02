#ifndef ENVIRONMENT_SCANNER_H_
#define ENVIRONMENT_SCANNER_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tuple>
#include <vector>

#include "robot/robot_control.h"
#include "perception/image_processing.h"
#include "perception/point_cloud_processing.h"
#include "task1/pick_and_place.h"

namespace cw2 {

class EnvironmentScanner {
public:
  /**
   * @brief Constructor for the EnvironmentScanner class
   * @param robot_control Reference to the robot control module
   * @param image_proc Reference to the image processing module
   * @param point_cloud_proc Reference to the point cloud processing module
   * @param pick_and_place Reference to the pick and place module
   */
  EnvironmentScanner(RobotControl& robot_control, 
                    ImageProcessing& image_proc,
                    PointCloudProcessing& point_cloud_proc,
                    PickAndPlace& pick_and_place);

  /**
   * @brief Execute Task 3 - Scan environment, identify objects, and interact with most common
   * @return Tuple with total number of shapes and count of most common shape
   */
  std::tuple<uint64_t, uint64_t> executeTask();

  /**
   * @brief Scan the environment by moving the robot arm to gather point cloud data
   */
  void scanEnvironment();

private:
  /** \brief Reference to the robot control module */
  RobotControl& robot_control_;
  
  /** \brief Reference to the image processing module */
  ImageProcessing& image_proc_;
  
  /** \brief Reference to the point cloud processing module */
  PointCloudProcessing& point_cloud_proc_;
  
  /** \brief Reference to the pick and place module */
  PickAndPlace& pick_and_place_;

  /** \brief Position precision for rounding */
  double position_precision_ = 10000;

  /** \brief Camera offset from end-effector */
  double camera_offset_ = 0.0425;
};

} // namespace cw2

#endif // ENVIRONMENT_SCANNER_H_
