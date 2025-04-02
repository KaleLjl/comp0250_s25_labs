#include "task2/shape_detection.h"

namespace cw2 {

ShapeDetection::ShapeDetection(RobotControl& robot_control, ImageProcessing& image_proc)
  : robot_control_(robot_control),
    image_proc_(image_proc)
{
  // Constructor
}

int64_t ShapeDetection::executeTask(std::vector<geometry_msgs::PointStamped> ref, 
                                   geometry_msgs::PointStamped mystery) {
  // Initialise output string
  std::string output_string = "Reference Shape 1: ";

  // Inspect first reference shape
  std::string ref_shape_1 = checkShape(ref[0].point);
  output_string += ref_shape_1;
  output_string += " | Reference Shape 2: ";

  // Infer second reference shape
  std::string ref_shape_2;
  if (ref_shape_1 == "nought") {
    ref_shape_2 = "cross";
  } else {
    ref_shape_2 = "nought";
  }
  output_string += ref_shape_2;
  output_string += " | Mystery Shape: ";

  // Inspect mystery shape
  std::string mystery_shape = checkShape(mystery.point);
  output_string += mystery_shape;
  int64_t mystery_object_num;
  if (mystery_shape == ref_shape_1) {
    mystery_object_num = 1;
  } else {
    mystery_object_num = 2;
  }

  // Print results
  ROS_INFO("/////////////////////////////////////////////////////////////////////");
  ROS_INFO("%s", output_string.c_str());
  ROS_INFO("/////////////////////////////////////////////////////////////////////");

  return mystery_object_num;
}

std::string ShapeDetection::checkShape(geometry_msgs::Point point) {
  // Define imaging pose
  geometry_msgs::Pose image_pose = robot_control_.point2Pose(point);
  image_pose.position.z += 0.3; // Offset above object
  image_pose.position.x -= 0.04; // Offset of camera from end-effector

  // Move camera above shape
  bool success = robot_control_.moveArm(image_pose);
  
  // Use the image processor to check the shape
  return image_proc_.checkShape();
}

} // namespace cw2
