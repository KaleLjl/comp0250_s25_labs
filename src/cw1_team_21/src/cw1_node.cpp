/* Software License Agreement (MIT License)
 *
 *  Copyright (c) 2025 Jiale Li, Renkai Liu, Zhengyang Zhu
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the “Software”), to
 *  deal in the Software without restriction, including without limitation the
 *  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 *  sell copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 *  IN THE SOFTWARE.
 */

#include <cw1_class.h>

int main(int argc, char **argv){
  
  ros::init(argc, argv, "cw1_solution_node");
  ros::NodeHandle nh("~");

  // MoveIt! requirement for non-blocking group.move()
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // create an instance of the cw1 class
  CW1 cw_class(nh);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_cloud_one =
    nh.subscribe("/r200/camera/depth_registered/points",
                 1,
                 &CW1::cloudCallBackOne,
                 &cw_class);
  ros::Subscriber sub_cloud_two =
    nh.subscribe("/r200/camera/depth_registered/points",
                 1,
                 &CW1::cloudCallBackTwo,
                 &cw_class);

  ros::Rate loop_rate(10);

  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}