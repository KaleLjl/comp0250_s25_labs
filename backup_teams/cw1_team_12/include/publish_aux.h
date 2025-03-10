#ifndef PUBLISH_AUX_H
#define PUBLISH_AUX_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>

namespace cw1_publish
{
// Publish a filtered PointCloud2 message.
void publishFilteredPointCloud(ros::Publisher &pub, const sensor_msgs::PointCloud2 &cloud_msg);

// Publish a PointStamped message with a given point.
void publishPose(ros::Publisher &pub, const geometry_msgs::Point &point);

// Callback to extract point from a PointStamped message.
void targetPoseCallback(const geometry_msgs::PointStamped &msg, geometry_msgs::Point &target_point);

// Remove duplicate positions based on a threshold.
std::vector<geometry_msgs::Point> discardRepeatPositions(const std::vector<geometry_msgs::Point> &positions, double threshold);

// Print cube and basket positions for debugging.
void printCubeBasket(const std::vector<geometry_msgs::Point> &red_cube,
                     const std::vector<geometry_msgs::Point> &blue_cube,
                     const std::vector<geometry_msgs::Point> &purple_cube,
                     const std::vector<geometry_msgs::Point> &red_basket,
                     const std::vector<geometry_msgs::Point> &blue_basket,
                     const std::vector<geometry_msgs::Point> &purple_basket);
}
#endif // PUBLISH_AUX_H
