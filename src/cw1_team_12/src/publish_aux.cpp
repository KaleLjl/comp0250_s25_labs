#include "publish_aux.h"
#include <pcl_conversions/pcl_conversions.h>

namespace cw1_publish
{
// Publish a filtered PointCloud2 message.
void publishFilteredPointCloud(ros::Publisher &pub, const sensor_msgs::PointCloud2 &cloud_msg)
{
    pub.publish(cloud_msg);
}

// Publish a PointStamped message with the given point.
void publishPose(ros::Publisher &pub, const geometry_msgs::Point &point)
{
    geometry_msgs::PointStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.point = point;
    pub.publish(msg);
}

// Callback to extract point from a PointStamped message.
void targetPoseCallback(const geometry_msgs::PointStamped &msg, geometry_msgs::Point &target_point)
{
    target_point = msg.point;
}

// Remove duplicate positions from a vector (threshold defines closeness).
std::vector<geometry_msgs::Point> discardRepeatPositions(const std::vector<geometry_msgs::Point> &positions, double threshold)
{
    std::vector<geometry_msgs::Point> unique_positions;
    for (size_t i = 0; i < positions.size(); i++)
    {
        bool duplicate = false;
        for (size_t j = 0; j < unique_positions.size(); j++)
        {
            if (std::abs(positions[i].x - unique_positions[j].x) < threshold &&
                std::abs(positions[i].y - unique_positions[j].y) < threshold)
            {
                duplicate = true;
                break;
            }
        }
        if (!duplicate)
        {
            unique_positions.push_back(positions[i]);
        }
    }
    return unique_positions;
}

// Print positions of cubes and baskets for debugging.
void printCubeBasket(const std::vector<geometry_msgs::Point> &red_cube,
                     const std::vector<geometry_msgs::Point> &blue_cube,
                     const std::vector<geometry_msgs::Point> &purple_cube,
                     const std::vector<geometry_msgs::Point> &red_basket,
                     const std::vector<geometry_msgs::Point> &blue_basket,
                     const std::vector<geometry_msgs::Point> &purple_basket)
{
    ROS_INFO("Red Cube Positions:");
    for (size_t i = 0; i < red_cube.size(); i++)
        ROS_INFO("  (%.3f, %.3f, %.3f)", red_cube[i].x, red_cube[i].y, red_cube[i].z);

    ROS_INFO("Blue Cube Positions:");
    for (size_t i = 0; i < blue_cube.size(); i++)
        ROS_INFO("  (%.3f, %.3f, %.3f)", blue_cube[i].x, blue_cube[i].y, blue_cube[i].z);

    ROS_INFO("Purple Cube Positions:");
    for (size_t i = 0; i < purple_cube.size(); i++)
        ROS_INFO("  (%.3f, %.3f, %.3f)", purple_cube[i].x, purple_cube[i].y, purple_cube[i].z);

    ROS_INFO("Red Basket Positions:");
    for (size_t i = 0; i < red_basket.size(); i++)
        ROS_INFO("  (%.3f, %.3f, %.3f)", red_basket[i].x, red_basket[i].y, red_basket[i].z);

    ROS_INFO("Blue Basket Positions:");
    for (size_t i = 0; i < blue_basket.size(); i++)
        ROS_INFO("  (%.3f, %.3f, %.3f)", blue_basket[i].x, blue_basket[i].y, blue_basket[i].z);

    ROS_INFO("Purple Basket Positions:");
    for (size_t i = 0; i < purple_basket.size(); i++)
        ROS_INFO("  (%.3f, %.3f, %.3f)", purple_basket[i].x, purple_basket[i].y, purple_basket[i].z);
}
}