#include "perception/image_processing.h"

namespace cw2 {

ImageProcessing::ImageProcessing() 
  : color_image_width_(0),
    color_image_height_(0),
    color_image_midpoint_(0)
{
  // Default initialization
}

void ImageProcessing::processColorImage(const sensor_msgs::Image& msg) {
  // If image parameters not set up yet, initialize them
  if (color_image_width_ == 0) {
    setupImageParameters(msg.width, msg.height);
  }

  // Store the image data
  color_image_data = msg.data;
}

void ImageProcessing::setupImageParameters(int width, int height) {
  // Camera feed resolution
  color_image_width_ = width;
  color_image_height_ = height;

  // Computing the index of the middle pixel
  color_image_midpoint_ = color_channels_ * ((color_image_width_ * 
    (color_image_height_ / 2)) + (color_image_width_ / 2)) - color_channels_;
}

Object ImageProcessing::checkObject() {
  // Make sure we have image data
  if (color_image_data.empty()) {
    ROS_WARN("No image data available to check object");
    return Object::unknown;
  }

  // Extract central pixel values from raw RGB data
  int redValue = color_image_data[color_image_midpoint_];
  int greenValue = color_image_data[color_image_midpoint_ + 1];
  int blueValue = color_image_data[color_image_midpoint_ + 2];

  ROS_INFO("red: %d, green: %d, blue: %d", redValue, greenValue, blueValue);

  // Determine object type based on pixel values
  if (redValue < 50 && greenValue < 50 && blueValue < 50) {
    return Object::obstacle;
  } else if (greenValue > redValue && greenValue > blueValue) {
    return Object::nought;
  } else {
    return Object::cross;
  }
}

std::string ImageProcessing::checkShape() {
  // Make sure we have image data
  if (color_image_data.empty()) {
    ROS_WARN("No image data available to check shape");
    return "unknown";
  }

  // Extract central pixel values from raw RGB data
  int redValue = color_image_data[color_image_midpoint_];
  int greenValue = color_image_data[color_image_midpoint_ + 1];
  int blueValue = color_image_data[color_image_midpoint_ + 2];

  // Determine shape
  if (greenValue > redValue && greenValue > blueValue) {
    ROS_INFO("NOUGHT found");
    return "nought";
  } else {
    ROS_INFO("CROSS found");
    return "cross";
  }
}

} // namespace cw2
