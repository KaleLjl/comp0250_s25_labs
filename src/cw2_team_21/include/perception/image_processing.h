#ifndef IMAGE_PROCESSING_H_
#define IMAGE_PROCESSING_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>

namespace cw2 {

enum class Object {
  cross,
  nought,
  basket,
  obstacle,
  unknown
};

class ImageProcessing {
public:
  /**
   * @brief Constructor for the ImageProcessing class
   */
  ImageProcessing();

  /**
   * @brief Process incoming color image
   * @param msg Input image message
   */
  void processColorImage(const sensor_msgs::Image& msg);

  /**
   * @brief Check object type based on current image data
   * @return Object type (cross, nought, obstacle, etc.)
   */
  Object checkObject();

  /**
   * @brief Check shape type based on current image data
   * @return String describing the shape ("cross" or "nought")
   */
  std::string checkShape();

  /**
   * @brief Get the current image data
   * @return Reference to the image data vector
   */
  const std::vector<unsigned char>& getImageData() const { return color_image_data; }

  /**
   * @brief Get the image width
   * @return The image width in pixels
   */
  int getImageWidth() const { return color_image_width_; }

  /**
   * @brief Get the image height
   * @return The image height in pixels
   */
  int getImageHeight() const { return color_image_height_; }

  /**
   * @brief Get the image midpoint index
   * @return The index of the middle pixel in the image data
   */
  int getImageMidpoint() const { return color_image_midpoint_; }

private:
  /** \brief Camera data */
  std::vector<unsigned char> color_image_data;

  /** \brief Camera Resolution */
  int color_image_width_;
  int color_image_height_;

  /** \brief Pixel array midpoint */
  int color_image_midpoint_;

  /** \brief Number of colour channels (RGB) */
  const int color_channels_ = 3;

  /** \brief Setup image parameters (width, height, midpoint) */
  void setupImageParameters(int width, int height);
};

} // namespace cw2

#endif // IMAGE_PROCESSING_H_
