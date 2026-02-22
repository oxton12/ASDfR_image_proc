//==============================================================================
// Authors : Max Solis, Aleksei Obshatko
// Group : ASDfR 5
// License : LGPL open source license
//
// Brief : Node that estimates brightness of an image received over the "image"
// topic and publishes the result in the "brightness estimate" topic
//==============================================================================

#ifndef __BRIGHTNESS_H__
#define __BRIGHTNESS_H__

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

/// @brief brightness node constructor
/// @param options node options
class Brightness : public rclcpp::Node {
 public:
  explicit Brightness(const rclcpp::NodeOptions& options);

 private:
  /// @brief Evsluates brightness of received image and decides whether its
  /// bright
  /// or not
  /// @param msg Message containing an image
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      imageSub_;  // subscription to receive input images
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      brightnessPub_;  // publiher of the brightness estimation

  // ROS parameters
  double threshold_;  // threshold for brightness decision
};

#endif  // __BRIGHTNESS_H__