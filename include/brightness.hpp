#ifndef __BRIGHTNESS_H__
#define __BRIGHTNESS_H__

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

namespace imgproc {

class Brightness : public rclcpp::Node {
 public:
  explicit Brightness(const rclcpp::NodeOptions& options);

 private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr brightnessPub_;

  double threshold_;
};
}  // namespace imgproc

#endif  // __BRIGHTNESS_H__