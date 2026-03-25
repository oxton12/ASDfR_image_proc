//==============================================================================
// Authors : Max Solis, Aleksei Obshatko
// Group : ASDfR 5
// License : LGPL open source license
//
// Brief : Node that estimates brightness of an image received over the "image"
// topic and publishes the result in the "brightness estimate" topic
//==============================================================================

#include <brightness.hpp>

namespace imgproc {

Brightness::Brightness(const rclcpp::NodeOptions& options)
    : Node("brightness", options) {
  threshold_ = this->declare_parameter<int>("threshold", 128);
  imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image", 1,
      std::bind(&Brightness::imageCallback, this, std::placeholders::_1));

  brightnessPub_ =
      this->create_publisher<std_msgs::msg::String>("brightness_estimate", 10);

  RCLCPP_INFO(this->get_logger(), "Brightness node started.");
}

void Brightness::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  cv_bridge::CvImagePtr imgPtr;  // pointer to the received image

  // Convert ROS image to OpenCV image
  try {
    imgPtr = cv_bridge::toCvCopy(msg, msg->encoding);
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Convert to grayscale if not
  cv::Mat gray;  // graycaled image
  if (imgPtr->image.channels() == 1) {
    gray = imgPtr->image;
  } else {
    cv::cvtColor(imgPtr->image, gray, cv::COLOR_BGR2GRAY);
  }

  cv::Scalar meanVal = cv::mean(gray);  // mean brightness of the image

  threshold_ =
      this->get_parameter("threshold").as_int();  // update threshold parameter

  std_msgs::msg::String brightnessMsg;  // brightness decision message
  if (meanVal[0] > threshold_) {
    brightnessMsg.data = "Light (On)";
  } else {
    brightnessMsg.data = "Dark (Off)";
  }

  brightnessPub_->publish(brightnessMsg);
}

}  // namespace imgproc

RCLCPP_COMPONENTS_REGISTER_NODE(imgproc::Brightness)
