
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
  explicit Brightness(const rclcpp::NodeOptions& options)
      : Node("brightness", options) {
    threshold_ = this->declare_parameter<int>("threshold", 128);
    imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image", 1,
        std::bind(&Brightness::imageCallback, this, std::placeholders::_1));

    brightnessPub_ =
        this->create_publisher<std_msgs::msg::String>("bright estimate", 10);

    RCLCPP_INFO(this->get_logger(), "Brightness node started.");
  }

 private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr imgPtr;

    try {
      // Convert ROS image to OpenCV image
      imgPtr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat gray;

    // Convert to grayscale if needed
    if (imgPtr->image.channels() == 1) {
      gray = imgPtr->image;
    } else {
      cv::cvtColor(imgPtr->image, gray, cv::COLOR_BGR2GRAY);
    }

    cv::Scalar meanVal = cv::mean(gray);
    threshold_ = this->get_parameter("threshold").as_int();
    std_msgs::msg::String brightnessMsg;
    if (meanVal[0] > threshold_) {
      brightnessMsg.data = "Light (On)";
    } else {
      brightnessMsg.data = "Dark (Off)";
    }

    brightnessPub_->publish(brightnessMsg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr brightnessPub_;

  double threshold_;
};

}  // namespace imgproc

RCLCPP_COMPONENTS_REGISTER_NODE(imgproc::Brightness)
