
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>

namespace imgproc {

class Img_Proc : public rclcpp::Node {
 public:
  explicit Img_Proc(const rclcpp::NodeOptions& options)
      : Node("brightness_node", options) {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image", 1,
        std::bind(&Img_Proc::imageCallback, this, std::placeholders::_1));

    brightness_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "average_brightness", 10);

    RCLCPP_INFO(this->get_logger(), "Brightness node started.");
  }

 private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
      // Convert ROS image to OpenCV image
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat gray;

    // Convert to grayscale if needed
    if (cv_ptr->image.channels() == 1) {
      gray = cv_ptr->image;
    } else {
      cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    }

    // Compute mean brightness
    cv::Scalar mean_val = cv::mean(gray);

    std_msgs::msg::Float32 brightness_msg;
    brightness_msg.data = static_cast<float>(mean_val[0]);

    brightness_pub_->publish(brightness_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brightness_pub_;
};

}  // namespace imgproc

RCLCPP_COMPONENTS_REGISTER_NODE(imgproc::Img_Proc)
