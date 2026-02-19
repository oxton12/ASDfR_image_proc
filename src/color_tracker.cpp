
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>

#include "image_tools/cv_mat_sensor_msgs_image_type_adapter.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace imgproc {

class ColorTracker : public rclcpp::Node {
 public:
  explicit ColorTracker(const rclcpp::NodeOptions& options)
      : Node("color_tracker", options) {
    imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image", 1,
        std::bind(&ColorTracker::imageCallback, this, std::placeholders::_1));

    trackedComPub_ =
        this->create_publisher<geometry_msgs::msg::Point>("tracked_CoM", 10);
    trackedBboxPub_ = this->create_publisher<vision_msgs::msg::BoundingBox2D>(
        "tracked_bbox", 10);

    channel_ = this->declare_parameter("channel", "k");
    if (channel_ != "b" && channel_ != "g" && channel_ != "r" &&
        channel_ != "k") {
      throw std::invalid_argument("Unexpected channel value " + channel_);
    }
    threshold_ = this->declare_parameter("threshold", 200);
    showImage_ = this->declare_parameter("show_image", false);

    callbackHandle_ = this->add_on_set_parameters_callback(std::bind(
        &ColorTracker::parametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Color_tracker node started.");
  }

 private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr imgPtr;

    try {
      imgPtr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat singleChannel = extractChannel(imgPtr);
    cv::Mat imgBinary = binaryze(singleChannel);

    geometry_msgs::msg::Point comMsg;
    vision_msgs::msg::BoundingBox2D bboxMsg;

    getComBbox(imgBinary, comMsg, bboxMsg);

    trackedComPub_->publish(comMsg);
    trackedBboxPub_->publish(bboxMsg);

    if (showImage_) {
      showImage(imgBinary, comMsg, bboxMsg);
    }
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto& param : params) {
      if (param.get_name() == "channel") {
        std::string newChannel = param.as_string();
        if (newChannel != "b" && newChannel != "g" && newChannel != "r" &&
            newChannel != "k") {
          result.successful = false;
          result.reason = "Unexpected channel value \"" + newChannel +
                          "\". Parameter is not changed.";
        } else
          channel_ = newChannel;
      } else if (param.get_name() == "threshold") {
        threshold_ = param.as_int();
      } else if (param.get_name() == "show_image") {
        showImage_ = param.as_bool();
      }
    }
    return result;
  }

  cv::Mat extractChannel(const cv_bridge::CvImagePtr imgPtr) {
    cv::Mat singleChannel;

    if (imgPtr->image.channels() == 1) {
      singleChannel = imgPtr->image;
    } else {
      switch (channel_[0]) {
        case 'k':
          cv::cvtColor(imgPtr->image, singleChannel, cv::COLOR_BGR2GRAY);
          break;

        case 'b': {
          std::vector<cv::Mat> channels;
          cv::split(imgPtr->image, channels);
          singleChannel = channels[0];
          break;
        }

        case 'g': {
          std::vector<cv::Mat> channels;
          cv::split(imgPtr->image, channels);
          singleChannel = channels[1];
          break;
        }

        case 'r': {
          std::vector<cv::Mat> channels;
          cv::split(imgPtr->image, channels);
          singleChannel = channels[2];
          break;
        }

        default:
          throw std::invalid_argument("Unexpected channel name " + channel_);
          break;
      }
    }

    return singleChannel;
  }

  cv::Mat binaryze(const cv::Mat& singleChannel) {
    cv::Mat imgBinary;
    cv::threshold(singleChannel, imgBinary, threshold_, 255, cv::THRESH_BINARY);

    cv::Mat imgClosed;
    cv::Mat imgOpened;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(imgBinary, imgClosed, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(imgClosed, imgOpened, cv::MORPH_OPEN, kernel);
    return imgOpened;
  }

  void getComBbox(const cv::Mat& imgBinary, geometry_msgs::msg::Point& comMsg,
                  vision_msgs::msg::BoundingBox2D& bboxMsg) {
    cv::Moments m = cv::moments(imgBinary, true);

    if (m.m00 == 0) {
      comMsg.x = -1;
      comMsg.y = -1;
      comMsg.z = 0;

      bboxMsg.center.position.x = -1;
      bboxMsg.center.position.y = -1;
      bboxMsg.size_x = -1;
      bboxMsg.size_y = -1;

      return;
    }

    double cx = m.m10 / m.m00;
    double cy = m.m01 / m.m00;

    comMsg.x = cx;
    comMsg.y = cy;
    comMsg.z = 0;

    cv::Rect bbox = cv::boundingRect(imgBinary);
    float bbox_cx = bbox.x + bbox.width / 2;
    float bbox_cy = bbox.y + bbox.height / 2;

    bboxMsg.center.position.x = bbox_cx;
    bboxMsg.center.position.y = bbox_cy;
    bboxMsg.size_x = bbox.width;
    bboxMsg.size_y = bbox.height;
  }

  void showImage(const cv::Mat& imgBinary,
                 const geometry_msgs::msg::Point& comMsg,
                 const vision_msgs::msg::BoundingBox2D& bboxMsg) {
    cv::Mat object_img_color;
    cv::cvtColor(imgBinary, object_img_color, cv::COLOR_GRAY2BGR);

    double cx = comMsg.x;
    double cy = comMsg.y;

    double x1 = bboxMsg.center.position.x - bboxMsg.size_x / 2;
    double y1 = bboxMsg.center.position.y - bboxMsg.size_y / 2;
    double x2 = bboxMsg.center.position.x + bboxMsg.size_x / 2;
    double y2 = bboxMsg.center.position.y + bboxMsg.size_y / 2;

    if (cx != -1) {
      cv::circle(object_img_color, cv::Point(cx, cy), 5, cv::Scalar(0, 0, 255),
                 cv::FILLED, cv::LINE_8);
      cv::rectangle(object_img_color, cv::Point(x1, y1), cv::Point(x2, y2),
                    cv::Scalar(255, 0, 0), 2);
    }
    cv::imshow("color_tracker", object_img_color);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr trackedComPub_;
  rclcpp::Publisher<vision_msgs::msg::BoundingBox2D>::SharedPtr trackedBboxPub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      callbackHandle_;

  std::string channel_;
  int threshold_;
  bool showImage_;
};

}  // namespace imgproc

RCLCPP_COMPONENTS_REGISTER_NODE(imgproc::ColorTracker)
