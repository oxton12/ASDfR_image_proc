//==============================================================================
// Authors : Max Solis, Aleksei Obshatko
// Group : ASDfR 5
// License : LGPL open source license
//
// Brief : Node that detects the brightest regions in a specific channel an
// image received over the "image" topic and publishes it's center of mass in
// the topic "tracked_CoM" and it's bounding box in topic "tracked_bbox"
//==============================================================================

#include <color_tracker.hpp>

namespace imgproc {

ColorTracker::ColorTracker(const rclcpp::NodeOptions& options)
    : Node("color_tracker", options) {
  imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image", 1,
      std::bind(&ColorTracker::imageCallback, this, std::placeholders::_1));

  trackedComPub_ =
      this->create_publisher<geometry_msgs::msg::Point>("tracked_CoM", 10);
  trackedBboxPub_ = this->create_publisher<vision_msgs::msg::BoundingBox2D>(
      "tracked_bbox", 10);

  channel_ = this->declare_parameter("channel", "k");
  if (!checkChannel(channel_)) {
    throw std::invalid_argument("Unexpected channel value " + channel_);
  }
  threshold_ = this->declare_parameter("threshold", 200);
  showImage_ = this->declare_parameter("show_image", false);

  callbackHandle_ = this->add_on_set_parameters_callback(std::bind(
      &ColorTracker::parametersCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Color_tracker node started.");
}

void ColorTracker::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  cv_bridge::CvImagePtr imgPtr;  // pointer to the received image

  // Convert ROS image to OpenCV image
  try {
    imgPtr = cv_bridge::toCvCopy(msg, msg->encoding);
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat singleChannel =
      extractChannel(imgPtr);  // a copy of the chosen image channel
  cv::Mat imgBinary =
      binaryze(singleChannel);  // image after threshold and cleanup

  geometry_msgs::msg::Point comMsg;         // center of mass message
  vision_msgs::msg::BoundingBox2D bboxMsg;  // bounding box message

  getComBbox(imgBinary, comMsg, bboxMsg);

  trackedComPub_->publish(comMsg);
  trackedBboxPub_->publish(bboxMsg);

  if (showImage_) {
    showImage(imgBinary, comMsg, bboxMsg);
  }
}

rcl_interfaces::msg::SetParametersResult ColorTracker::parametersCallback(
    const std::vector<rclcpp::Parameter>& params) {
  rcl_interfaces::msg::SetParametersResult
      result;  // result of parameter change
  result.successful = true;
  for (const auto& param : params) {
    if (param.get_name() == "channel") {
      std::string newChannel = param.as_string();  // received channel value
      if (!checkChannel(newChannel)) {
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

cv::Mat ColorTracker::extractChannel(const cv_bridge::CvImagePtr imgPtr) {
  cv::Mat singleChannel;

  // Return input if it is single channel
  if (imgPtr->image.channels() == 1) {
    return imgPtr->image;
  }

  switch (channel_[0]) {
    case 'k':
      cv::cvtColor(imgPtr->image, singleChannel, cv::COLOR_BGR2GRAY);
      break;

    case 'b': {
      std::vector<cv::Mat> channels;  // vector of channels
      cv::split(imgPtr->image, channels);
      singleChannel = channels[0];
      break;
    }

    case 'g': {
      std::vector<cv::Mat> channels;  // vector of channels
      cv::split(imgPtr->image, channels);
      singleChannel = channels[1];
      break;
    }

    case 'r': {
      std::vector<cv::Mat> channels;  // vector of channels
      cv::split(imgPtr->image, channels);
      singleChannel = channels[2];
      break;
    }

    default:
      throw std::invalid_argument("Unexpected channel name " + channel_);
      break;
  }

  return singleChannel;
}

cv::Mat ColorTracker::binaryze(const cv::Mat& inputImg) {
  cv::Mat imgBinary;  // channel after threshold
  cv::threshold(inputImg, imgBinary, threshold_, 255, cv::THRESH_BINARY);

  cv::Mat imgOpened;  // channel after opening
  cv::Mat imgClosed;  // channel after closing
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(imgBinary, imgOpened, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(imgOpened, imgClosed, cv::MORPH_CLOSE, kernel);
  return imgClosed;
}

void ColorTracker::getComBbox(const cv::Mat& imgBinary,
                              geometry_msgs::msg::Point& comMsg,
                              vision_msgs::msg::BoundingBox2D& bboxMsg) {
  cv::Moments m =
      cv::moments(imgBinary, true);  // moments of the white pixels on the image

  // If area is 0, set all values to invalid and return
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

  double cx = m.m10 / m.m00;  // x coordinate of the CoM
  double cy = m.m01 / m.m00;  // y coordinate of the CoM

  comMsg.x = cx;
  comMsg.y = cy;
  comMsg.z = 0;

  cv::Rect bbox = cv::boundingRect(imgBinary);
  float bbox_cx =
      bbox.x + bbox.width / 2;  // x coordinate of the bounding box center
  float bbox_cy =
      bbox.y + bbox.height / 2;  // y coordinate of the bounding box center

  bboxMsg.center.position.x = bbox_cx;
  bboxMsg.center.position.y = bbox_cy;
  bboxMsg.size_x = bbox.width;
  bboxMsg.size_y = bbox.height;
}

void ColorTracker::showImage(const cv::Mat& imgBinary,
                             const geometry_msgs::msg::Point& comMsg,
                             const vision_msgs::msg::BoundingBox2D& bboxMsg) {
  cv::Mat object_img_color;  // Input binary image in color format
  cv::cvtColor(imgBinary, object_img_color, cv::COLOR_GRAY2BGR);

  double cx = comMsg.x;  // x coordinate of the CoM
  double cy = comMsg.y;  // y coordinate of the CoM

  double x1 =
      bboxMsg.center.position.x - bboxMsg.size_x / 2;  // bbox top-left x
  double y1 =
      bboxMsg.center.position.y - bboxMsg.size_y / 2;  // bbox top-left y
  double x2 =
      bboxMsg.center.position.x + bboxMsg.size_x / 2;  // bbox bottom-right x
  double y2 =
      bboxMsg.center.position.y + bboxMsg.size_y / 2;  // bbox bottom-right y

  if (cx != -1) {  // If object area is not 0
    cv::circle(object_img_color, cv::Point(cx, cy), 5, cv::Scalar(0, 0, 255),
               cv::FILLED, cv::LINE_8);
    cv::rectangle(object_img_color, cv::Point(x1, y1), cv::Point(x2, y2),
                  cv::Scalar(255, 0, 0), 2);
  }
  cv::imshow("color_tracker", object_img_color);
  cv::waitKey(1);
}

bool ColorTracker::checkChannel(const std::string channel) {
  return std::find(validChannels_.begin(), validChannels_.end(), channel) !=
         validChannels_.end();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorTracker>());
  rclcpp::shutdown();
  return 0;
}

}  // namespace imgproc
