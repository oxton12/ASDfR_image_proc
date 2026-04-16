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

  lowerColor_ =
      this->declare_parameter("lower_color", std::vector<long int>{40, 80, 40});
  upperColor_ = this->declare_parameter("upper_color",
                                        std::vector<long int>{95, 255, 255});
  showImage_ = this->declare_parameter("show_image", true);
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

  cv::Mat imgBinary =
      binaryze(imgPtr->image);  // image after threshold and cleanup

  cv::Mat flipped;
  cv::flip(imgBinary, flipped, -1);

  geometry_msgs::msg::Point comMsg;         // center of mass message
  vision_msgs::msg::BoundingBox2D bboxMsg;  // bounding box message

  getComBbox(flipped, comMsg, bboxMsg);

  trackedComPub_->publish(comMsg);
  trackedBboxPub_->publish(bboxMsg);

  if (showImage_) {
    showImage(flipped, comMsg, bboxMsg);
  }
}

rcl_interfaces::msg::SetParametersResult ColorTracker::parametersCallback(
    const std::vector<rclcpp::Parameter>& params) {
  rcl_interfaces::msg::SetParametersResult
      result;  // result of parameter change
  result.successful = true;
  for (const auto& param : params) {
    if (param.get_name() == "lower_green") {
      lowerColor_ = param.as_integer_array();
    } else if (param.get_name() == "upper_green") {
      upperColor_ = param.as_integer_array();
    } else if (param.get_name() == "show_image") {
      showImage_ = param.as_bool();
    }
  }
  return result;
}

cv::Mat ColorTracker::binaryze(const cv::Mat& inputImg) {
  cv::Mat hsvImg;     // HSV version of input image
  cv::Mat imgBinary;  // Binary mask after color thresholding

  cv::cvtColor(inputImg, hsvImg, cv::COLOR_BGR2HSV);  // Convert to HSV

  cv::Scalar lower(lowerColor_[0], lowerColor_[1],
                   lowerColor_[2]);  // lower HSV bound
  cv::Scalar upper(upperColor_[0], upperColor_[1],
                   upperColor_[2]);  // upper HSV bound

  cv::inRange(hsvImg, lower, upper,
              imgBinary);  // Threshold image to extract green regions

  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(5, 5));  // Morphological kernel for noise removal

  cv::Mat imgOpened;  // Image after opening (remove small noise)
  cv::Mat imgClosed;  // Image after closing (fill holes)

  cv::morphologyEx(imgBinary, imgOpened, cv::MORPH_OPEN,
                   kernel);  // Remove small noise
  cv::morphologyEx(imgOpened, imgClosed, cv::MORPH_CLOSE,
                   kernel);  // Close gaps inside detected object

  return imgClosed;
}

void ColorTracker::getComBbox(const cv::Mat& imgBinary,
                              geometry_msgs::msg::Point& comMsg,
                              vision_msgs::msg::BoundingBox2D& bboxMsg) {
  std::vector<std::vector<cv::Point>> contours;  // all detected contours

  cv::findContours(imgBinary, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  // If no contours found, return invalid values
  if (contours.empty()) {
    comMsg.x = -1;
    comMsg.y = -1;
    comMsg.z = 0;

    bboxMsg.center.position.x = -1;
    bboxMsg.center.position.y = -1;
    bboxMsg.size_x = -1;
    bboxMsg.size_y = -1;
    return;
  }

  double maxArea = 0.0;  // largest contour area
  int largestIdx = -1;   // index of largest contour

  for (size_t i = 0; i < contours.size(); i++) {
    double area = cv::contourArea(contours[i]);  // area of current contour
    if (area > maxArea) {
      maxArea = area;
      largestIdx = i;
    }
  }

  cv::Moments m = cv::moments(contours[largestIdx]);  // moments of largest blob

  // If area is zero, return invalid
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

  double cx = m.m10 / m.m00;  // x coordinate of CoM
  double cy = m.m01 / m.m00;  // y coordinate of CoM

  comMsg.x = cx;
  comMsg.y = cy;
  comMsg.z = 0;

  cv::Rect bbox = cv::boundingRect(contours[largestIdx]);  // bounding box

  float bbox_cx = bbox.x + bbox.width / 2.0f;   // bbox center x
  float bbox_cy = bbox.y + bbox.height / 2.0f;  // bbox center y

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

}  // namespace imgproc

RCLCPP_COMPONENTS_REGISTER_NODE(imgproc::ColorTracker)
