#ifndef __COLOR_TRACKER_H__
#define __COLOR_TRACKER_H__

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
  explicit ColorTracker(const rclcpp::NodeOptions& options);

 private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  cv::Mat extractChannel(const cv_bridge::CvImagePtr imgPtr);
  cv::Mat binaryze(const cv::Mat& singleChannel);
  void getComBbox(const cv::Mat& imgBinary, geometry_msgs::msg::Point& comMsg,
                  vision_msgs::msg::BoundingBox2D& bboxMsg);
  void showImage(const cv::Mat& imgBinary,
                 const geometry_msgs::msg::Point& comMsg,
                 const vision_msgs::msg::BoundingBox2D& bboxMsg);
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter>& params);

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

#endif  // __COLOR_TRACKER_H__