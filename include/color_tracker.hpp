//==============================================================================
// Authors : Max Solis, Aleksei Obshatko
// Group : ASDfR 5
// License : LGPL open source license
//
// Brief : Node that detects the brightest regions in a specific channel an
// image received over the "image" topic and publishes it's center of mass in
// the topic "tracked_CoM" and it's bounding box in topic "tracked_bbox"
//==============================================================================

#ifndef __COLOR_TRACKER_H__
#define __COLOR_TRACKER_H__

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>
#include <vision_msgs/msg/bounding_box2_d.hpp>

#include "image_tools/cv_mat_sensor_msgs_image_type_adapter.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

class ColorTracker : public rclcpp::Node {
 public:
  /// @brief color_tracker node constructor
  /// @param options node options
  explicit ColorTracker(const rclcpp::NodeOptions& options);

 private:
  /// @brief Processes received image, detects object and publishes it's CoM and
  /// bounding box
  /// @param msg Message containing an image
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  /// @brief Extracts the desired channel from image
  /// @param imgPtr Input image
  /// @return An image which is a copy of the desired channel
  cv::Mat extractChannel(const cv_bridge::CvImagePtr imgPtr);

  /// @brief Applies threshold, removes noise and fills gaps
  /// @param inputImg Input image
  /// @return Image after threshold and cleanup
  cv::Mat binaryze(const cv::Mat& singleChannel);

  /// @brief Calculates the center of mass and a bounding box of all white
  /// pixels
  /// and saves it as messages
  /// @param imgBinary Binary input image
  /// @param comMsg Center of Mass message
  /// @param bboxMsg Bounding box message
  void getComBbox(const cv::Mat& imgBinary, geometry_msgs::msg::Point& comMsg,
                  vision_msgs::msg::BoundingBox2D& bboxMsg);

  /// @brief Shows image with indicated CoM and bounding box
  /// @param imgBinary Image to show
  /// @param comMsg Center of Mass message
  /// @param bboxMsg Bounding box message
  void showImage(const cv::Mat& imgBinary,
                 const geometry_msgs::msg::Point& comMsg,
                 const vision_msgs::msg::BoundingBox2D& bboxMsg);

  /// @brief Checks whether string value representing channel to extract is
  /// valid
  /// @param channel
  /// @return True if valid, false othrwise
  bool checkChannel(const std::string channel);

  /// @brief Checks and applies received parameters
  /// @param params New parameters
  /// @return Result of parameter check
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter>& params);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      imageSub_;  // subscription to receive input images
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr
      trackedComPub_;  // publisher of the center of mass of the brightest
                       // object in an image
  rclcpp::Publisher<vision_msgs::msg::BoundingBox2D>::SharedPtr
      trackedBboxPub_;  // publisher of the bounding box of the brightest
                        // object in an image
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      callbackHandle_;  // callback to change the parameters at runtime

  // ros parameters
  std::string channel_;  // chosen channel to look for the brightest object
  int threshold_;        // the value of a pixel below which it will be set to 0
  bool showImage_;  // whether to show the image after threshold with CoM and
                    // bbox of the detected object

  const std::vector<std::string> validChannels_{
      "r", "g", "b", "k"};  // all valid values of the channel parameter
};

#endif  // __COLOR_TRACKER_H__