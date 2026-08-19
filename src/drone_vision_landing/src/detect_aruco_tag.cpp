#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/bool.hpp"
#include <memory>
#include <chrono>
#include <functional>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>

using std::placeholders::_1;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

// ── Aruco dictionary parser ──────────────────────────
cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_from_name(const std::string& name)
{
  if (name == "DICT_4X4_50")
    return cv::aruco::DICT_4X4_50;

  if (name == "DICT_4X4_100")
    return cv::aruco::DICT_4X4_100;

  if (name == "DICT_5X5_50")
    return cv::aruco::DICT_5X5_50;

  if (name == "DICT_6X6_50")
    return cv::aruco::DICT_6X6_50;

  throw std::invalid_argument("Unknown ArUco dictionary: " + name);
}


// ─────────────────────────────────────────────────────────────────────────────
// Node
// ─────────────────────────────────────────────────────────────────────────────
class ArucoDetectorNode : public rclcpp::Node 
{
public:
  ArucoDetectorNode() : Node("aruco_detector_node")
  {
    // ── Parameters  ────────────────────────
    this->declare_parameter<int>("target_marker_id", 1);
    this->declare_parameter<double>("marker_size_m", 1.5);
    this->declare_parameter<std::string>("dictionary_name", "DICT_4X4_50");

    // ── Read params ────────────────────────
    _target_marker_id = this->get_parameter("target_marker_id").as_int();
    _marker_size_m = this->get_parameter("marker_size_m").as_double();
    _dict_name = this->get_parameter("dictionary_name").as_string();

    // ── Aruco Init  ────────────────────────
    _dictionary = cv::aruco::getPredefinedDictionary(dictionary_from_name(_dict_name));

    // ── Publishers ─────────────────────────
    _tag_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/tag/pose", 10);
    _is_detected_pub = this->create_publisher<std_msgs::msg::Bool>("/tag/detected", 10);

    // ── Subscribers ────────────────────────
    _img_sub = this->create_subscription<sensor_msgs::msg::Image>("/x500_mono_cam_down_0/downfaced_camera/image_raw", rclcpp::SensorDataQoS(), std::bind(&ArucoDetectorNode::image_callback, this, _1));

    _cam_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/x500_mono_cam_down_0/downfaced_camera/camera_info", rclcpp::SensorDataQoS(), std::bind(&ArucoDetectorNode::camera_info_callback, this, _1));

  }

private:
  // ─────────────────────────────────────────────────────────────────────────
  // Callbacks
  // ─────────────────────────────────────────────────────────────────────────
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    // Fill it only once
    if (_hasCamInfo) {
      return;
    }

    // Fill in width and height 
    width = static_cast<int>(msg->width);
    height = static_cast<int>(msg->height);

    // Fill in intrinsics matrix
    _camMatrix = (cv::Mat_<double>(3, 3) << 
        msg->k[0], msg->k[1], msg->k[2],
        msg->k[3], msg->k[4], msg->k[5],
        msg->k[6], msg->k[7], msg->k[8]
      );

    // Fill in distortion matrix
    _distCoeffs = msg->d;

    _hasCamInfo = true;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Sanity check: if there is no camera info return
    if (!_hasCamInfo) {
      RCLCPP_WARN(this->get_logger(), "No valid camera info!");
      return;
    }

    try {
      // Convert Image into a CV data type
      _raw_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000, 
          "Image size: %dx%d", 
          _raw_img.size().width, 
          _raw_img.size().height
        );
      cv::imshow("Camera OpenCv", _raw_img);
      cv::waitKey(1);
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    }




  }

  // ─────────────────────────────────────────────────────────────────────────
  // Members
  // ─────────────────────────────────────────────────────────────────────────

  // ── Subscribers ────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _img_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _cam_info_sub;

  // ── Publisher ──────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _tag_pose_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _is_detected_pub;

  // ── Camera Props ──────────────────────────
  cv::Mat _raw_img;
  int width{0};
  int height{0};
  cv::Mat _camMatrix;
  std::vector<double> _distCoeffs;
  bool _hasCamInfo{false};

  // ── Aruco ──────────────────────────
  cv::Ptr<cv::aruco::Dictionary> _dictionary;
  int _target_marker_id{1};
  double _marker_size_m{1.5};
  std::string _dict_name;

};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
