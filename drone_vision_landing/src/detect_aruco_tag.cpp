#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cstddef>
#include <memory>
#include <chrono>
#include <functional>
#include <cmath>
#include <opencv2/core/types.hpp>
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
#include <Eigen/Geometry>

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
      // FIXME: Remove this debug
      // cv::imshow("Camera OpenCv", _raw_img);
      // cv::waitKey(1);

      // Detect Markers with aruco lib
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(_raw_img, _dictionary, corners, ids);

      cv::Mat debug_img = _raw_img.clone();
      if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(debug_img, corners, ids);
        cv::drawFrameAxes(debug_img, _camMatrix, _distCoeffs, rvec, tvec, _marker_size_m * 0.5f);
      }
      cv::imshow("ArUco Debug", debug_img);
      cv::waitKey(1);

      // Find the _target_marker_id
      for (size_t i=0; i<ids.size(); ++i) {
          // Assuming the marker is center at (0,0,0)
          float s = _marker_size_m / 2.0f;
          std::vector<cv::Point3f> object_points = {
            {-s,  s, 0},
            { s,  s, 0},
            { s, -s, 0},
            {-s, -s, 0}
          };

        // If found
        if (ids[i] == _target_marker_id) {
          RCLCPP_INFO(this->get_logger(), "Target marker detected: %d", _target_marker_id);
          // SolvePnP: rvec->[marker orientation relative to camera] | tvec->[marker position relative to camera]
          cv::solvePnP(object_points, corners[i], _camMatrix,
              _distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE
            );

          // Fill in the PoseStamped
          // Convert rvec to quat
          cv::Mat R;
          cv::Rodrigues(rvec, R);

          // OpenCv -> Eigen rotation matrix
          Eigen::Matrix3d rotation;
          for (int row=0; row<3; ++row) {
            for (int col = 0; col < 3; ++col) {
              rotation(row, col) = R.at<double>(row, col);
            }
          }

          Eigen::Quaterniond q(rotation);
          q.normalize();

          geometry_msgs::msg::PoseStamped pose;

          pose.header.stamp = msg->header.stamp;
          pose.header.frame_id = msg->header.frame_id;

          // Position
          pose.pose.position.x = tvec[0];
          pose.pose.position.y = tvec[1];
          pose.pose.position.z = tvec[2];

          // Orientation
          pose.pose.orientation.x = q.x();
          pose.pose.orientation.y = q.y();
          pose.pose.orientation.z = q.z();
          pose.pose.orientation.w = q.w();

          // Publish
          _tag_pose_pub->publish(pose);
        }
      }
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
  cv::Vec3d rvec, tvec;

};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
