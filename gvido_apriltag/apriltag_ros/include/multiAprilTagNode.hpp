// -----------------------------------------------------------------------------
// multiAprilTagNode.hpp
// -----------------------------------------------------------------------------
#pragma once

#include <memory>
#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

extern "C" {
#include <apriltag.h>
}

namespace apriltag_ros
{

class MultiAprilTagNode : public rclcpp::Node
{
public:
  explicit MultiAprilTagNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MultiAprilTagNode();

  void onCamera(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_ci,
    const std::string & camera_suffix,
    rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections);

private:
  static int idComparison(const void * first, const void * second);
  void removeDuplicates(zarray_t * detections_);

  void addObjectPoints(double s, cv::Matx44d T_oi,
                       std::vector<cv::Point3d> & objectPoints) const;

  void addImagePoints(apriltag_detection_t * detection,
                      std::vector<cv::Point2d> & imagePoints) const;

  Eigen::Matrix4d getRelativeTransform(
    std::vector<cv::Point3d> objectPoints,
    std::vector<cv::Point2d> imagePoints,
    double fx, double fy, double cx, double cy) const;

  geometry_msgs::msg::TransformStamped makeTagPose(
    const Eigen::Matrix4d & transform,
    Eigen::Quaternion<double> rot_quaternion,
    const std_msgs::msg::Header & header);

  // Члены данных
  image_transport::CameraSubscriber left_image_subscriber_;
  image_transport::CameraSubscriber right_image_subscriber_;

  rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr left_pub_detections_;
  rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr right_pub_detections_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  apriltag_detector_t * td_{nullptr};
  apriltag_family_t   * tf_{nullptr};

  std::string tag_family_;
  int   max_hamming_{0};
  bool  z_up_{true};
  double tag_edge_size_{0.162};
  bool  remove_duplicates_{true};

  std::map<int, double> tag_sizes_;
  std::map<int, std::string> tag_frames_;

  static const std::map<std::string, apriltag_family_t *(*)(void)> tag_create_;
  static const std::map<std::string, void (*)(apriltag_family_t *)> tag_destroy_;
};

}  // namespace apriltag_ros