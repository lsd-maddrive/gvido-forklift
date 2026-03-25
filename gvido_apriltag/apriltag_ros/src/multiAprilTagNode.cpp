// -----------------------------------------------------------------------------
// multiAprilTagNode.cpp — Multi AprilTag для стерео-камер (Jazzy compatible)
// -----------------------------------------------------------------------------

#include "multiAprilTagNode.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/calib3d.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <chrono>

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
  #include <tf2_eigen/tf2_eigen.hpp>
#else
  #include <tf2_eigen/tf2_eigen.h>
#endif

#include <Eigen/Geometry>

extern "C" {
#include <apriltag.h>
#include <tag16h5.h>
#include <tag25h9.h>
#include <tag36h10.h>
#include <tag36h11.h>
#include <tagCircle21h7.h>
#include <tagCircle49h12.h>
#include <tagCustom48h12.h>
#include <tagStandard41h12.h>
#include <tagStandard52h13.h>
#include <common/homography.h>
}

#define TAG_CREATE(name)  {#name, tag ## name ## _create},
#define TAG_DESTROY(name) {#name, tag ## name ## _destroy},

namespace apriltag_ros
{

// Статические карты семейств
const std::map<std::string, apriltag_family_t *(*)(void)> MultiAprilTagNode::tag_create_ = {
  TAG_CREATE(16h5)
  TAG_CREATE(25h9)
  TAG_CREATE(36h10)
  TAG_CREATE(36h11)
  TAG_CREATE(Circle21h7)
  TAG_CREATE(Circle49h12)
  TAG_CREATE(Custom48h12)
  TAG_CREATE(Standard41h12)
  TAG_CREATE(Standard52h13)
};

const std::map<std::string, void (*)(apriltag_family_t *)> MultiAprilTagNode::tag_destroy_ = {
  TAG_DESTROY(16h5)
  TAG_DESTROY(25h9)
  TAG_DESTROY(36h10)
  TAG_DESTROY(36h11)
  TAG_DESTROY(Circle21h7)
  TAG_DESTROY(Circle49h12)
  TAG_DESTROY(Custom48h12)
  TAG_DESTROY(Standard41h12)
  TAG_DESTROY(Standard52h13)
};

// ====================== КОНСТРУКТОР ======================
MultiAprilTagNode::MultiAprilTagNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("multi_apriltag", options)
{
  RCLCPP_INFO(get_logger(), "MultiAprilTagNode стартует...");

  tag_family_        = declare_parameter<std::string>("family", "36h11");
  max_hamming_       = declare_parameter<int>("max_hamming", 0);
  z_up_              = declare_parameter<bool>("z_up", true);
  tag_edge_size_     = declare_parameter<double>("tag_edge_size", 0.162);
  remove_duplicates_ = declare_parameter<bool>("remove_duplicates", true);

  RCLCPP_INFO(get_logger(), "Параметры: family=%s, max_hamming=%d, z_up=%s, edge=%.3f",
              tag_family_.c_str(), max_hamming_, z_up_ ? "true" : "false", tag_edge_size_);

  if (!tag_create_.count(tag_family_)) {
    RCLCPP_WARN(get_logger(), "Семейство '%s' не поддерживается → 36h11", tag_family_.c_str());
    tag_family_ = "36h11";
  }

  tf_ = tag_create_.at(tag_family_)();
  td_ = apriltag_detector_create();
  apriltag_detector_add_family(td_, tf_);

  td_->quad_decimate = declare_parameter<float>("decimate", 1.0f);
  td_->quad_sigma    = declare_parameter<float>("blur", 0.0f);
  td_->nthreads      = declare_parameter<int>("threads", 4);
  td_->debug         = declare_parameter<int>("debug", 0);
  td_->refine_edges  = declare_parameter<int>("refine-edges", 1);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  left_pub_detections_  = create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
    "apriltag_detections_left", rclcpp::QoS(10));
  right_pub_detections_ = create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
    "apriltag_detections_right", rclcpp::QoS(10));

  std::string left_cam   = declare_parameter<std::string>("left_camera_name", "camera_left");
  std::string right_cam  = declare_parameter<std::string>("right_camera_name", "camera_right");
  std::string img_topic  = declare_parameter<std::string>("image_topic", "image_raw");
  std::string transport  = declare_parameter<std::string>("image_transport", "raw");

  std::string left_base  = (left_cam[0] == '/' ? left_cam : "/" + left_cam) + "/" + img_topic;
  std::string right_base = (right_cam[0] == '/' ? right_cam : "/" + right_cam) + "/" + img_topic;

  left_image_subscriber_ = image_transport::create_camera_subscription(
    this, left_base,
    std::bind(&MultiAprilTagNode::onCamera, this,
              std::placeholders::_1, std::placeholders::_2, "_L", left_pub_detections_),
    transport);

  right_image_subscriber_ = image_transport::create_camera_subscription(
    this, right_base,
    std::bind(&MultiAprilTagNode::onCamera, this,
              std::placeholders::_1, std::placeholders::_2, "_R", right_pub_detections_),
    transport);

  RCLCPP_INFO(get_logger(), "MultiAprilTagNode готов: left=%s | right=%s", left_base.c_str(), right_base.c_str());
}

// ====================== ДЕСТРУКТОР ======================
MultiAprilTagNode::~MultiAprilTagNode()
{
  RCLCPP_INFO(get_logger(), "MultiAprilTagNode завершает работу");
  if (td_) { apriltag_detector_destroy(td_); td_ = nullptr; }
  if (tf_) { tag_destroy_.at(tag_family_)(tf_); tf_ = nullptr; }
}

// ====================== ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ ======================
int MultiAprilTagNode::idComparison(const void * first, const void * second)
{
  int a = (*static_cast<const apriltag_detection_t * const *>(first))->id;
  int b = (*static_cast<const apriltag_detection_t * const *>(second))->id;
  return (a > b) - (a < b);
}

void MultiAprilTagNode::removeDuplicates(zarray_t * detections_)
{
  zarray_sort(detections_, &MultiAprilTagNode::idComparison);
  int count = 0;
  bool dup = false;
  while (true) {
    if (count > zarray_size(detections_) - 1) return;
    apriltag_detection_t * det;
    zarray_get(detections_, count, &det);
    int id_curr = det->id;
    int id_next = -1;
    if (count < zarray_size(detections_) - 1) {
      zarray_get(detections_, count + 1, &det);
      id_next = det->id;
    }
    if (id_curr == id_next || (id_curr != id_next && dup)) {
      zarray_remove_index(detections_, count, 1);
      dup = true;
    } else {
      ++count;
      dup = false;
    }
  }
}

void MultiAprilTagNode::addObjectPoints(double s, cv::Matx44d T_oi,
                                        std::vector<cv::Point3d> & objectPoints) const
{
  objectPoints.emplace_back(T_oi(0,3) - s, T_oi(1,3) - s, T_oi(2,3));
  objectPoints.emplace_back(T_oi(0,3) + s, T_oi(1,3) - s, T_oi(2,3));
  objectPoints.emplace_back(T_oi(0,3) + s, T_oi(1,3) + s, T_oi(2,3));
  objectPoints.emplace_back(T_oi(0,3) - s, T_oi(1,3) + s, T_oi(2,3));
}

void MultiAprilTagNode::addImagePoints(apriltag_detection_t * detection,
                                       std::vector<cv::Point2d> & imagePoints) const
{
  double tag_x[4] = {-1, 1, 1, -1};
  double tag_y[4] = { 1, 1, -1, -1};
  double im_x, im_y;
  for (int i = 0; i < 4; ++i) {
    homography_project(detection->H, tag_x[i], tag_y[i], &im_x, &im_y);
    imagePoints.emplace_back(im_x, im_y);
  }
}

void MultiAprilTagNode::onCamera(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_img,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_ci,
  const std::string & camera_suffix,
  rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections)
{
  const auto start = std::chrono::steady_clock::now();

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::MONO8);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
    return;
  }

  image_u8_t im{ cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.cols, cv_ptr->image.data };

  const auto & K = msg_ci->k;
  double fx = K[0], fy = K[4], cx = K[2], cy = K[5];

  zarray_t * detections = apriltag_detector_detect(td_, &im);
  const int raw_cnt = zarray_size(detections);

  if (remove_duplicates_) removeDuplicates(detections);

  const int final_cnt = zarray_size(detections);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "Камера%s: %d тегов (сырых %d)", camera_suffix.c_str(), final_cnt, raw_cnt);

  apriltag_msgs::msg::AprilTagDetectionArray msg_array;
  msg_array.header = msg_img->header;

  for (int i = 0; i < final_cnt; ++i) {
    apriltag_detection_t * det;
    zarray_get(detections, i, &det);

    if (det->hamming > max_hamming_) continue;

    double half = tag_edge_size_ / 2.0;

    std::vector<cv::Point3d> obj_pts;
    addObjectPoints(half, cv::Matx44d::eye(), obj_pts);

    std::vector<cv::Point2d> img_pts;
    addImagePoints(det, img_pts);

    Eigen::Matrix4d T = getRelativeTransform(obj_pts, img_pts, fx, fy, cx, cy);
    Eigen::Quaternion<double> q(T.block<3,3>(0,0));

    geometry_msgs::msg::TransformStamped tag_pose = makeTagPose(T, q, msg_img->header);
    std::string base_name = std::string(det->family->name) + ":" + std::to_string(det->id);
    tag_pose.child_frame_id = base_name + camera_suffix;

    tf_broadcaster_->sendTransform(tag_pose);

    apriltag_msgs::msg::AprilTagDetection det_msg;
    det_msg.family = det->family->name;
    det_msg.pose.pose.pose.position.x = T(0,3);
    det_msg.pose.pose.pose.position.y = T(1,3);
    det_msg.pose.pose.pose.position.z = T(2,3);
    det_msg.pose.pose.pose.orientation = tag_pose.transform.rotation;
    det_msg.id = det->id;
    det_msg.size = half * 2.0;

    msg_array.detections.push_back(det_msg);
  }

  pub_detections->publish(msg_array);
  apriltag_detections_destroy(detections);

  auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start).count();
  RCLCPP_DEBUG(get_logger(), "Камера%s обработана за %ld ms", camera_suffix.c_str(), dt);
}

Eigen::Matrix4d MultiAprilTagNode::getRelativeTransform(
  std::vector<cv::Point3d> objectPoints,
  std::vector<cv::Point2d> imagePoints,
  double fx, double fy, double cx, double cy) const
{
  cv::Mat rvec, tvec;
  cv::Matx33d K(fx, 0, cx, 0, fy, cy, 0, 0, 1);
  cv::Vec4f D(0,0,0,0);

  cv::solvePnP(objectPoints, imagePoints, K, D, rvec, tvec);
  cv::Matx33d R_cv;
  cv::Rodrigues(rvec, R_cv);

  Eigen::Matrix3d R_e;
  if (z_up_) {
    R_e << R_cv(1,0), R_cv(1,1), R_cv(1,2),
           R_cv(2,0), R_cv(2,1), R_cv(2,2),
           R_cv(0,0), R_cv(0,1), R_cv(0,2);
    Eigen::AngleAxisd rot_x(M_PI / 2.0, Eigen::Vector3d::UnitX());
    R_e = R_e * rot_x.matrix();
  } else {
    R_e = Eigen::Map<const Eigen::Matrix3d>(R_cv.val);
  }

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = R_e;
  T.block<3,1>(0,3) = Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
  return T;
}

geometry_msgs::msg::TransformStamped MultiAprilTagNode::makeTagPose(
  const Eigen::Matrix4d & transform,
  Eigen::Quaternion<double> rot_quaternion,
  const std_msgs::msg::Header & header)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header = header;
  tf_msg.transform.translation.x = transform(0,3);
  tf_msg.transform.translation.y = transform(1,3);
  tf_msg.transform.translation.z = transform(2,3);

  if (z_up_) {
    tf_msg.transform.rotation.x = rot_quaternion.z();
    tf_msg.transform.rotation.y = rot_quaternion.x();
    tf_msg.transform.rotation.z = rot_quaternion.y();
  } else {
    tf_msg.transform.rotation.x = rot_quaternion.x();
    tf_msg.transform.rotation.y = rot_quaternion.y();
    tf_msg.transform.rotation.z = rot_quaternion.z();
  }
  tf_msg.transform.rotation.w = rot_quaternion.w();

return tf_msg;
}

} 
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_ros::MultiAprilTagNode)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<apriltag_ros::MultiAprilTagNode>());
  rclcpp::shutdown();
  return 0;
}