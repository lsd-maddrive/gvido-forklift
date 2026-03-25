// -----------------------------------------------------------------------------
// AprilTagNode.hpp — объявление ROS 2‑узла для обнаружения маркеров AprilTag
// -----------------------------------------------------------------------------
// Файл содержит объявление класса AprilTagNode, который наследует rclcpp::Node
// и реализует полный цикл: получение изображений камеры, детектирование тегов,
// оценка их позы, публикация результатов в виде сообщений и TF‑трансформаций.
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

#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

#include <image_transport/image_transport.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <tf2_ros/transform_broadcaster.h>

extern "C" {
#include <apriltag.h>
}

namespace apriltag_ros
{

class AprilTagNode : public rclcpp::Node
{
public:
  // ---------- Публичный интерфейс ----------
  // Конструктор/деструктор и основной колбэк обработки кадров
  /// @brief Конструктор: читает параметры, инициализирует детектор и ROS‑интерфейсы
  explicit AprilTagNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AprilTagNode();

  /// @brief Колбэк, вызываемый при получении нового изображения и camera_info
  void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr & msg_img,
                const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_ci);

private:
  // ---------- Приватные методы ----------
  // Вспомогательные функции сортировки, фильтрации детекций и геометрии
  /// @brief Компаратор для сортировки zarray по ID тега
  static int idComparison(const void * first, const void * second);
  /// @brief Удаляет повторные детекции одного и того же ID (опц.)
  void removeDuplicates(zarray_t * detections_);

  /// @brief Формирует модельные 3D‑точки углов тега
  void addObjectPoints(double s, cv::Matx44d T_oi,
                       std::vector<cv::Point3d> & objectPoints) const;

  /// @brief Извлекает пиксельные координаты углов по гомографии H
  void addImagePoints(apriltag_detection_t * detection,
                      std::vector<cv::Point2d> & imagePoints) const;

  /// @brief Решает PnP и возвращает матрицу T_cam_tag
  Eigen::Matrix4d getRelativeTransform(std::vector<cv::Point3d> objectPoints,
                                       std::vector<cv::Point2d> imagePoints,
                                       double fx, double fy, double cx, double cy) const;

  /// @brief Формирует geometry_msgs::TransformStamped для TF broadcaster
  geometry_msgs::msg::TransformStamped makeTagPose(const Eigen::Matrix4d & transform,
                                                   Eigen::Quaternion<double> rot_quaternion,
                                                   const std_msgs::msg::Header & header);

  // ---------- Члены‑данные ----------
  // Объекты ROS и внутренние структуры детектора
  // Подписка image_transport на пару image + camera_info
  image_transport::CameraSubscriber image_subscriber_;

  // Паблишер массива apriltag_msgs::AprilTagDetectionArray
  rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections_;
  // Бродкастер TF‑трансформаций (cam_frame → tag_frame)
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // C‑структура детектора apriltag (создаётся в конструкторе)
  apriltag_detector_t * td_{nullptr};
  // Выбранное семейство тегов (tag36h11 и др.)
  apriltag_family_t   * tf_{nullptr};

  // --- Настраиваемые ROS‑параметры ---
  // Имя семейства тегов (строка, напр. "36h11")
  std::string tag_family_;
  // Максимальное допустимое значение Hamming‑дистанции
  int   max_hamming_{};
  // Переставлять ли оси в режим "Z‑вверх"
  bool  z_up_{};
  // Физический размер ребра тега (м)
  double tag_edge_size_{};
  // Удалять ли дублирующиеся детекции одного ID
  bool  remove_duplicates_{};

  // --- Необязательные таблицы переопределения ---
  // Позволяют задать индивидуальный размер тега или child_frame_id
  // Индивидуальные размеры для тегов (id → размер, м)
  std::map<int, double> tag_sizes;
  // Собственные имена child_frame для TF (id → frame_id)
  std::map<int, std::string> tag_frames;

  // --- Статические словари для создания/очистки семейств ---
  // map<имя_семейства, функция_создания>
  static const std::map<std::string, apriltag_family_t *(*)(void)> tag_create_;
  // map<имя_семейства, функция_удаления>
  static const std::map<std::string, void (*)(apriltag_family_t *)> tag_destroy_;
};

}  // namespace apriltag_ros
