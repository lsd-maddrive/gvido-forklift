// -----------------------------------------------------------------------------
// AprilTagNode.cpp — обнаружение AprilTag в ROS 2
// ----------------------------------------------------------------------------
// BSD 2‑Clause License
// -----------------------------------------------------------------------------

#include "AprilTagNode.hpp"

// ------------- Библиотеки OpenCV и ROS -------------
#include <cv_bridge/cv_bridge.hpp>         // Преобразование изображений ROS⇄OpenCV
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.hpp>

// tf2_eigen может лежать в разных заголовках на разных платформах
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
  #include <tf2_eigen/tf2_eigen.hpp>
#else
  #include <tf2_eigen/tf2_eigen.h>
#endif

// Для AngleAxis и матриц поворота
#include <Eigen/Geometry>

// ------------ Сообщения с результатами обнаружения ------------
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

// ------------ C‑API библиотеки apriltag -------------
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

#include <rclcpp_components/register_node_macro.hpp>
#include <chrono>

#define TAG_CREATE(name)  {#name, tag ## name ## _create},
#define TAG_DESTROY(name) {#name, tag ## name ## _destroy},

namespace apriltag_ros
{

// -----------------------------------------------------------------------------
// 1. ***СТАТИЧЕСКИЕ КАРТЫ***
// -----------------------------------------------------------------------------
// map<имя_семейства, функция_создания>. Позволяет выбрать семейство по строке
// из параметра ROS.
// -----------------------------------------------------------------------------
const std::map<std::string, apriltag_family_t *(*)(void)> AprilTagNode::tag_create_ = {
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

// Аналогичная карта для функций освобождения памяти.
const std::map<std::string, void (*)(apriltag_family_t *)> AprilTagNode::tag_destroy_ = {
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

// -----------------------------------------------------------------------------
// 2. ***КОНСТРУКТОР / ДЕСТРУКТОР***
// -----------------------------------------------------------------------------
// В конструкторе:
//  • Читаем параметры из ROS‑параметр‑сервиса (семейство, фильтры, геометрию).
//  • Инициализируем детектор AprilTag.
//  • Подписываемся на пару топиков image + camera_info с помощью image_transport.
//  • Подготавливаем паблишер результатов и broadcaster TF.
// -----------------------------------------------------------------------------
AprilTagNode::AprilTagNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("apriltag", options)
{
  RCLCPP_INFO(get_logger(), "AprilTagNode стартует…");

  // ----------- Параметры, настраиваемые через launch/ros2 param -----------
  tag_family_        = declare_parameter<std::string>("family", "36h11");
  max_hamming_       = declare_parameter<int>("max_hamming", 0);   // фильтрация по Hamming‑дистанции
  z_up_              = declare_parameter<bool>("z_up", true);     // ориентация осей в выходной TF
  tag_edge_size_     = declare_parameter<double>("tag_edge_size", 0.162); // по умолчанию 162 мм
  remove_duplicates_ = declare_parameter<bool>("remove_duplicates", true); // удалять ли дубликаты

  RCLCPP_INFO(get_logger(), "Параметры: family=%s, max_hamming=%d, z_up=%d, edge_size=%.3f, rm_dup=%d",
              tag_family_.c_str(), max_hamming_, z_up_, tag_edge_size_, remove_duplicates_);

  // -------- Проверяем, поддерживается ли указанное семейство --------
  if (!tag_create_.count(tag_family_)) {
    RCLCPP_WARN(get_logger(), "Семейство '%s' не поддерживается, переключаемся на 36h11", tag_family_.c_str());
    tag_family_ = "36h11";
  }

  // -------- Создаём детектор --------
  tf_ = tag_create_.at(tag_family_)();          // структура семейства
  td_ = apriltag_detector_create();             // сам детектор
  apriltag_detector_add_family(td_, tf_);       // регистрируем семейство

  // Параметры детектора (decimate — масштаб, blur, потоки, refine_edges)
  td_->quad_decimate  = declare_parameter<float>("decimate", 1.0f);
  td_->quad_sigma     = declare_parameter<float>("blur", 0.0f);
  td_->nthreads       = declare_parameter<int>("threads", 4);
  td_->debug          = declare_parameter<int>("debug", 0);
  td_->refine_edges   = declare_parameter<int>("refine-edges", 1);

  RCLCPP_INFO(get_logger(), "Настройки детектора: decimate=%.2f blur=%.2f threads=%d refine=%d",
              td_->quad_decimate, td_->quad_sigma, td_->nthreads, td_->refine_edges);

  // -------- Подписка на изображение --------
  // Используем image_transport::create_camera_subscription, т.к. нужен и image, и camera_info.
  // Второй аргумент bind'им к onCamera() — основному колбэку обработки кадров.
  image_subscriber_ = image_transport::create_camera_subscription(
    this, "image",
    std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2),
    declare_parameter<std::string>("image_transport", "raw"));

  RCLCPP_INFO(get_logger(), "Подписка на 'image' и 'camera_info' установлена");

  // -------- Паблишер результатов --------
  pub_detections_ = create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
    "apriltag_detections", rclcpp::QoS(10));

  // -------- TF Broadcaster --------
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

AprilTagNode::~AprilTagNode()
{
  RCLCPP_INFO(get_logger(), "Завершение работы AprilTagNode");
  if (td_) { apriltag_detector_destroy(td_); td_ = nullptr; }
  if (tf_) { tag_destroy_.at(tag_family_)(tf_); tf_ = nullptr; }
}

// -----------------------------------------------------------------------------
// 3. ***HELPERS***
// -----------------------------------------------------------------------------
// idComparison()    — функция сравнения по ID для сортировки zarray.
// removeDuplicates() — удаляет дубли при необходимости.
// -----------------------------------------------------------------------------
int AprilTagNode::idComparison(const void * first, const void * second)
{
  int a = (*static_cast<const apriltag_detection_t * const *>(first))->id;
  int b = (*static_cast<const apriltag_detection_t * const *>(second))->id;
  return (a > b) - (a < b);
}

void AprilTagNode::removeDuplicates(zarray_t * detections_)
{
  // Сортируем детекции по ID и удаляем все повторяющиеся.
  zarray_sort(detections_, &AprilTagNode::idComparison);
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
      dup = true;           // продолжаем убирать, пока ID тот же
    } else {
      ++count;
      dup = false;
    }
  }
}

// -----------------------------------------------------------------------------
// 4. ***CALLBACK onCamera()***
// -----------------------------------------------------------------------------
// Вызывается каждый раз, когда приходит синхронная пара
//   sensor_msgs/Image          (собственно кадр)
//   sensor_msgs/CameraInfo     (матрица калибровки)
//   1) Конвертируем изображение в grayscale (MONO8) при помощи cv_bridge.
//   2) Вызываем apriltag_detector_detect.
//   3) Удаляем дубликаты, фильтруем по Hamming.
//   4) Для каждой детекции считаем позу через solvePnP.
//   5) Публикуем TF и сообщение AprilTagDetectionArray.
// -----------------------------------------------------------------------------
void AprilTagNode::onCamera(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_img,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_ci)
{
  const auto start = std::chrono::steady_clock::now(); // для профилировки

  // -------- 1) cv_bridge: ROS → OpenCV --------
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::MONO8);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge исключение: %s", e.what());
    return;
  }

  // -------- 2) Подготовка структуры image_u8_t для C‑API apriltag --------
  image_u8_t im{ cv_ptr->image.cols,
                 cv_ptr->image.rows,
                 cv_ptr->image.cols,
                 cv_ptr->image.data };

  // -------- 3) Параметры камеры --------
  const auto & K = msg_ci->k; // 3×3 матрица вектором из 9 элементов
  double fx = K[0];
  double fy = K[4];
  double cx = K[2];
  double cy = K[5];

  RCLCPP_DEBUG(get_logger(), "Кадр %s: %dx%d fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
               msg_img->header.frame_id.c_str(), im.width, im.height, fx, fy, cx, cy);

  // -------- 4) Детекция тегов --------
  zarray_t * detections = apriltag_detector_detect(td_, &im);
  const int raw_cnt = zarray_size(detections);
  if (remove_duplicates_) {
    removeDuplicates(detections);
  }
  const int final_cnt = zarray_size(detections);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                       "Обнаружено %d тегов (сырых %d) в последнем кадре", final_cnt, raw_cnt);

  // -------- 5) Заполняем сообщение  --------
  apriltag_msgs::msg::AprilTagDetectionArray msg_array;
  msg_array.header = msg_img->header;

  for (int i = 0; i < final_cnt; ++i) {
    apriltag_detection_t * det;
    zarray_get(detections, i, &det);

    // ---- 5.1) Фильтрация по списку разрешённых ID и hamming ‑ дистанции ----
    if (!tag_frames.empty() && !tag_frames.count(det->id)) {
      RCLCPP_DEBUG(get_logger(), "Пропуск тега %d: нет в tag_frames", det->id);
      continue;
    }
    if (det->hamming > max_hamming_) {
      RCLCPP_DEBUG(get_logger(), "Пропуск тега %d: hamming %d > %d", det->id, det->hamming, max_hamming_);
      continue;
    }

    // ---- 5.2) Готовим модельные точки (углы квадрата в 3D) ----
    double half = (tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size_) / 2.0;
    std::vector<cv::Point3d> obj_pts;
    addObjectPoints(half, cv::Matx44d::eye(), obj_pts);

    // ---- 5.3) Картируем углы обнаруженного тега в пиксельные координаты ----
    std::vector<cv::Point2d> img_pts;
    addImagePoints(det, img_pts);

    // ---- 5.4) Решаем PnP → получаем SE3‑трансформацию тега относительно камеры ----
    Eigen::Matrix4d T = getRelativeTransform(obj_pts, img_pts, fx, fy, cx, cy);
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Quaternion<double> q(R);

    // ---- 5.5) Публикуем TF transform (camera_frame → tag_frame) ----
    geometry_msgs::msg::TransformStamped tag_pose = makeTagPose(T, q, msg_img->header);
    tag_pose.child_frame_id = tag_frames.count(det->id)
      ? tag_frames.at(det->id)
      : std::string(det->family->name) + ":" + std::to_string(det->id);
    tf_broadcaster_->sendTransform(tag_pose);

    // ---- 5.6) Заполняем элемент массива detections ----
    apriltag_msgs::msg::AprilTagDetection det_msg;
    det_msg.family = std::string(det->family->name);
    det_msg.pose.pose.pose.position.x = T(0,3);
    det_msg.pose.pose.pose.position.y = T(1,3);
    det_msg.pose.pose.pose.position.z = T(2,3);
    det_msg.pose.pose.pose.orientation = tag_pose.transform.rotation;
    det_msg.id   = det->id;
    det_msg.size = half * 2.0; // реальный размер ребра тега (м)

    msg_array.detections.push_back(det_msg);

    RCLCPP_DEBUG(get_logger(), "Тег %d (fam %s): pos[%.3f %.3f %.3f] size=%.3f",
                 det->id, det_msg.family.c_str(), T(0,3), T(1,3), T(2,3), det_msg.size);
  }

  // -------- 6) Публикуем массив детекций --------
  pub_detections_->publish(msg_array);

  // -------- 7) Чистим --------
  apriltag_detections_destroy(detections);

  const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
  RCLCPP_DEBUG(get_logger(), "Кадр обработан за %ld ms, опубликовано %zu детекций", dt, msg_array.detections.size());
}

// -----------------------------------------------------------------------------
// 5. ***GEOMETRY HELPERS***
// -----------------------------------------------------------------------------
// addObjectPoints() — генерирует 3D‑координаты четырёх углов тега.
// addImagePoints()  — прожектирует шаблон квадрата через матрицу гомографии
//                     H, полученную от apriltag, чтобы получить пиксельные точки.
// getRelativeTransform() — решает задачу PnP и возвращает 4×4 матрицу T_c_t.
// makeTagPose() — упаковывает Eigen‑матрицу в geometry_msgs::TransformStamped,
//                 учитывая вариант z‑up.
// -----------------------------------------------------------------------------

void AprilTagNode::addObjectPoints(double s, cv::Matx44d T_oi,
                                   std::vector<cv::Point3d> & objectPoints) const
{
  // Четыре угла квадрата в плоскости Z=0, центрированного в (0,0)
  objectPoints.emplace_back(T_oi(0,3) - s, T_oi(1,3) - s, T_oi(2,3));
  objectPoints.emplace_back(T_oi(0,3) + s, T_oi(1,3) - s, T_oi(2,3));
  objectPoints.emplace_back(T_oi(0,3) + s, T_oi(1,3) + s, T_oi(2,3));
  objectPoints.emplace_back(T_oi(0,3) - s, T_oi(1,3) + s, T_oi(2,3));
}

void AprilTagNode::addImagePoints(apriltag_detection_t * detection,
                                  std::vector<cv::Point2d> & imagePoints) const
{
  // Проецируем заданные опорные точки через гомографию обнаруженного тега
  double tag_x[4] = {-1, 1, 1, -1};
  double tag_y[4] = { 1, 1,-1, -1};
  double im_x, im_y;
  for (int i = 0; i < 4; ++i) {
    homography_project(detection->H, tag_x[i], tag_y[i], &im_x, &im_y);
    imagePoints.emplace_back(im_x, im_y);
  }
}

// solvePnP → преобразование из системы камеры в систему тега
Eigen::Matrix4d AprilTagNode::getRelativeTransform(std::vector<cv::Point3d> objectPoints,
                                                   std::vector<cv::Point2d> imagePoints,
                                                   double fx, double fy, double cx, double cy) const
{
  cv::Mat rvec, tvec;
  cv::Matx33d K(fx, 0, cx, 0, fy, cy, 0, 0, 1);
  cv::Vec4f D(0,0,0,0); // без дисторсии
  cv::solvePnP(objectPoints, imagePoints, K, D, rvec, tvec);
  cv::Matx33d R_cv;
  cv::Rodrigues(rvec, R_cv);

  // Переставляем оси, если выбран режим z_up
  Eigen::Matrix3d R_e;
  if (z_up_) {
    R_e << R_cv(1,0), R_cv(1,1), R_cv(1,2),
           R_cv(2,0), R_cv(2,1), R_cv(2,2),
           R_cv(0,0), R_cv(0,1), R_cv(0,2);

    // Дополнительная post-ротация для ориентации: Z вверх, X вперёд, Y влево
    // Eigen::Vector3d axis(-1.0, 1.0, -1.0);
    // axis.normalize();
    Eigen::AngleAxisd rot_x(M_PI / 2.0, Eigen::Vector3d::UnitX());
    // Eigen::AngleAxisd rot_x(M_PI / 2.0, Eigen::Vector3d::UnitX());

    // Eigen::Matrix3d R_correction = rot_correction.matrix();
    R_e = R_e * rot_x ;

  } else {
    R_e << R_cv(0,0), R_cv(0,1), R_cv(0,2),
           R_cv(1,0), R_cv(1,1), R_cv(1,2),
           R_cv(2,0), R_cv(2,1), R_cv(2,2);
  }

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = R_e;
  T(0,3) = tvec.at<double>(0);
  T(1,3) = tvec.at<double>(1);
  T(2,3) = tvec.at<double>(2);
  return T;
}

geometry_msgs::msg::TransformStamped AprilTagNode::makeTagPose(
  const Eigen::Matrix4d & transform,
  Eigen::Quaternion<double> rot_quaternion,
  const std_msgs::msg::Header & header)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header = header;
  tf_msg.transform.translation.x = transform(0,3);
  tf_msg.transform.translation.y = transform(1,3);
  tf_msg.transform.translation.z = transform(2,3);

  // Переставляем компоненты кватерниона при необходимости z_up
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

} // namespace apriltag_ros

// -----------------------------------------------------------------------------
// 6. ***MAIN()***
// -----------------------------------------------------------------------------
// Позволяет запускать узел как отдельный executable, помимо возможности загрузки
// через rclcpp_components.
// -----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<apriltag_ros::AprilTagNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// Регистрация компонента для динамической загрузки в compostable‑container
RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_ros::AprilTagNode)