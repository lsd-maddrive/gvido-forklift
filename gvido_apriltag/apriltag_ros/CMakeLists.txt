cmake_minimum_required(VERSION 3.8)
project(apriltag_ros)

set(CMAKE_CXX_STANDARD 17)          # лучше использовать 17 в Jazzy
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)               # ← ОБЯЗАТЕЛЬНО
find_package(Eigen3 REQUIRED)
find_package(apriltag REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

# ====================== Старый узел ======================
add_library(AprilTagNode SHARED src/AprilTagNode.cpp)
add_executable(tag_detector src/AprilTagNode.cpp)

# ====================== Новый мульти-узел ======================
add_library(MultiAprilTagNode SHARED src/multiAprilTagNode.cpp)
add_executable(multi_tag_detector src/multiAprilTagNode.cpp)

# ====================== Зависимости ======================
set(DEPENDENCIES
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  apriltag_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  image_transport
  cv_bridge
  Eigen3
  apriltag
)

ament_target_dependencies(AprilTagNode     ${DEPENDENCIES})
ament_target_dependencies(MultiAprilTagNode ${DEPENDENCIES})
ament_target_dependencies(tag_detector     ${DEPENDENCIES})
ament_target_dependencies(multi_tag_detector ${DEPENDENCIES})

# OpenCV нужно линковать явно
target_link_libraries(AprilTagNode     ${OpenCV_LIBS} apriltag::apriltag)
target_link_libraries(MultiAprilTagNode ${OpenCV_LIBS} apriltag::apriltag)
target_link_libraries(tag_detector     ${OpenCV_LIBS} apriltag::apriltag)
target_link_libraries(multi_tag_detector ${OpenCV_LIBS} apriltag::apriltag)

# ====================== Регистрация компонентов ======================
rclcpp_components_register_nodes(AprilTagNode     "apriltag_ros::AprilTagNode")
rclcpp_components_register_nodes(MultiAprilTagNode "apriltag_ros::MultiAprilTagNode")

# ====================== Установка ======================
install(TARGETS 
  AprilTagNode
  MultiAprilTagNode
  tag_detector
  multi_tag_detector
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)
ament_export_libraries(AprilTagNode MultiAprilTagNode)

ament_package()