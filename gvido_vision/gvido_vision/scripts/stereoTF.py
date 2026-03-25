#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
import tf_transformations as tf
import numpy as np
from geometry_msgs.msg import Pose, TransformStamped
import message_filters

from tf2_ros import StaticTransformBroadcaster


class CameraTfCalibrator(Node):
    def __init__(self):
        super().__init__('camera_tf_calibrator')

        self.transforms = []
        self.current_tag_id = None
        self.min_pairs = 30
        self.max_angle_diff = 0.15

        # Добавка к z перед выводом команды base_link -> camera_link_L
        self.z_offset_for_output = 1.245

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.last_published_tf = None
        self.final_command_printed = False

        sub_left = message_filters.Subscriber(
            self, AprilTagDetectionArray, '/apriltag_detections_left'
        )
        sub_right = message_filters.Subscriber(
            self, AprilTagDetectionArray, '/apriltag_detections_right'
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [sub_left, sub_right], queue_size=30, slop=0.2
        )
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info("Camera TF Calibrator запущен")
        self.get_logger().info(
            f"Ожидаем минимум {self.min_pairs} надёжных пар для публикации TF"
        )

    def sync_callback(self, msg_left, msg_right):
        left_by_id = {d.id: d for d in msg_left.detections}
        right_by_id = {d.id: d for d in msg_right.detections}

        common_ids = set(left_by_id.keys()) & set(right_by_id.keys())
        if not common_ids:
            return

        if self.current_tag_id is not None and self.current_tag_id in common_ids:
            tag_id = self.current_tag_id
        else:
            tag_id = sorted(common_ids)[0]
            self.current_tag_id = tag_id

        det_left = left_by_id[tag_id]
        det_right = right_by_id[tag_id]

        T_L_tag = pose_to_matrix(det_left.pose.pose.pose)
        T_R_tag = pose_to_matrix(det_right.pose.pose.pose)

        qL = normalize_quaternion(tf.quaternion_from_matrix(T_L_tag))
        qR = normalize_quaternion(tf.quaternion_from_matrix(T_R_tag))
        dot = np.clip(np.abs(np.dot(qL, qR)), -1.0, 1.0)
        angle_diff = 2.0 * np.arccos(dot)

        if angle_diff > self.max_angle_diff:
            return

        # camera_link_L -> camera_link_R
        T_L_R_current = np.dot(T_L_tag, np.linalg.inv(T_R_tag))
        self.transforms.append(T_L_R_current)

        T_proposed = self.compute_average_tf()

        if len(self.transforms) >= self.min_pairs:
            self.publish_static_tf(T_proposed)

        if len(self.transforms) % 5 == 0 or len(self.transforms) == self.min_pairs:
            roll, pitch, yaw = matrix_to_rpy(T_proposed)
            self.get_logger().info(
                f"Собрано пар: {len(self.transforms)}/{self.min_pairs} | "
                f"x={T_proposed[0,3]:.4f}, y={T_proposed[1,3]:.4f}, z={T_proposed[2,3]:.4f} | "
                f"roll={roll:.4f}, pitch={pitch:.4f}, yaw={yaw:.4f}"
            )

    def compute_average_tf(self):
        if not self.transforms:
            return np.eye(4)

        if len(self.transforms) < 5:
            return self.transforms[-1]

        translations = np.array([T[:3, 3] for T in self.transforms])
        avg_trans = np.median(translations, axis=0)

        quats = [normalize_quaternion(tf.quaternion_from_matrix(T)) for T in self.transforms]
        avg_quat = average_quaternions(quats)
        avg_rot = tf.quaternion_matrix(avg_quat)[:3, :3]

        T = np.eye(4)
        T[:3, :3] = avg_rot
        T[:3, 3] = avg_trans
        return T

    def publish_static_tf(self, T_L_R: np.ndarray):
        if self.last_published_tf is not None:
            translation_delta = np.linalg.norm(
                T_L_R[:3, 3] - self.last_published_tf[:3, 3]
            )

            q_new = normalize_quaternion(tf.quaternion_from_matrix(T_L_R))
            q_old = normalize_quaternion(tf.quaternion_from_matrix(self.last_published_tf))
            rot_dot = np.clip(np.abs(np.dot(q_new, q_old)), -1.0, 1.0)
            rotation_delta = 2.0 * np.arccos(rot_dot)

            if translation_delta < 0.001 and rotation_delta < 0.001:
                return

        # Публикация в TF через quaternion
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_link_L"
        t.child_frame_id = "camera_link_R"

        t.transform.translation.x = float(T_L_R[0, 3])
        t.transform.translation.y = float(T_L_R[1, 3])
        t.transform.translation.z = float(T_L_R[2, 3])

        q = normalize_quaternion(tf.quaternion_from_matrix(T_L_R))
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])

        self.tf_broadcaster.sendTransform(t)
        self.last_published_tf = T_L_R.copy()

        if len(self.transforms) >= self.min_pairs and not self.final_command_printed:
            self.print_final_commands(T_L_R)
            self.final_command_printed = True

    def print_final_commands(self, T_L_R):
        x_lr, y_lr, z_lr = T_L_R[:3, 3]
        roll_lr, pitch_lr, yaw_lr = matrix_to_rpy(T_L_R)

        T_B_L, _ = compute_base_to_left_transform(T_L_R)

        x_bl, y_bl, z_bl = T_B_L[:3, 3]
        roll_bl, pitch_bl, yaw_bl = matrix_to_rpy(T_B_L)

        # Правка только перед выводом команды base_link -> camera_link_L
        x_bl_out = x_bl
        y_bl_out = -y_bl
        z_bl_out = z_bl + self.z_offset_for_output

        roll_bl_out = -roll_bl
        pitch_bl_out = pitch_bl
        yaw_bl_out = -yaw_bl

        self.get_logger().info("\n" + "=" * 90)
        self.get_logger().info("✅ Финальные трансформации готовы:")

        self.get_logger().info("\n1) camera_link_L → camera_link_R")
        self.get_logger().info(
            f"   xyz: x={x_lr:.6f}  y={y_lr:.6f}  z={z_lr:.6f}"
        )
        self.get_logger().info(
            f"   rpy: roll={roll_lr:.6f}  pitch={pitch_lr:.6f}  yaw={yaw_lr:.6f}"
        )

        self.get_logger().info("\n2) base_link → camera_link_L")
        self.get_logger().info(
            f"   исходные xyz: x={x_bl:.6f}  y={y_bl:.6f}  z={z_bl:.6f}"
        )
        self.get_logger().info(
            f"   исходные rpy: roll={roll_bl:.6f}  pitch={pitch_bl:.6f}  yaw={yaw_bl:.6f}"
        )

        self.get_logger().info("\n2a) base_link → camera_link_L (после правки перед выводом)")
        self.get_logger().info(
            f"   xyz: x={x_bl_out:.6f}  y={y_bl_out:.6f}  z={z_bl_out:.6f}"
        )
        self.get_logger().info(
            f"   rpy: roll={roll_bl_out:.6f}  pitch={pitch_bl_out:.6f}  yaw={yaw_bl_out:.6f}"
        )

        self.get_logger().info("\nПринятая геометрия base_link:")
        self.get_logger().info("   origin(base_link) = середина между камерами")
        self.get_logger().info("   y_base = от левой камеры к правой")
        self.get_logger().info("   x_base = среднее 'вперёд' камер, спроецированное перпендикулярно baseline")
        self.get_logger().info("   z_base = x_base × y_base")

        # Формат static_transform_publisher с именованными аргументами
        self.get_logger().info("\nГотовая команда camera_link_L -> camera_link_R:")
        self.get_logger().info(
            f"ros2 run tf2_ros static_transform_publisher "
            f"--x {x_lr:.6f} --y {y_lr:.6f} --z {z_lr:.6f} "
            f"--yaw {yaw_lr:.6f} --pitch {pitch_lr:.6f} --roll {roll_lr:.6f} "
            f"--frame-id camera_link_L --child-frame-id camera_link_R"
        )

        self.get_logger().info("\nГотовая команда base_link -> camera_link_L:")
        self.get_logger().info(
            f"ros2 run tf2_ros static_transform_publisher "
            f"--x {x_bl_out:.6f} --y {y_bl_out:.6f} --z {z_bl_out:.6f} "
            f"--yaw {yaw_bl_out:.6f} --pitch {pitch_bl_out:.6f} --roll {roll_bl_out:.6f} "
            f"--frame-id base_link --child-frame-id camera_link_L"
        )

        # Дополнительный формат arguments=[...]
        self.get_logger().info("\nФормат arguments для camera_link_L -> camera_link_R:")
        self.get_logger().info(
            format_arguments_line(
                x_lr, y_lr, z_lr,
                roll_lr, pitch_lr, yaw_lr,
                "camera_link_L", "camera_link_R"
            )
        )

        self.get_logger().info("\nФормат arguments для base_link -> camera_link_L:")
        self.get_logger().info(
            format_arguments_line(
                x_bl_out, y_bl_out, z_bl_out,
                roll_bl_out, pitch_bl_out, yaw_bl_out,
                "base_link", "camera_link_L"
            )
        )

        self.get_logger().info("=" * 90)

    def destroy_node(self):
        if self.transforms:
            self.get_logger().info(f"Узел завершён. Собрано {len(self.transforms)} пар.")
        super().destroy_node()


def pose_to_matrix(pose: Pose) -> np.ndarray:
    mat = np.eye(4)
    mat[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    q = normalize_quaternion([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ])
    mat[:3, :3] = tf.quaternion_matrix(q)[:3, :3]
    return mat


def normalize_quaternion(q):
    q = np.array(q, dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    q /= norm
    if q[3] < 0:
        q = -q
    return q


def normalize_vector(v):
    v = np.array(v, dtype=np.float64)
    n = np.linalg.norm(v)
    if n < 1e-12:
        return None
    return v / n


def project_to_plane(v, normal):
    normal = normalize_vector(normal)
    return v - np.dot(v, normal) * normal


def average_quaternions(quaternions):
    Q = np.array([normalize_quaternion(q) for q in quaternions], dtype=np.float64)

    ref = Q[0]
    for i in range(len(Q)):
        if np.dot(Q[i], ref) < 0:
            Q[i] = -Q[i]

    A = np.zeros((4, 4), dtype=np.float64)
    for q in Q:
        A += np.outer(q, q)
    A /= len(Q)

    eigenvalues, eigenvectors = np.linalg.eigh(A)
    avg_quat = eigenvectors[:, np.argmax(eigenvalues)]
    return normalize_quaternion(avg_quat)


def invert_transform(T):
    R = T[:3, :3]
    t = T[:3, 3]

    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def matrix_to_rpy(T):
    q = normalize_quaternion(tf.quaternion_from_matrix(T))
    roll, pitch, yaw = tf.euler_from_quaternion(q, 'sxyz')
    return roll, pitch, yaw


def format_arguments_line(x, y, z, roll, pitch, yaw, parent_frame, child_frame):
    return (
        f"arguments=['{x:.4f}', '{y:.4f}', '{z:.4f}', "
        f"'{yaw:.4f}', '{pitch:.4f}', '{roll:.4f}', "
        f"'{parent_frame}', '{child_frame}'],"
    )


def compute_base_to_left_transform(T_L_R):
    """
    По T_L_R = camera_link_L -> camera_link_R
    строим T_L_B = camera_link_L -> base_link,
    затем инвертируем и получаем T_B_L = base_link -> camera_link_L.
    """
    R_L_R = T_L_R[:3, :3]
    p_R_in_L = T_L_R[:3, 3]

    baseline = normalize_vector(p_R_in_L)
    if baseline is None:
        raise ValueError("Невозможно построить base_link: baseline между камерами имеет нулевую длину.")

    p_B_in_L = 0.5 * p_R_in_L

    z_L_in_L = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    z_R_in_L = R_L_R @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    forward_avg = z_L_in_L + z_R_in_L

    x_B_in_L = project_to_plane(forward_avg, baseline)
    x_B_in_L = normalize_vector(x_B_in_L)

    if x_B_in_L is None:
        x_guess = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        x_B_in_L = normalize_vector(project_to_plane(x_guess, baseline))

    if x_B_in_L is None:
        y_guess = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        x_B_in_L = normalize_vector(project_to_plane(y_guess, baseline))

    if x_B_in_L is None:
        raise ValueError("Не удалось построить ось x_base.")

    y_B_in_L = baseline
    z_B_in_L = normalize_vector(np.cross(x_B_in_L, y_B_in_L))
    if z_B_in_L is None:
        raise ValueError("Не удалось построить ось z_base.")

    x_B_in_L = normalize_vector(np.cross(y_B_in_L, z_B_in_L))

    R_L_B = np.column_stack((x_B_in_L, y_B_in_L, z_B_in_L))

    T_L_B = np.eye(4)
    T_L_B[:3, :3] = R_L_B
    T_L_B[:3, 3] = p_B_in_L

    T_B_L = invert_transform(T_L_B)

    return T_B_L, T_L_B


def main(args=None):
    rclpy.init(args=args)
    node = CameraTfCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()