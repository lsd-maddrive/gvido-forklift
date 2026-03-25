#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from pathlib import Path
from collections import defaultdict, deque
from dataclasses import dataclass

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tf2_msgs.msg import TFMessage


# ============================================================
# Математика
# ============================================================

def quat_norm(q):
    n = math.sqrt(sum(x * x for x in q))
    if n < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [x / n for x in q]


def quat_dot_abs(q1, q2):
    q1 = quat_norm(q1)
    q2 = quat_norm(q2)
    return abs(sum(q1[i] * q2[i] for i in range(4)))


def quat_angle(q1, q2):
    dot = quat_dot_abs(q1, q2)
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def avg_quat(quats):
    """
    Простое усреднение с выравниванием знака относительно первого кватерниона.
    Для стабилизированных *_stb этого обычно достаточно.
    """
    if not quats:
        return [0.0, 0.0, 0.0, 1.0]

    ref = quat_norm(quats[0])
    aligned = []

    for q in quats:
        qn = quat_norm(q)
        dot = sum(ref[i] * qn[i] for i in range(4))
        if dot < 0.0:
            qn = [-x for x in qn]
        aligned.append(qn)

    q_avg = [sum(q[i] for q in aligned) / len(aligned) for i in range(4)]
    return quat_norm(q_avg)


def vec_mean(vectors):
    n = len(vectors)
    return [sum(v[i] for v in vectors) / n for i in range(3)]


def vec_dist(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )


# ============================================================
# Данные
# ============================================================

@dataclass
class PoseSample:
    stamp_ns: int
    frame_id: str
    child_frame_id: str
    t: list
    q: list


# ============================================================
# Узел
# ============================================================

class TagMapBuilder(Node):
    def __init__(self):
        super().__init__('tag_map_builder')

        # ---- параметры ----
        self.declare_parameter('yaml_filename', 'tag_map.yaml')
        self.declare_parameter('samples_per_window', 10)
        self.declare_parameter('min_windows_before_save', 5)
        self.declare_parameter('max_buffer_size', 100)
        self.declare_parameter('stb_suffix', '_stb')
        self.declare_parameter('save_only_frame', 'map')

        self.yaml_filename = self.get_parameter('yaml_filename').value
        self.samples_per_window = int(self.get_parameter('samples_per_window').value)
        self.min_windows_before_save = int(self.get_parameter('min_windows_before_save').value)
        self.max_buffer_size = int(self.get_parameter('max_buffer_size').value)
        self.stb_suffix = self.get_parameter('stb_suffix').value
        self.save_only_frame = self.get_parameter('save_only_frame').value

        # ---- путь к yaml в текущей директории ----
        self.yaml_path = Path.cwd() / self.yaml_filename

        # ---- данные ----
        self.saved_tags = self.load_or_create_yaml()
        self.buffers = defaultdict(lambda: deque(maxlen=self.max_buffer_size))
        self.best_candidates = {}
        self.window_counts = defaultdict(int)

        # ---- подписка ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )
        self.sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, qos)

        self.get_logger().info(f'Работаю с файлом: {self.yaml_path}')

    # ========================================================
    # YAML
    # ========================================================

    def load_or_create_yaml(self):
        if not self.yaml_path.exists():
            data = {'tags': {}}
            try:
                with self.yaml_path.open('w', encoding='utf-8') as f:
                    yaml.safe_dump(data, f, allow_unicode=True, sort_keys=True)
                self.get_logger().info(f'Создан новый файл: {self.yaml_path}')
            except Exception as e:
                self.get_logger().error(f'Не удалось создать {self.yaml_path}: {e}')
            return data['tags']

        try:
            with self.yaml_path.open('r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            if 'tags' not in data or not isinstance(data['tags'], dict):
                data = {'tags': {}}
            return data['tags']
        except Exception as e:
            self.get_logger().error(f'Ошибка чтения {self.yaml_path}: {e}')
            return {}

    def write_yaml(self):
        data = {'tags': self.saved_tags}
        try:
            with self.yaml_path.open('w', encoding='utf-8') as f:
                yaml.safe_dump(data, f, allow_unicode=True, sort_keys=True)
            return True
        except Exception as e:
            self.get_logger().error(f'Ошибка записи {self.yaml_path}: {e}')
            return False

    # ========================================================
    # TF callback
    # ========================================================

    def tf_callback(self, msg: TFMessage):
        for tr in msg.transforms:
            child = tr.child_frame_id
            if not child.endswith(self.stb_suffix):
                continue

            if self.save_only_frame and tr.header.frame_id != self.save_only_frame:
                continue

            tag_id = child[:-len(self.stb_suffix)]

            # если уже есть в yaml — ничего не делаем
            if tag_id in self.saved_tags:
                continue

            sample = PoseSample(
                stamp_ns=tr.header.stamp.sec * 1_000_000_000 + tr.header.stamp.nanosec,
                frame_id=tr.header.frame_id,
                child_frame_id=tr.child_frame_id,
                t=[
                    tr.transform.translation.x,
                    tr.transform.translation.y,
                    tr.transform.translation.z
                ],
                q=[
                    tr.transform.rotation.x,
                    tr.transform.rotation.y,
                    tr.transform.rotation.z,
                    tr.transform.rotation.w
                ]
            )

            self.buffers[tag_id].append(sample)
            self.try_update_candidate(tag_id)

    # ========================================================
    # Оценка лучшего окна
    # ========================================================

    def try_update_candidate(self, tag_id: str):
        buf = self.buffers[tag_id]

        if len(buf) < self.samples_per_window:
            return

        # Берём последнее окно из N измерений
        window = list(buf)[-self.samples_per_window:]

        translations = [s.t for s in window]
        quaternions = [s.q for s in window]

        t_mean = vec_mean(translations)
        q_mean = avg_quat(quaternions)

        # считаем "биение"
        pos_errors = [vec_dist(t, t_mean) for t in translations]
        ang_errors = [quat_angle(q, q_mean) for q in quaternions]

        pos_jitter_mean = sum(pos_errors) / len(pos_errors)
        pos_jitter_max = max(pos_errors)

        ang_jitter_mean = sum(ang_errors) / len(ang_errors)
        ang_jitter_max = max(ang_errors)

        # общий score: позиция + угол с мягким весом
        # угол в радианах, переводим примерно в "эквивалентную" метрику веса
        score = pos_jitter_mean + 0.2 * ang_jitter_mean + 0.5 * pos_jitter_max + 0.1 * ang_jitter_max

        self.window_counts[tag_id] += 1

        candidate = {
            'frame_id': window[-1].frame_id,
            'child_frame_id': f'{tag_id}{self.stb_suffix}',
            'translation': {
                'x': float(t_mean[0]),
                'y': float(t_mean[1]),
                'z': float(t_mean[2]),
            },
            'rotation': {
                'x': float(q_mean[0]),
                'y': float(q_mean[1]),
                'z': float(q_mean[2]),
                'w': float(q_mean[3]),
            },
            'stats': {
                'samples_per_window': int(self.samples_per_window),
                'windows_seen': int(self.window_counts[tag_id]),
                'pos_jitter_mean_m': float(pos_jitter_mean),
                'pos_jitter_max_m': float(pos_jitter_max),
                'ang_jitter_mean_rad': float(ang_jitter_mean),
                'ang_jitter_max_rad': float(ang_jitter_max),
                'ang_jitter_mean_deg': float(math.degrees(ang_jitter_mean)),
                'ang_jitter_max_deg': float(math.degrees(ang_jitter_max)),
                'score': float(score),
            }
        }

        prev = self.best_candidates.get(tag_id)
        if prev is None or candidate['stats']['score'] < prev['stats']['score']:
            self.best_candidates[tag_id] = candidate

        # Сохраняем только когда увидели достаточно окон
        if self.window_counts[tag_id] >= self.min_windows_before_save:
            self.save_best_candidate(tag_id)

    # ========================================================
    # Сохранение
    # ========================================================

    def save_best_candidate(self, tag_id: str):
        if tag_id in self.saved_tags:
            return

        candidate = self.best_candidates.get(tag_id)
        if candidate is None:
            return

        self.saved_tags[tag_id] = candidate
        ok = self.write_yaml()

        if ok:
            self.get_logger().info(
                f'Тег {tag_id} сохранён в {self.yaml_path.name} '
                f'(score={candidate["stats"]["score"]:.6f})'
            )

            # после сохранения больше этот тег не трогаем
            if tag_id in self.buffers:
                del self.buffers[tag_id]
            if tag_id in self.best_candidates:
                del self.best_candidates[tag_id]
            if tag_id in self.window_counts:
                del self.window_counts[tag_id]


def main(args=None):
    rclpy.init(args=args)
    node = TagMapBuilder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()