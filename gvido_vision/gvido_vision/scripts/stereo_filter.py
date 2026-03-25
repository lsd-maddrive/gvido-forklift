#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from collections import defaultdict, deque
from dataclasses import dataclass

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

import tf2_ros
from tf2_ros import TransformException


# ================= math =================

def quat_norm(q):
    n = math.sqrt(sum(x*x for x in q))
    return [x/n for x in q] if n > 1e-12 else [0,0,0,1]

def quat_conjugate(q):
    return [-q[0], -q[1], -q[2], q[3]]

def quat_multiply(q1, q2):
    x1,y1,z1,w1 = q1
    x2,y2,z2,w2 = q2
    return [
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ]

def quat_rotate_vector(q, v):
    return quat_multiply(
        quat_multiply(q, [*v,0]),
        quat_conjugate(q)
    )[:3]

def compose(t1,q1,t2,q2):
    t2r = quat_rotate_vector(q1, t2)
    t = [t1[i] + t2r[i] for i in range(3)]
    q = quat_norm(quat_multiply(q1,q2))
    return t,q

def vec_dist(a,b):
    return math.sqrt(sum((a[i]-b[i])**2 for i in range(3)))

def quat_angle(q1,q2):
    q1 = quat_norm(q1)
    q2 = quat_norm(q2)
    dot = abs(sum(q1[i]*q2[i] for i in range(4)))
    dot = max(-1.0,min(1.0,dot))
    return 2*math.acos(dot)

def avg_quat(q1,q2):
    q1 = quat_norm(q1)
    q2 = quat_norm(q2)
    if sum(q1[i]*q2[i] for i in range(4)) < 0:
        q2 = [-x for x in q2]
    return quat_norm([(q1[i]+q2[i])*0.5 for i in range(4)])


# ================= data =================

@dataclass
class Detection:
    stamp_ns: int
    parent: str
    trans: list
    quat: list


# ================= node =================

class NodeTF(Node):

    def __init__(self):
        super().__init__('tag_tf_stabilizer')

        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('max_pos', 0.1)
        self.declare_parameter('max_ang', 0.2)

        self.output = self.get_parameter('output_frame').value
        self.max_pos = self.get_parameter('max_pos').value
        self.max_ang = self.get_parameter('max_ang').value

        self.tf_buf = tf2_ros.Buffer(Duration(seconds=10.0))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf, self)
        self.br = tf2_ros.TransformBroadcaster(self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        self.sub = self.create_subscription(TFMessage, '/tf', self.cb, qos)

        self.L = defaultdict(deque)
        self.R = defaultdict(deque)

        self.get_logger().info(f'Запуск. frame={self.output}')

    def cb(self, msg):
        for tr in msg.transforms:
            name = tr.child_frame_id

            if name.endswith('_L'):
                side = 'L'
                base = name[:-2]
            elif name.endswith('_R'):
                side = 'R'
                base = name[:-2]
            else:
                continue

            d = Detection(
                stamp_ns = tr.header.stamp.sec*1_000_000_000 + tr.header.stamp.nanosec,
                parent = tr.header.frame_id,
                trans = [
                    tr.transform.translation.x,
                    tr.transform.translation.y,
                    tr.transform.translation.z
                ],
                quat = [
                    tr.transform.rotation.x,
                    tr.transform.rotation.y,
                    tr.transform.rotation.z,
                    tr.transform.rotation.w
                ]
            )

            (self.L if side=='L' else self.R)[base].append(d)
            self.match(base)

    def match(self, base):
        L = self.L[base]
        R = self.R[base]

        if not L or not R:
            return

        best = None

        for i, l in enumerate(L):
            for j, r in enumerate(R):
                dt = abs(l.stamp_ns - r.stamp_ns)
                if best is None or dt < best[0]:
                    best = (dt, i, j)

        _, i, j = best
        l = L[i]
        r = R[j]

        pose_l = self.to_out(l)
        pose_r = self.to_out(r)

        if pose_l is None or pose_r is None:
            return

        t1,q1 = pose_l
        t2,q2 = pose_r

        pos = vec_dist(t1,t2)
        ang = quat_angle(q1,q2)

        if pos > self.max_pos:
            self.get_logger().warn(f'{base}: отклонён по позиции {pos:.3f}')
            return

        if ang > self.max_ang:
            self.get_logger().warn(f'{base}: отклонён по углу {math.degrees(ang):.1f}°')
            return

        t = [(t1[i]+t2[i])*0.5 for i in range(3)]
        q = avg_quat(q1,q2)

        self.publish(base, t, q, l.stamp_ns, r.stamp_ns)

        del L[i]
        del R[j]

    def to_out(self, d):
        time = Time(
            seconds=int(d.stamp_ns // 1e9),
            nanoseconds=int(d.stamp_ns % 1e9)
        )

        try:
            tf = self.tf_buf.lookup_transform(
                self.output,
                d.parent,
                time.to_msg(),
                timeout=Duration(seconds=0.05)
            )
        except TransformException as e:
            self.get_logger().error(
                f'Нет TF {self.output}->{d.parent}'
            )
            return None

        t0 = [tf.transform.translation.x,
              tf.transform.translation.y,
              tf.transform.translation.z]

        q0 = [tf.transform.rotation.x,
              tf.transform.rotation.y,
              tf.transform.rotation.z,
              tf.transform.rotation.w]

        return compose(t0, q0, d.trans, d.quat)

    def publish(self, base, t, q, s1, s2):
        msg = TransformStamped()
        ns = (s1+s2)//2

        msg.header.stamp.sec = int(ns//1e9)
        msg.header.stamp.nanosec = int(ns%1e9)
        msg.header.frame_id = self.output
        msg.child_frame_id = base + '_stb'

        msg.transform.translation.x = t[0]
        msg.transform.translation.y = t[1]
        msg.transform.translation.z = t[2]

        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]

        self.br.sendTransform(msg)

        self.get_logger().info(f'{base}: опубликован')


def main():
    rclpy.init()
    node = NodeTF()

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