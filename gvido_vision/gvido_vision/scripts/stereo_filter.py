#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from collections import deque, defaultdict

class TagStabilizer(Node):
    def __init__(self):
        super().__init__('tag_tf_stabilizer')
        self.window_size = 10
        self.buffers = defaultdict(lambda: deque(maxlen=self.window_size))
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.get_logger().info('Filter started (clean version).')

    def tf_callback(self, msg: TFMessage):
        for tr in msg.transforms:
            child = tr.child_frame_id
            if '_stb' in child:
                continue
            if not child.startswith('tag_'):
                continue

            self.buffers[child].append({
                'x': tr.transform.translation.x,
                'y': tr.transform.translation.y,
                'z': tr.transform.translation.z,
            })

            if len(self.buffers[child]) >= 5:
                self.publish_stabilized(child)

    def publish_stabilized(self, tag_id):
        buffer = self.buffers[tag_id]
        avg_x = sum(d['x'] for d in buffer) / len(buffer)
        avg_y = sum(d['y'] for d in buffer) / len(buffer)
        avg_z = sum(d['z'] for d in buffer) / len(buffer)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link_R'
        t.child_frame_id = f'{tag_id}_stb'
        t.transform.translation.x = avg_x
        t.transform.translation.y = avg_y
        t.transform.translation.z = avg_z
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TagStabilizer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()