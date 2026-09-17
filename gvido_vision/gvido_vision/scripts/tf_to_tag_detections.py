#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray

class TFToTagDetections(Node):
    def __init__(self):
        super().__init__('tf_to_tag_detections')
        self.sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.pub = self.create_publisher(AprilTagDetectionArray, '/apriltag/detections', 10)
        self.get_logger().info("Конвертер TF -> /apriltag/detections запущен")
    
    def tf_callback(self, msg):
        detections = AprilTagDetectionArray()
        if msg.transforms:
            detections.header = msg.transforms[0].header
        
        for tr in msg.transforms:
            child = tr.child_frame_id
            if child.startswith('tag_'):
                try:
                    tag_id = int(child.replace('tag_', ''))
                except:
                    continue
                
                detection = AprilTagDetection()
                detection.id = [tag_id]
                detection.pose.pose.position.x = tr.transform.translation.x
                detection.pose.pose.position.y = tr.transform.translation.y
                detection.pose.pose.position.z = tr.transform.translation.z
                detection.pose.pose.orientation.w = 1.0
                detections.detections.append(detection)
        
        if detections.detections:
            self.pub.publish(detections)
            self.get_logger().info(f"Опубликовано {len(detections.detections)} тегов")

def main(args=None):
    rclpy.init(args=args)
    node = TFToTagDetections()
    rclpy.spin(node)

if __name__ == '__main__':
    main()