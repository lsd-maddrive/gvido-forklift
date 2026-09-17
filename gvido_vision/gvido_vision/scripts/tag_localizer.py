#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
import yaml
from pathlib import Path

class TagLocalizer(Node):
    def __init__(self):
        super().__init__('tag_localizer')
        
        # Статическая трансформация base_link → camera_link_R
        self.base_to_cam_x = 0.0
        self.base_to_cam_y = 0.2328
        self.base_to_cam_z = 1.245
        
        # Загружаем карту тегов
        self.tag_map = self.load_tag_map()
        
        # TF бродкастер
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Подписываемся на детекции тегов
        self.sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )
        
        # Текущая позиция робота в карте
        self.current_robot_x = 0.0
        self.current_robot_y = 0.0
        self.current_robot_z = 0.0
        
        # Публикуем map → odom постоянно (10 Гц)
        self.create_timer(0.1, self.publish_tf_loop)
        
        self.get_logger().info("Локализатор запущен")
        self.get_logger().info(f"Загружено тегов: {len(self.tag_map)}")
    
    def load_tag_map(self):
        yaml_path = Path.cwd() / 'tag_map.yaml'
        if not yaml_path.exists():
            self.get_logger().error(f"Файл {yaml_path} не найден!")
            return {}
        
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        
        tag_map = {}
        for tag_id, tag_data in data.get('tags', {}).items():
            tag_map[tag_id] = {
                'x': tag_data['translation']['x'],
                'y': tag_data['translation']['y'],
                'z': tag_data['translation']['z']
            }
            self.get_logger().info(f"  {tag_id}: x={tag_data['translation']['x']:.3f}, y={tag_data['translation']['y']:.3f}")
        
        return tag_map
    
    def calculate_robot_pose(self, tag_id, tag_x, tag_y):
        if tag_id not in self.tag_map:
            return None
        
        map_x = self.tag_map[tag_id]['x']
        map_y = self.tag_map[tag_id]['y']
        
        robot_x = map_x - tag_x - self.base_to_cam_x
        robot_y = map_y - tag_y - self.base_to_cam_y
        
        return (robot_x, robot_y)
    
    def publish_tf_loop(self):
        # 1. Публикуем map → odom (положение робота в карте)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = self.current_robot_x
        t.transform.translation.y = self.current_robot_y
        t.transform.translation.z = self.current_robot_z
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # 2. Публикуем odom → base_link (одометрия, пока 0)
        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = self.get_clock().now().to_msg()
        t_odom_base.header.frame_id = 'odom'
        t_odom_base.child_frame_id = 'base_link'
        t_odom_base.transform.translation.x = 0.0
        t_odom_base.transform.translation.y = 0.0
        t_odom_base.transform.translation.z = 0.0
        t_odom_base.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_odom_base)
    
    def tf_callback(self, msg: TFMessage):
        for tr in msg.transforms:
            child = tr.child_frame_id
            if not child.startswith('tag_'):
                continue
            
            # Извлекаем ID тега (убираем _stb, если есть)
            tag_id = child[:-4] if child.endswith('_stb') else child
            
            if tag_id not in self.tag_map:
                continue
            
            tag_x = tr.transform.translation.x
            tag_y = tr.transform.translation.y
            
            robot_pose = self.calculate_robot_pose(tag_id, tag_x, tag_y)
            if robot_pose:
                self.current_robot_x = robot_pose[0]
                self.current_robot_y = robot_pose[1]
                self.get_logger().info(f"Робот в карте: x={self.current_robot_x:.3f}, y={self.current_robot_y:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = TagLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()