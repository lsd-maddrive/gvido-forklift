#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import yaml
import os
import time

class RtabmapPriorUpdater(Node):
    def __init__(self):
        super().__init__('rtabmap_prior_updater')
        self.tag_map_path = os.path.expanduser('~/ros2_ws/tag_map.yaml')
        self.last_mtime = 0
        self.timer = self.create_timer(2.0, self.check_and_update)
        self.get_logger().info("Запущен обновлятор приоритетов RTAB-Map")
    
    def check_and_update(self):
        if not os.path.exists(self.tag_map_path):
            return
        
        mtime = os.path.getmtime(self.tag_map_path)
        if mtime == self.last_mtime:
            return
        
        self.last_mtime = mtime
        
        with open(self.tag_map_path, 'r') as f:
            data = yaml.safe_load(f)
        
        if not data or 'tags' not in data:
            return
        
        priors = []
        for tag_id, info in data['tags'].items():
            tag_num = tag_id.replace('tag_', '')
            x = info['translation']['x']
            y = info['translation']['y']
            priors.append(f"{tag_num} {x} {y} 0 0 0 0")
        
        priors_str = ' '.join(priors)
        
        # Обновляем параметр RTAB-Map
        client = self.create_client(rclpy.parameter_client.AsyncParameterClient, '/rtabmap')
        if client.wait_for_service(timeout_sec=1.0):
            future = client.set_parameters([Parameter('Marker/Priors', rclpy.Parameter.Type.STRING, priors_str)])
            rclpy.spin_once(self, timeout_sec=0.5)
            self.get_logger().info(f"Обновлены приоритеты: {priors_str}")
        else:
            self.get_logger().warn("RTAB-Map не отвечает")

def main(args=None):
    rclpy.init(args=args)
    node = RtabmapPriorUpdater()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
