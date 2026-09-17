#!/usr/bin/env python3
import yaml
import os

def convert():
    tag_map_path = os.path.expanduser('~/ros2_ws/tag_map.yaml')
    
    if not os.path.exists(tag_map_path):
        print("")
        return
    
    with open(tag_map_path, 'r') as f:
        data = yaml.safe_load(f)
    
    if not data or 'tags' not in data:
        print("")
        return
    
    priors = []
    for tag_id, info in data['tags'].items():
        tag_num = tag_id.replace('tag_', '')
        x = info['translation']['x']
        y = info['translation']['y']
        priors.append(f"{tag_num} {x} {y} 0 0 0 0")
    
    print(' '.join(priors))

if __name__ == '__main__':
    convert()
