#!/usr/bin/env python3

# scripts/save_map_on_demand.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image
import os
from datetime import datetime

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver_wrapper')
        self.declare_parameter('save_folder', os.path.expanduser('~/slam_maps'))
        self.save_folder = self.get_parameter('save_folder').value
        os.makedirs(self.save_folder, exist_ok=True)

        self.sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.get_logger().info(f'Watching /map and saving to {self.save_folder}')

    def map_cb(self, msg: OccupancyGrid):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        fname = os.path.join(self.save_folder, f'map_{ts}.png')
        yaml_fname = os.path.join(self.save_folder, f'map_{ts}.yaml')

        w = msg.info.width
        h = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        # convert occupancy to image (0 free -> 255 white; 100 occ -> 0 black; -1 unknown -> 127)
        img = np.zeros((h, w), dtype=np.uint8)
        img[data == -1] = 127
        img[data == 0] = 255
        img[data >= 65] = 0   # threshold for occupied
        Image.fromarray(img).transpose(Image.FLIP_TOP_BOTTOM).save(fname)

        # yaml with resolution and origin
        yaml_txt = f"""image: {os.path.basename(fname)}
resolution: {msg.info.resolution}
origin: [{msg.info.origin.position.x}, {msg.info.origin.position.y}, {msg.info.origin.orientation.z}]
"""
        with open(yaml_fname, 'w') as f:
            f.write(yaml_txt)

        self.get_logger().info(f'Saved {fname} and {yaml_fname}')

def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
