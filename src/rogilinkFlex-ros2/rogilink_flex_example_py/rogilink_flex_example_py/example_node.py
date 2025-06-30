import time
from ctypes import *

import rclpy
from rclpy.node import Node
from rogilink_flex_lib import Publisher, Subscriber


class RogiLinkFlexExampleNode(Node):
    def __init__(self):
        super().__init__('RogiLinkFlexExampleNode')
        self.get_logger().info("RogiLinkFlexExampleNode started")
        time.sleep(2)

    def callback(self, value):
        self.get_logger().info(f"Received: {value}")

def main():
    rclpy.init()
    node = RogiLinkFlexExampleNode()
    rclpy.spin(node)
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
    
        