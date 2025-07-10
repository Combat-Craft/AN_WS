#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class Multi_Camera_Subscriber_Gst(Node):
    def __init__(self):
        super().__init__('multi_camera_subscriber_gst')
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            CompressedImage,
            'camera/compressed',
            self.image_callback,
            10)
        
        self.window_name = 'Combined Camera View'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 640, 480)
        
        self.get_logger().info('Remote Viewer Node started')
    
    def image_callback(self, msg):
        self.get_logger().info("I am Crashing out")
        frame = self.bridge.compressedimgmsg_to_cv2(msg, "bgr8")
        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(1)


    def __del__(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = Multi_Camera_Subscriber_Gst()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
