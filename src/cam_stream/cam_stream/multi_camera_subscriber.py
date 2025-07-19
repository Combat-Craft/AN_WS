#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class MultiCameraH264Subscriber(Node):
    def __init__(self):
        super().__init__('multi_camera_subscriber')
        self.bridge = CvBridge()
        self.frames = {0: None, 1: None, 2: None}#

        self.create_subscription(CompressedImage, '/cam0/compressed', lambda msg: self.callback(msg, 0), 10)
        self.create_subscription(CompressedImage, '/cam1/compressed', lambda msg: self.callback(msg, 1), 10)
        self.create_subscription(CompressedImage, '/cam2/compressed', lambda msg: self.callback(msg, 2), 10)
        self.get_logger().info('trying')
        self.timer = self.create_timer(0.05, self.display_frames)  # 20 Hz

    def callback(self, msg, cam_id):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                self.frames[cam_id] = cv2.resize(frame, (320, 240))
            else:
                self.get_logger().warn(f"Failed to decode H264 frame for cam {cam_id}")
        except Exception as e:
            self.get_logger().warn(f"Exception decoding cam {cam_id}: {e}")

    def display_frames(self):
        # Check if all frames are not None
        if all(frame is not None for frame in self.frames.values()):
            try:
                combined = np.hstack([self.frames[0], self.frames[1], self.frames[2]])# Testing for 2
                cv2.imshow("Three Camera Feeds", combined)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warn(f'Error displaying frames: {e}')
        else:
            self.get_logger().info("Waiting for frames...")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraH264Subscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()