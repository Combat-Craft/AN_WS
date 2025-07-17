import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class MultiCameraSubscriber(Node):
    def __init__(self):
        super().__init__('multi_camera_subscriber')
        self.bridge = CvBridge()
        self.frames = {0: None, 1: None}#, 2: None

        self.create_subscription(CompressedImage, '/cam0/compressed', lambda msg: self.callback(msg, 0), 10)
        self.create_subscription(CompressedImage, '/cam1/compressed', lambda msg: self.callback(msg, 1), 10)
        #self.create_subscription(CompressedImage, '/cam2/compressed', lambda msg: self.callback(msg, 2), 10)
        self.get_logger().info('trying')
        self.timer = self.create_timer(0.05, self.display_frames)  # 20 Hz

    def callback(self, msg, cam_id):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            self.frames[cam_id] = cv2.resize(frame, (320, 240))
        except Exception as e:
            self.get_logger().warn(f'Error decoding camera {cam_id}: {e}')

    def display_frames(self):
        # Check if all frames are not None
        if all(frame is not None for frame in self.frames.values()):
            try:
                combined = np.hstack([self.frames[0], self.frames[1]])# Testing for 2: , self.frames[2]
                cv2.imshow("Three Camera Feeds", combined)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warn(f'Error displaying frames: {e}')
        else:
            self.get_logger().info('Waiting for all camera frames...')

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
