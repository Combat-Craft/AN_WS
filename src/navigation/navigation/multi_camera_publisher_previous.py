import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import re
import sys

class CameraPublisher(Node):
    def __init__(self, video_device=0):
        super().__init__('camera_publisher')

        self.get_logger().info(f'Using video device: {video_device}')

        # Determine video_id for topic naming
        if isinstance(video_device, str):
            video_id_match = re.findall(r'\d+', video_device)
            video_id = int(video_id_match[0]) if video_id_match else -1
        elif isinstance(video_device, int):
            video_id = video_device
        else:
            self.get_logger().error('Invalid video device parameter. Must be int or str.')
            exit()

        # Create a publisher for sensor_msgs/Image
        if video_id == -2:
            topic_name = 'camera/image_raw'
        elif video_id == -1:
            topic_name = 'cameram1/image_raw'
        else:
            topic_name = f'camera{video_id}/image_raw'

        self.publisher_ = self.create_publisher(Image, topic_name, 10)

        # Timer to control the publishing rate (30 Hz)
        timer_period = 1.0 / 30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # OpenCV capture setup
        self.cap = cv2.VideoCapture(video_device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 15)

        if not self.cap.isOpened():
            self.get_logger().error('Could not open video device')
            exit()

        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    # Safer camera device parsing (ignore ROS 2 launch arguments)
    video_device = 0  # Default
    for arg in sys.argv:
        if arg.isdigit():
            video_device = int(arg)

    rclpy.init(args=args)
    node = CameraPublisher(video_device=video_device)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
