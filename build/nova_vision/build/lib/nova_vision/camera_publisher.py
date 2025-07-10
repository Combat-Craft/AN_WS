import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Create a publisher for sensor_msgs/Image
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        
        # Timer to control the publishing rate (30 Hz)
        timer_period = 1.0 / 30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # OpenCV capture
        self.cap = cv2.VideoCapture(0)  # Use 0 or another index if needed

        if not self.cap.isOpened():
            self.get_logger().error('Could not open video device')
            exit()

        # CvBridge for converting OpenCV images to ROS messages
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        # Convert OpenCV image (BGR) to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()