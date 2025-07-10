import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import re
import sys

class CameraPublisher(Node):
    def __init__(self, video_device=0):
        super().__init__(f'camera_publisher_{video_device}')

        #self.declare_parameter('video', 0)  # Default video device
        
        # Log the video device being used
        #video_device = self.get_parameter('video').value
        self.get_logger().info(f'Using video device: {video_device}')

        # Check if the video device is an integer or a string
        if isinstance(video_device, str):
            video_id = re.findall(r'\d+', video_device)
        elif isinstance(video_device, int):
            video_id = video_device
        else:
            self.get_logger().error('Invalid video device parameter. Must be an integer or a string representing a device.')
            exit()
        # Create a publisher for sensor_msgs/Image
        if video_id==0:
            self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        elif video_id==-1: 
            self.publisher_ = self.create_publisher(Image, 'cameram1/image_raw', 10)
        else:   
            self.publisher_ = self.create_publisher(Image, f'camera{video_id}/image_raw', 10)
        
        # Timer to control the publishing rate (30 Hz)
        timer_period = 1.0 / 30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # OpenCV capture
        self.cap = cv2.VideoCapture(video_device)  # Use 0 or another index if needed

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
    video_device = int(sys.argv[1]) if len(sys.argv) > 1 else 0
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