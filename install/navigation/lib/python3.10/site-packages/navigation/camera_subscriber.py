import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        # Subscribing to the main camera topic '/camera/image_raw'
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)

        self.bridge = CvBridge()
        self.get_logger().info('Camera subscriber initialized and listening to /camera/image_raw')

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Show the frame
            cv2.imshow('Received Camera Feed', frame)
            cv2.waitKey(1)  # Required for imshow to work properly in OpenCV

        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def destroy_node(self):
        self.get_logger().info('Shutting down camera subscriber...')
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
