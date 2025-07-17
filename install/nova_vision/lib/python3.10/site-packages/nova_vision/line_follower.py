from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class LineFollower(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Replace with your actual image topic
            self.listener_callback,
            10
            )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Receiving image data')
        twist = Twist()
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        crop_img = frame[60:120, 0:160]

        # Convert to grayscale
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

        # Gaussian blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Color thresholding
        ret, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)

        # Find the contours of the frame
        contours, hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

        # Find the biggest contour (if detected)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
            cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)

            cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 1)

            if cx >= 120:
                print("Turn Left!")
                twist.linear.x = 0.0
                twist.angular.z = 0.5  

            if 50 < cx < 120:
                print("On Track!")
                twist.linear.x = 0.5  # Move forward at 0.5 m/s
                twist.angular.z = 0.0  # No angular velocity

            if cx <= 50:
                print("Turn Right")
                twist.linear.x = 0.0
                twist.angular.z = -0.5  
        else:
            print("I don't see the line")
            twist.linear.x = 0.0
            twist.angular.z = 0.0  # No angular velocity

        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)
        
        # Publish the twist message to cmd_vel
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
