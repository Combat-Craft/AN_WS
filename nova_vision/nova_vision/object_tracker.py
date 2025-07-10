import cv2  # state of the art computer vision algorithms library
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Replace with your actual image topic
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.tracker = cv2.TrackerCSRT_create()
        self.regionSelected = False

    def listener_callback(self, msg):
        self.get_logger().info('Receiving image data')
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #program goes here
        if not self.regionSelected:
            bbox = cv2.selectROI("Select object", frame, fromCenter=False, showCrosshair=True)
            self.tracker.init(frame, bbox)
            self.regionSelected=True
        
        ret, bbox = self.tracker.update(frame)
        if ret:
            x,y,w,h=[int(v) for v in bbox]
            cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)
        else:
            cv2.putText(frame, "Tracking failed", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255), 2)

        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()