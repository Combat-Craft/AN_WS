import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.br = CvBridge()
        
        # Create the ArUco detector
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100), 
                                                cv2.aruco.DetectorParameters())


    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        frame = self.br.imgmsg_to_cv2(data)
        #THIS IS WHERE YOU PUT THE PROGRAM
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect the markers
        corners, marker_ids, rejected = self.detector.detectMarkers(gray)

        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)
      
      # write the number on the frame
        for i, marker_id in enumerate(marker_ids):
            marker_textlocn = corners[i][0][1]
            m_loc = int(marker_textlocn[0]), int(marker_textlocn[1])
            marker_msg=f'ID:{marker_id}'
            cv2.putText(frame, marker_msg, m_loc, cv2.FONT_HERSHEY_PLAIN,1, (0, 255, 255), 2)
        
        cv2.imshow("camera", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()