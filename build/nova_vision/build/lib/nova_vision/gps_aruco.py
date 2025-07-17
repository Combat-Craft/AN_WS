import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from cv_bridge import CvBridge
import cv2


class SensorDisplayNode(Node):
    def __init__(self):
        super().__init__('gps_display')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.br = CvBridge()
        
        # Create the ArUco detector
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100), 
                                                cv2.aruco.DetectorParameters())

        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',  # Topic for GPS data
            self.gps_callback,
            10
        )

        # Store latest GPS data
        self.current_latitude = None
        self.current_longitude = None
        self.current_altitude = None
        self.gps_status_string = "No GPS Fix"

    def gps_callback(self, msg):
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude
        self.current_altitude = msg.altitude
        self.gps_status_string = self.get_gps_status_string(msg.status.status)

    def get_gps_status_string(self, status):
        """Helper to get a human-readable string for GPS status."""
        if status == NavSatFix.STATUS_NO_FIX:
            return "NO_FIX"
        elif status == NavSatFix.STATUS_FIX:
            return "FIX"
        elif status == NavSatFix.STATUS_SBAS_FIX:
            return "SBAS_FIX"
        elif status == NavSatFix.STATUS_GBAS_FIX:
            return "GBAS_FIX"
        else:
            return "UNKNOWN"
        
    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        
        frame = self.br.imgmsg_to_cv2(data)
        
         # --- Overlay GPS data on the image ---
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        font_thickness = 2
        text_color = (0, 0, 0) # Black color (BGR)
       # background_color = (0, 0, 0) # Black background for text

        if self.current_latitude is not None and self.current_longitude is not None:
            text = f"Lat: {self.current_latitude:.6f}  Lon: {self.current_longitude:.6f}"
            status_text = f"Status: {self.gps_status_string}"
            text_pos = (10,30)
            status_pos = (10, 120)

            cv2.putText(frame, text, text_pos, font, font_scale, text_color, font_thickness, cv2.LINE_AA)
            cv2.putText(frame, status_text, status_pos, font, font_scale, text_color, font_thickness, cv2.LINE_AA)
        else:
            no_gps_text = "Waiting for GPS data..."
            cv2.putText(frame, no_gps_text, (10, 30), font, font_scale, (0, 0, 255), font_thickness, cv2.LINE_AA) # Red color

        #The Aruco section
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
    gps_display = SensorDisplayNode()
    rclpy.spin(gps_display)
    gps_display.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()