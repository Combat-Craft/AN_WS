import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
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

        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps_data',
            self.gps_callback,
            10
        )

        self.current_latitude = None
        self.current_longitude = None
        self.current_altitude = None
        self.gps_status_string = "No GPS Fix"

        # To store the last valid GPS fix
        self.last_known = "Waiting for GPS data..."

    def gps_callback(self, msg):
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude
        self.current_altitude = msg.altitude
        self.gps_status_string = self.get_gps_status_string(msg.status.status)

        # Only update last_known if there is a valid fix
        if msg.status.status != -1:
            self.last_known = f"Lat: {self.current_latitude:.6f}  Lon: {self.current_longitude:.6f}"

    def get_gps_status_string(self, status):
        if status == -1:
            return "NO_FIX"
        elif status == 0:
            return "FIX"
        elif status == 1:
            return "SBAS_FIX"
        elif status == 2:
            return "GBAS_FIX"
        else:
            return "UNKNOWN"

    def listener_callback(self, data):
        # self.get_logger().info('Receiving video frame')

        frame = self.br.imgmsg_to_cv2(data)

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        font_thickness = 2
        text_color = (0, 0, 0)  # Black

        text_pos = (10, 30)
        status_pos = (10, 120)

        # Use last known valid GPS text
        cv2.putText(frame, self.last_known, text_pos, font, font_scale,
                    text_color, font_thickness, cv2.LINE_AA)

        status_text = f"Status: {self.gps_status_string}"
        cv2.putText(frame, status_text, status_pos, font, font_scale,
                    text_color, font_thickness, cv2.LINE_AA)

        cv2.imshow("camera with GPS", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    gps_display = SensorDisplayNode()
    rclpy.spin(gps_display)
    gps_display.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

