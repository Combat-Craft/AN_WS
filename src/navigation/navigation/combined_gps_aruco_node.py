import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge
import cv2
import os
import time

class CombinedGpsArucoNode(Node):
    def __init__(self, video_device=0, show_window=True):
        super().__init__('combined_gps_aruco_node')

        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps_data',
            self.gps_callback,
            10
        )

        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.br = CvBridge()

        self.cap = cv2.VideoCapture(video_device)
        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open video device {video_device}')
            raise RuntimeError(f'Could not open video device {video_device}')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 15)

        self.show_window = show_window

        # GPS info
        self.current_latitude = None
        self.current_longitude = None
        self.current_altitude = None
        self.gps_status_string = "No GPS Fix"
        self.last_known_gps_text = "Waiting for GPS data..."

        # Logging
        log_dir = '/home/bao/AN_WS/logs'
        os.makedirs(log_dir, exist_ok=True)
        self.log_file_path = os.path.join(log_dir, 'combined_node.log')

        self.last_logged_aruco_ids = set()
        self.last_aruco_log_time = 0.0

        self.last_logged_gps_text = ""
        self.last_gps_log_time = 0.0

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.get_logger().info('Combined GPS + ArUco node started')

    def gps_callback(self, msg: NavSatFix):
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude
        self.current_altitude = msg.altitude
        self.gps_status_string = self.get_gps_status_string(msg.status.status)

        if msg.status.status != -1:
            self.last_known_gps_text = f"Lat: {self.current_latitude:.6f} Lon: {self.current_longitude:.6f} Alt: {self.current_altitude:.2f}m"

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

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, marker_ids, _ = self.detector.detectMarkers(gray)

        # Draw markers if detected
        detected_ids = set()
        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)
            detected_ids = {int(id[0]) for id in marker_ids}
            for i, marker_id in enumerate(marker_ids):
                corner = corners[i][0][0]
                pos = (int(corner[0]), int(corner[1]))
                text = f'ID: {marker_id[0]}'
                cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 255), 2, cv2.LINE_AA)

        now = time.time()

        # Log ArUco IDs if changed or every 5 sec if still same
        if detected_ids != self.last_logged_aruco_ids or (now - self.last_aruco_log_time) > 5.0:
            if detected_ids:
                msg = f'ArUco IDs detected: {sorted(detected_ids)} at {time.strftime("%Y-%m-%d %H:%M:%S")}'
                self.get_logger().info(msg)
                self.write_log(msg)
                self.last_aruco_log_time = now
                self.last_logged_aruco_ids = detected_ids

        # Overlay GPS text on frame
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        thickness = 2
        text_color_gps = (0, 255, 0)
        text_color_status = (0, 0, 0)

        cv2.putText(frame, self.last_known_gps_text, (10, 30), font, font_scale, text_color_gps, thickness, cv2.LINE_AA)
        status_text = f"Status: {self.gps_status_string}"
        cv2.putText(frame, status_text, (10, 60), font, font_scale, text_color_status, thickness, cv2.LINE_AA)

        # Log GPS if text changed or every 5 seconds
        if self.last_known_gps_text != self.last_logged_gps_text or (now - self.last_gps_log_time) > 5.0:
            if self.last_known_gps_text != "Waiting for GPS data...":
                gps_log_msg = f"GPS Fix: {self.last_known_gps_text} Status: {self.gps_status_string} at {time.strftime('%Y-%m-%d %H:%M:%S')}"
                self.get_logger().info(gps_log_msg)
                self.write_log(gps_log_msg)
                self.last_logged_gps_text = self.last_known_gps_text
                self.last_gps_log_time = now

        # Publish annotated image
        img_msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(img_msg)

        # Show window if enabled
        if self.show_window:
            cv2.imshow('Camera with GPS + ArUco Overlay', frame)
            cv2.waitKey(1)

    def write_log(self, message):
        try:
            with open(self.log_file_path, 'a') as f:
                f.write(message + '\n')
        except Exception as e:
            self.get_logger().error(f"Failed to write to log file: {e}")

    def destroy_node(self):
        self.get_logger().info('Releasing camera and closing windows...')
        if self.cap:
            self.cap.release()
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    video_device = 0
    show_window = True

    import sys
    if len(sys.argv) > 1:
        try:
            video_device = int(sys.argv[1])
        except ValueError:
            pass

    node = CombinedGpsArucoNode(video_device=video_device, show_window=show_window)

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

