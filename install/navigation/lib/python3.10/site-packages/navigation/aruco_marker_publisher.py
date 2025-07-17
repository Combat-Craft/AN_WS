import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import os
import time

class ArUcoMarkerPublisher(Node):
    def __init__(self, video_device=0, show_window=False):
        super().__init__('aruco_marker_publisher')

        self.publisher_ = self.create_publisher(Image, 'camera/aruco_image', 10)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        self.br = CvBridge()

        self.cap = cv2.VideoCapture(video_device)
        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open video device {video_device}')
            sys.exit(1)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 15)

        self.show_window = show_window

        self.last_detected_ids = set()
        self.last_log_time = time.time()

        # Set fixed log path
        log_dir = '/home/bao/AN_WS/sensorlogs'
        os.makedirs(log_dir, exist_ok=True)
        self.log_file_path = os.path.join(log_dir, 'aruco_detections.log')
        self.get_logger().info(f'AruCo detections will be logged to {self.log_file_path}')

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, marker_ids, rejected = self.detector.detectMarkers(gray)

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
        if detected_ids != self.last_detected_ids or (now - self.last_log_time) > 5.0:
            if detected_ids:
                log_msg = f'Detected ArUco IDs: {sorted(detected_ids)} at {time.strftime("%Y-%m-%d %H:%M:%S")}'
                self.get_logger().info(log_msg)
                self.write_to_log_file(log_msg)
                self.last_log_time = now
                self.last_detected_ids = detected_ids

        img_msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(img_msg)

        if self.show_window:
            cv2.imshow('ArUco Detection', frame)
            cv2.waitKey(1)

    def write_to_log_file(self, msg):
        try:
            with open(self.log_file_path, 'a') as f:
                f.write(msg + '\n')
        except Exception as e:
            self.get_logger().error(f'Failed to write to ArUco log file: {e}')

    def destroy_node(self):
        self.get_logger().info('Releasing camera and closing window...')
        if self.cap:
            self.cap.release()
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    video_device = 0
    show_window = True

    if len(sys.argv) > 1:
        try:
            video_device = int(sys.argv[1])
        except ValueError:
            pass

    if 'ROS_NAMESPACE' in os.environ:
        show_window = False

    node = ArUcoMarkerPublisher(video_device=video_device, show_window=show_window)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

