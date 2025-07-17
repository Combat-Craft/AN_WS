import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge
import cv2
import sys
import threading


class MultiCameraPublisher(Node):
    def __init__(self, video_devices):
        super().__init__('multi_camera_publisher')

        self._publishers = []
        self._caps = []
        self._bridges = []
        self._window_names = []

        self._latest_gps = None  # To store last GPS fix
        self._gps_lock = threading.Lock()

        self.get_logger().info(f'Initializing cameras: {video_devices}')

        # Subscribe to GPS fixes on correct topic
        self.create_subscription(NavSatFix, '/gps_data', self.gps_callback, 10)

        # Publisher for republishing first camera image on '/camera/image_raw'
        self.generic_pub = self.create_publisher(Image, 'camera/image_raw', 10)

        for device in video_devices:
            pub = self.create_publisher(Image, f'camera{device}/image_raw', 10)
            self._publishers.append(pub)

            cap = cv2.VideoCapture(device)
            if not cap.isOpened():
                self.get_logger().error(f'Could not open video device {device}')
                self._caps.append(None)
                self._bridges.append(None)
                self._window_names.append(None)
                continue

            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 15)

            self._caps.append(cap)
            self._bridges.append(CvBridge())
            window_name = f'Camera {device}'
            self._window_names.append(window_name)

            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def gps_callback(self, msg: NavSatFix):
        with self._gps_lock:
            self._latest_gps = msg

    def timer_callback(self):
        for i, cap in enumerate(self._caps):
            if cap is None:
                continue

            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn(f'Failed to capture frame from device {i}')
                continue

            # Overlay GPS info if available
            with self._gps_lock:
                gps = self._latest_gps

            if gps is not None:
                gps_text = f'Lat: {gps.latitude:.6f}, Lon: {gps.longitude:.6f}, Alt: {gps.altitude:.2f}m'
                cv2.putText(frame, gps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0), 2, cv2.LINE_AA)

            window_name = self._window_names[i]
            if window_name:
                cv2.imshow(window_name, frame)

            msg = self._bridges[i].cv2_to_imgmsg(frame, encoding='bgr8')
            self._publishers[i].publish(msg)

            # Republish the first camera's image on '/camera/image_raw'
            if i == 0:
                self.generic_pub.publish(msg)

        # Call waitKey once per cycle
        cv2.waitKey(1)

    def destroy_node(self):
        self.get_logger().info('Releasing cameras and closing windows...')
        for cap in self._caps:
            if cap:
                cap.release()
        for window_name in self._window_names:
            if window_name:
                cv2.destroyWindow(window_name)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    video_devices = []
    for arg in sys.argv[1:]:
        try:
            video_devices.append(int(arg))
        except ValueError:
            pass

    if not video_devices:
        video_devices = [0, 1]

    node = MultiCameraPublisher(video_devices=video_devices)

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

