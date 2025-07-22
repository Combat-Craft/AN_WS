import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge
import cv2
import sys
import threading
import os


class MultiCameraPublisher(Node):
    def __init__(self, video_sources):
        super().__init__('multi_camera_publisher')

        self._publishers = []
        self._caps = []
        self._bridges = []
        self._window_names = []

        self._latest_gps = None  # To store last GPS fix
        self._gps_lock = threading.Lock()

        self.get_logger().info(f'Initializing video sources: {video_sources}')

        # Subscribe to GPS fixes on correct topic
        self.create_subscription(NavSatFix, '/gps_data', self.gps_callback, 10)

        # Publisher for republishing first camera image on '/camera/image_raw'
        self.generic_pub = self.create_publisher(Image, 'camera/image_raw', 10)

        for i, source in enumerate(video_sources):
            pub = self.create_publisher(Image, f'camera{i}/image_raw', 10)
            self._publishers.append(pub)

            # Try to open as file first, then as device
            if isinstance(source, str) and os.path.exists(source):
                self.get_logger().info(f'Opening video file: {source}')
                cap = cv2.VideoCapture(source)
            else:
                self.get_logger().info(f'Opening video device: {source}')
                cap = cv2.VideoCapture(source)
            
            if not cap.isOpened():
                self.get_logger().error(f'Could not open video source {source}')
                self._caps.append(None)
                self._bridges.append(None)
                self._window_names.append(None)
                continue

            # Set properties for device sources only
            if isinstance(source, int):
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, 15)

            self._caps.append(cap)
            self._bridges.append(CvBridge())
            window_name = f'Video Source {i}: {source}'
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
                # For video files, restart from beginning
                if isinstance(cap, cv2.VideoCapture):
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, frame = cap.read()
                
                if not ret:
                    self.get_logger().warn(f'Failed to capture frame from source {i}')
                    continue

            # Overlay GPS info if available
            with self._gps_lock:
                gps = self._latest_gps

            if gps is not None:
                gps_text = f'Lat: {gps.latitude:.6f}, Lon: {gps.longitude:.6f}, Alt: {gps.altitude:.2f}m'
                cv2.putText(frame, gps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0), 2, cv2.LINE_AA)

            # Add source info overlay
            source_text = f'Source {i}'
            cv2.putText(frame, source_text, (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 2, cv2.LINE_AA)

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
        self.get_logger().info('Releasing video sources and closing windows...')
        for cap in self._caps:
            if cap:
                cap.release()
        for window_name in self._window_names:
            if window_name:
                cv2.destroyWindow(window_name)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    video_sources = []
    
    # Parse command line arguments
    for arg in sys.argv[1:]:
        if arg.startswith('--file='):
            # File path argument
            file_path = arg.split('=', 1)[1]
            video_sources.append(file_path)
        else:
            # Try to parse as device number
            try:
                video_sources.append(int(arg))
            except ValueError:
                # Try as file path
                if os.path.exists(arg):
                    video_sources.append(arg)

    # Default sources if none specified
    if not video_sources:
        # Try test.avi first, then fallback to devices
        test_file = '/home/TASC/AN_WS/test.avi'
        if os.path.exists(test_file):
            video_sources = [test_file]
            print(f"Using test video file: {test_file}")
        else:
            video_sources = [0, 1]
            print("Using default camera devices")

    node = MultiCameraPublisher(video_sources=video_sources)

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
