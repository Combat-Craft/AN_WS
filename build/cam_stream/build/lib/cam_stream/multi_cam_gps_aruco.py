import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import gi
from cv_bridge import CvBridge
from sensor_msgs.msg import NavSatFix
import time
import os
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import threading


class MultiCameraSubscriber(Node):
    def __init__(self):
        super().__init__('multi_camera_subscriber')
        Gst.init(None)

        # Define camera topics
        self.cameras = {
            0: '/cam0/h264',
            1: '/cam1/h264',
            #2: '/cam2/h264',
        }

        self.br = CvBridge()
        self.pipelines = {}
        self.appsrcs = {}
        self.appsinks = {}
        self.frames = {0: None, 1: None}#, 2: None


        for cam_id, topic in self.cameras.items():
            self.setup_pipeline(cam_id)
            self.create_subscription(CompressedImage, topic, lambda msg, cid=cam_id: self.image_callback(msg, cid), 10)


        # OpenCV display loop in background
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()

        self.last_aruco_log_time = 0
        self.last_logged_aruco_ids = set()
        self.last_gps_log_time = 0
        self.last_logged_gps_text = ""
        self.log_file_path = os.path.join(os.path.expanduser("~"), "camera_log.txt")

       
        # GPS data
        self.current_latitude = None
        self.current_longitude = None
        self.current_altitude = None
        self.gps_status_string = "No GPS Fix"
        self.last_known_gps_text = "Waiting for GPS data..."

        # ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # GPS subscriber
        self.create_subscription(NavSatFix, '/gps_data', self.gps_callback, 10)

        self.get_logger().info('Multi-camera H264 subscriber started.')


    def setup_pipeline(self, cam_id):
        decoder = "nvh264dec ! videoconvert" if self.has_cuda() else "avdec_h264 ! videoconvert"
        pipeline_str = (
            f"appsrc name=mysrc_{cam_id} is-live=true block=true format=time ! "
            f"h264parse ! {decoder} ! "
            "video/x-raw,format=BGR ! "
            f"appsink name=mysink_{cam_id} emit-signals=true sync=false max-buffers=1 drop=true"
        )
        pipeline = Gst.parse_launch(pipeline_str)
        appsrc = pipeline.get_by_name(f"mysrc_{cam_id}")
        appsink = pipeline.get_by_name(f"mysink_{cam_id}")
        pipeline.set_state(Gst.State.PLAYING)

        self.pipelines[cam_id] = pipeline
        self.appsrcs[cam_id] = appsrc
        self.appsinks[cam_id] = appsink
        self.get_logger().info(f"Pipeline started for camera {cam_id}")

    def has_cuda(self):
        return bool(Gst.Registry.get().find_plugin("nvh264dec"))

    def write_log(self, message):
        try:
            with open(self.log_file_path, 'a') as f:
                f.write(message + '\n')
        except Exception as e:
            self.get_logger().error(f"Failed to write to log file: {e}")


    def image_callback(self, msg, cam_id):
        try:
            # Push H264 encoded data into GStreamer
            buf = Gst.Buffer.new_allocate(None, len(msg.data), None)
            buf.fill(0, msg.data)
            buf.pts = buf.dts = int(time.time() * Gst.SECOND)
            self.appsrcs[cam_id].emit('push-buffer', buf)

            # Pull frame from appsink
            sample = self.appsinks[cam_id].emit('try-pull-sample', Gst.SECOND)
            if sample:
                buf = sample.get_buffer()
                caps = sample.get_caps()
                arr = buf.extract_dup(0, buf.get_size())
                w = caps.get_structure(0).get_value('width')
                h = caps.get_structure(0).get_value('height')
                frame = np.frombuffer(arr, dtype=np.uint8).reshape((h, w, 3))

                self.frames[cam_id] = frame.copy()

                # ArUco detection
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, marker_ids, _ = self.detector.detectMarkers(gray)

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

                # GPS overlay
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(frame, self.last_known_gps_text, (10, 30), font, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
                status_text = f"Status: {self.gps_status_string}"
                cv2.putText(frame, status_text, (10, 60), font, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

                # Logging
                now = time.time()
                if detected_ids != self.last_logged_aruco_ids or (now - self.last_aruco_log_time) > 5.0:
                    if detected_ids:
                        msg = f'Cam {cam_id} ArUco IDs: {sorted(detected_ids)} at {time.strftime("%Y-%m-%d %H:%M:%S")}'
                        self.get_logger().info(msg)
                        self.write_log(msg)
                        self.last_aruco_log_time = now
                        self.last_logged_aruco_ids = detected_ids

                if self.last_known_gps_text != self.last_logged_gps_text or (now - self.last_gps_log_time) > 5.0:
                    if self.last_known_gps_text != "Waiting for GPS data...":
                        gps_log_msg = f"GPS Fix: {self.last_known_gps_text} Status: {self.gps_status_string} at {time.strftime('%Y-%m-%d %H:%M:%S')}"
                        self.get_logger().info(gps_log_msg)
                        self.write_log(gps_log_msg)
                        self.last_logged_gps_text = self.last_known_gps_text
                        self.last_gps_log_time = now

        except Exception as e:
            self.get_logger().error(f"Error in image_callback for camera {cam_id}: {e}")


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


    def display_loop(self):
        frame_counter=0
        while rclpy.ok():
            frame_counter +=1
            if all(frame is not None for frame in self.frames.values()):
                try:
                    resized = [cv2.resize(self.frames[cid], (320, 240)) for cid in self.cameras.keys()]
                    combined = cv2.hconcat(resized)
                    cv2.imshow("Multi-Camera H264 Feed", combined)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                except Exception as e:
                    self.get_logger().warn(f"Display error: {e}")
            else:
                if frame_counter%100000==0: print(f"Waiting for frames...{frame_counter} ")
                pass
        cv2.destroyAllWindows()


    def destroy_node(self):
        for pipeline in self.pipelines.values():
            pipeline.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


