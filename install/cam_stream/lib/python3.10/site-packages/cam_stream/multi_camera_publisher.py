import rclpy
from rclpy.node import Node
from foxglove_msgs.msg import CompressedVideo
from std_msgs.msg import String
import gi
import glob
import numpy as np
import cv2
import json
from cv2 import aruco

gi.require_version('Gst', '1.0')
gi.require_version('GstBase', '1.0')
from gi.repository import Gst, GLib
import gi.repository.GstBase as GstBase


class MultiCameraPublisher(Node):
    def __init__(self):
        super().__init__('multi_camera_publisher')
        Gst.init(None)

        # ArUco setup
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        # Remove ArucoDetector initialization as we'll use direct functions
        
        # Detection interval parameter (in seconds)
        self.declare_parameter('detection_interval', 1.0)
        self.detection_interval = self.get_parameter('detection_interval').value

        self.cameras = {
            0: {'device': '/dev/video0', 'topic': '/cam0/h264'},
            1: {'device': '/dev/video2', 'topic': '/cam1/h264'},
        }

        self.camera_publishers = {}
        self.marker_publishers = {}  # New publishers for ArUco detection results
        self.pipelines = {}
        self.latest_frames = {}

        for cam_id, config in self.cameras.items():
            self.camera_publishers[cam_id] = self.create_publisher(
                CompressedVideo, config['topic'], 10
            )
            # Create marker detection result publisher
            self.marker_publishers[cam_id] = self.create_publisher(
                String, f"{config['topic']}/markers", 10
            )
            self.setup_pipeline(cam_id, config['device'])

        # Create timer for periodic ArUco detection
        self.detection_timer = self.create_timer(
            self.detection_interval, self.detect_markers
        )

    def setup_pipeline(self, cam_id, device):
        pipeline_str = (
            f"v4l2src device={device} ! "
            "video/x-raw,width=640,height=480,framerate=30/1 ! "
            "tee name=t ! queue ! "
            "videoconvert ! "
            "x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30 insert-vui=true ! "
            "video/x-h264,stream-format=byte-stream,alignment=au ! "
            "appsink name=sink emit-signals=True sync=false max-buffers=1 drop=true "
            "t. ! queue ! videoconvert ! video/x-raw,format=BGR ! "
            "appsink name=raw_sink emit-signals=True sync=false max-buffers=1 drop=true"
        )

        pipeline = Gst.parse_launch(pipeline_str)
        
        # Setup H264 sink
        sink = pipeline.get_by_name('sink')
        sink.connect('new-sample', lambda sink, cam=cam_id: self.on_new_sample(sink, cam))
        
        # Setup raw frame sink
        raw_sink = pipeline.get_by_name('raw_sink')
        raw_sink.connect('new-sample', lambda sink, cam=cam_id: self.on_raw_sample(sink, cam))

        pipeline.set_state(Gst.State.PLAYING)
        self.pipelines[cam_id] = pipeline
        self.get_logger().info(f"Started pipeline for camera {cam_id} ({device})")

    def on_raw_sample(self, sink, cam_id):
        sample = sink.emit('pull-sample')
        if not sample:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        caps = sample.get_caps()
        
        width = caps.get_structure(0).get_value('width')
        height = caps.get_structure(0).get_value('height')
        
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR

        # Convert buffer to numpy array for OpenCV processing
        frame = np.ndarray(
            shape=(height, width, 3),
            dtype=np.uint8,
            buffer=map_info.data
        )
        
        self.latest_frames[cam_id] = frame.copy()
        buf.unmap(map_info)
        return Gst.FlowReturn.OK

    def detect_markers(self):
        for cam_id, frame in self.latest_frames.items():
            if frame is not None:
                # Use the older detectMarkers API
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    frame,
                    self.aruco_dict,
                    parameters=self.aruco_params
                )
                
                if ids is not None:
                    # Create detection result message
                    result = {
                        'timestamp': self.get_clock().now().nanoseconds,
                        'camera_id': cam_id,
                        'markers': []
                    }
                    
                    for marker_id, corner in zip(ids.flatten(), corners):
                        center = corner[0].mean(axis=0)
                        result['markers'].append({
                            'id': int(marker_id),
                            'center_x': float(center[0]),
                            'center_y': float(center[1])
                        })
                    
                    # Publish detection results
                    msg = String()
                    msg.data = json.dumps(result)
                    self.marker_publishers[cam_id].publish(msg)
                    
                    self.get_logger().info(
                        f"Camera {cam_id}: Detected {len(ids)} markers"
                    )

    def on_new_sample(self, sink, cam_id):
        sample = sink.emit('pull-sample')
        if not sample:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR

        msg = CompressedVideo()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.format = 'h264'
        msg.data = map_info.data
        self.camera_publishers[cam_id].publish(msg)

        buf.unmap(map_info)
        return Gst.FlowReturn.OK

    def shutdown(self):
        for pipeline in self.pipelines.values():
            pipeline.set_state(Gst.State.NULL)


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
