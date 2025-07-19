import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import gi
from functools import partial
gi.require_version('Gst', '1.0')
<<<<<<< HEAD
from gi.repository import Gst, GLib

import threading
import signal
=======
from gi.repository import Gst
>>>>>>> refs/remotes/origin/main

class MultiCameraPublisher(Node):
    def __init__(self):
        super().__init__('multi_camera_h264_publisher')
        Gst.init(None)

<<<<<<< HEAD
        # Camera configurations
=======
<<<<<<< HEAD
        self.camera_publishers = {}
=======
>>>>>>> refs/remotes/origin/main
        self.cameras = {
            0: {'device': '/dev/video0', 'topic': '/cam0/h264'},
            1: {'device': '/dev/video2', 'topic': '/cam1/h264'},
            # 2: {'device': '/dev/video4', 'topic': '/cam2/h264'}, # Uncomment if needed
        }

<<<<<<< HEAD
        self.camera_publishers = {}
=======
        self.camera_publishers = {}  # Changed from publishers to camera_publishers
>>>>>>> refs/remotes/origin/main
>>>>>>> refs/remotes/origin/main
        self.pipelines = {}
        self.cameras = {
            0: {'device': 'videotestsrc pattern=ball', 'topic': '/cam0/h264'},
            1: {'device': 'videotestsrc pattern=pinwheel', 'topic': '/cam1/h264'},
        }

        for cam_id, config in self.cameras.items():
<<<<<<< HEAD
            topic = config['topic']
            device = config['device']
            self.camera_publishers[cam_id] = self.create_publisher(CompressedImage, topic, 10)
            self._start_pipeline(cam_id, device)

        self.get_logger().info('H264 multi-camera publisher running.')

    def _start_pipeline(self, cam_id, device):
        pipeline_str = (
            f"v4l2src device={device} ! "
            f"video/x-raw,width=640,height=480,framerate=30/1 ! "
            f"videoconvert ! "
            f"x264enc tune=zerolatency bitrate=800 speed-preset=ultrafast ! "
            f"video/x-h264,stream-format=byte-stream ! "
            f"appsink name=sink emit-signals=True sync=false"
=======
            self.camera_publishers[cam_id] = self.create_publisher(CompressedImage, config['topic'], 10)
            self.setup_pipeline(cam_id, config['device'])
<<<<<<< HEAD
=======
            

        self.get_logger().info('Starting')
>>>>>>> refs/remotes/origin/main

    def setup_pipeline(self, cam_id, device):
        pipeline_str = (
            f"{device} ! "
            "video/x-raw,width=640,height=480,framerate=15/1 ! "
            "videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=ultrafast ! "
            "video/x-h264,stream-format=byte-stream ! "
            "appsink name=sink emit-signals=True sync=false max-buffers=1 drop=true"
>>>>>>> refs/remotes/origin/main
        )

        pipeline = Gst.parse_launch(pipeline_str)
        sink = pipeline.get_by_name('sink')
<<<<<<< HEAD

        def on_new_sample(sink, data):
            return self._on_frame(sink, cam_id)

        sink.connect('new-sample', on_new_sample)
        pipeline.set_state(Gst.State.PLAYING)

        self.pipelines[cam_id] = pipeline
        self.get_logger().info(f"Camera {cam_id} ({device}) pipeline started")
=======
        sink.connect('new-sample', lambda sink: self.on_new_sample(sink, cam_id))

        pipeline.set_state(Gst.State.PLAYING)
        self.pipelines[cam_id] = pipeline
        self.get_logger().info(f"Started pipeline for camera {cam_id}")
>>>>>>> refs/remotes/origin/main

    def _on_frame(self, sink, cam_id):
        sample = sink.emit('pull-sample')
<<<<<<< HEAD
        if sample:
            buf = sample.get_buffer()
            success, map_info = buf.map(Gst.MapFlags.READ)
            if success:
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format = "h264"
                msg.data = map_info.data.tobytes()
                self.publishers[cam_id].publish(msg)
                buf.unmap(map_info)
=======
        if not sample:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        success, map_info = buf.map(Gst.MapFlags.READ)
        if success:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "h264"
            msg.data = map_info.data
            self.camera_publishers[cam_id].publish(msg)
            buf.unmap(map_info)

>>>>>>> refs/remotes/origin/main
        return Gst.FlowReturn.OK

    def shutdown(self):
        for pipeline in self.pipelines.values():
            pipeline.set_state(Gst.State.NULL)
<<<<<<< HEAD
        self.get_logger().info("Pipelines stopped.")
=======
>>>>>>> refs/remotes/origin/main

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraPublisher()
<<<<<<< HEAD

    def shutdown_handler(signum, frame):
        node.get_logger().info("Shutting down node...")
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    rclpy.spin(node)
=======
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
<<<<<<< HEAD
=======
        thread.join()
>>>>>>> refs/remotes/origin/main

if __name__ == '__main__':
    main()
>>>>>>> refs/remotes/origin/main
