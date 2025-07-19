#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import threading
import signal

class MultiCameraPublisher(Node):
    def __init__(self):
        super().__init__('multi_camera_h264_publisher')
        Gst.init(None)

        # Camera configurations
        self.cameras = {
            0: {'device': '/dev/video0', 'topic': '/cam0/h264'},
            1: {'device': '/dev/video2', 'topic': '/cam1/h264'},
            # 2: {'device': '/dev/video4', 'topic': '/cam2/h264'}, # Uncomment if needed
        }

        self.camera_publishers = {}
        self.pipelines = {}

        for cam_id, config in self.cameras.items():
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
        )

        pipeline = Gst.parse_launch(pipeline_str)
        sink = pipeline.get_by_name('sink')

        def on_new_sample(sink, data):
            return self._on_frame(sink, cam_id)

        sink.connect('new-sample', on_new_sample)
        pipeline.set_state(Gst.State.PLAYING)

        self.pipelines[cam_id] = pipeline
        self.get_logger().info(f"Camera {cam_id} ({device}) pipeline started")

    def _on_frame(self, sink, cam_id):
        sample = sink.emit('pull-sample')
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
        return Gst.FlowReturn.OK

    def shutdown(self):
        for pipeline in self.pipelines.values():
            pipeline.set_state(Gst.State.NULL)
        self.get_logger().info("Pipelines stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraPublisher()

    def shutdown_handler(signum, frame):
        node.get_logger().info("Shutting down node...")
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    rclpy.spin(node)

if __name__ == '__main__':
    main()