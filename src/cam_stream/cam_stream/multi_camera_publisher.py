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
        super().__init__('multi_camera_publisher')
        Gst.init(None)

        self.cameras = {
            0: {'device': '/dev/video0', 'topic': '/cam0/compressed'},
            1: {'device': '/dev/video2', 'topic': '/cam1/compressed'},
            #2: {'device': '/dev/video4', 'topic': '/cam2/compressed'},
        }

        self.camera_publishers = {}  # Changed from publishers to camera_publishers
        self.pipelines = {}

        for cam_id, config in self.cameras.items():
            self.camera_publishers[cam_id] = self.create_publisher(CompressedImage, config['topic'], 10)
            self.setup_pipeline(cam_id, config['device'])
            

        self.get_logger().info('Starting')

    def setup_pipeline(self, cam_id, device):

        pipeline_str = (
            f"v4l2src device={device} ! "
            f"image/jpeg,width=640,height=480,framerate=30/1 ! "
            "jpegparse ! appsink name=sink emit-signals=True"
        )
        self.get_logger().info(f"Starting pipeline for cam {cam_id} ({device})")
        pipeline = Gst.parse_launch(pipeline_str)
        sink = pipeline.get_by_name('sink')
        
	    
        def callback_wrapper(sink):
            return self.on_new_sample(sink, cam_id)

        sink.connect('new-sample', callback_wrapper)
        self.pipelines[cam_id] = pipeline
        pipeline.set_state(Gst.State.PLAYING)
        bus = pipeline.get_bus()
        bus.add_signal_watch()
        def on_bus_message(bus, message):
            if message.type == Gst.MessageType.ERROR:
                err, debug = message.parse_error()
                self.get_logger().error(f"GStreamer error cam {cam_id} ({device}): {err} — {debug}")
        bus.connect("message", on_bus_message)


    def on_new_sample(self, sink, cam_id):
        sample = sink.emit('pull-sample')
        if sample:
            buffer = sample.get_buffer()
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if success:
                try:
                    self.get_logger().info(f"Publishing frame from camera {cam_id}")
                    msg = CompressedImage()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.format = "jpeg"
                    msg.data = bytes(map_info.data)
                    self.camera_publishers[cam_id].publish(msg)  # Updated reference
                finally:
                    buffer.unmap(map_info)
        return Gst.FlowReturn.OK

    def shutdown(self):
        for pipeline in self.pipelines.values():
            pipeline.set_state(Gst.State.NULL)
        self.get_logger().info("Camera pipelines shut down.")

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraPublisher()
    loop = GLib.MainLoop()

    def handle_sigint(signum, frame):
        print("shut down")
        loop.quit()

    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()

    try:
        loop.run()
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join()

if __name__ == '__main__':
    main()
