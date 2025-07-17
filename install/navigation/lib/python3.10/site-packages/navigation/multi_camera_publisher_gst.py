#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import threading
import signal

class Multi_Camera_Publisher_Gst(Node):
    def __init__(self):
        super().__init__('multi_camera_publisher_gst')
        Gst.init(None)
        
        self.publisher = self.create_publisher(CompressedImage, 'camera/compressed', 10)
        self.sink = None
        self.setup_pipeline()
        self.get_logger().info('Single-Camera Publisher Node started')
        self.get_logger().info("bruhski1")

    def setup_pipeline(self):
        # Simplified single-camera pipeline with JPEG compression
        pipeline_str = (
        "v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! "
        "nvjpegenc ! appsink name=sink emit-signals=True"
        )
        self.get_logger().info("bruhski2")
        # Remove extra whitespace and newlines
        pipeline_str = ' '.join(pipeline_str.split())
        self.get_logger().info(f'Pipeline: {pipeline_str}')
        self.get_logger().info("bruhski3")
        
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.sink = self.pipeline.get_by_name('sink')
        self.sink.connect("new-sample", self.on_new_sample)
        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info("bruhski4")

    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample:
            buffer = sample.get_buffer()
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if success:
                try:
                    msg = CompressedImage()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.format = "jpeg"
                    msg.data = bytes(map_info.data)
                    self.publisher.publish(msg)
                finally:
                    buffer.unmap(map_info)
        return Gst.FlowReturn.OK

def main(args=None):
    rclpy.init(args=args)
    node = Multi_Camera_Publisher_Gst()
    
    loop = GLib.MainLoop()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    
    def shutdown_handler(signum, frame):
    	print("shut yo ahhh")
    	loop.quit()
    	
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)
    
    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    
    loop.quit()
    node.pipeline.set_state(Gst.State.NULL)
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
