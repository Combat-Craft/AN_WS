import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import gi
from functools import partial
gi.require_version('Gst', '1.0')
from gi.repository import Gst

class MultiCameraPublisher(Node):
    def __init__(self):
        super().__init__('multi_camera_publisher')
        Gst.init(None)

<<<<<<< HEAD
        self.camera_publishers = {}
=======
        self.cameras = {
            0: {'device': '/dev/video0', 'topic': '/cam0/compressed'},
            1: {'device': '/dev/video2', 'topic': '/cam1/compressed'},
            #2: {'device': '/dev/video4', 'topic': '/cam2/compressed'},
        }

        self.camera_publishers = {}  # Changed from publishers to camera_publishers
>>>>>>> refs/remotes/origin/main
        self.pipelines = {}
        self.cameras = {
            0: {'device': 'videotestsrc pattern=ball', 'topic': '/cam0/h264'},
            1: {'device': 'videotestsrc pattern=pinwheel', 'topic': '/cam1/h264'},
        }

        for cam_id, config in self.cameras.items():
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
        )

        pipeline = Gst.parse_launch(pipeline_str)
        sink = pipeline.get_by_name('sink')
        sink.connect('new-sample', lambda sink: self.on_new_sample(sink, cam_id))

        pipeline.set_state(Gst.State.PLAYING)
        self.pipelines[cam_id] = pipeline
        self.get_logger().info(f"Started pipeline for camera {cam_id}")

    def on_new_sample(self, sink, cam_id):
        sample = sink.emit('pull-sample')
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
<<<<<<< HEAD
=======
        thread.join()

if __name__ == '__main__':
    main()
>>>>>>> refs/remotes/origin/main
