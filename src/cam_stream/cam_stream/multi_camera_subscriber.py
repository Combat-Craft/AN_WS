import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import gi
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
            1: '/cam1/h264'
        }

        self.pipelines = {}
        self.appsrcs = {}
        self.appsinks = {}
        self.frames = {0: None, 1: None}

        for cam_id, topic in self.cameras.items():
            self.setup_pipeline(cam_id)
            self.create_subscription(CompressedImage, topic, lambda msg, cid=cam_id: self.image_callback(msg, cid), 10)

        # OpenCV display loop in background
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()

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

    def image_callback(self, msg, cam_id):
        try:
            buf = Gst.Buffer.new_allocate(None, len(msg.data), None)
            buf.fill(0, msg.data)
            timestamp = msg.header.stamp.sec * Gst.SECOND + msg.header.stamp.nanosec
            buf.pts = timestamp
            buf.dts = timestamp

            self.appsrcs[cam_id].emit("push-buffer", buf)

            sample = self.appsinks[cam_id].emit("try-pull-sample", 10000000)  # 10ms timeout
            if sample:
                buffer = sample.get_buffer()
                success, map_info = buffer.map(Gst.MapFlags.READ)
                if success:
                    caps = sample.get_caps()
                    width = caps.get_structure(0).get_value('width')
                    height = caps.get_structure(0).get_value('height')
                    frame = np.frombuffer(map_info.data, np.uint8).reshape((height, width, 3))
                    self.frames[cam_id] = frame
                    buffer.unmap(map_info)
        except Exception as e:
            self.get_logger().error(f"Cam{cam_id} decode error: {e}")

    def display_loop(self):
        while rclpy.ok():
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
                # print("Waiting for frames...")
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