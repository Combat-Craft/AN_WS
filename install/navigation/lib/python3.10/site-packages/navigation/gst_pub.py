import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class GStreamerCameraPublisher(Node):
    def __init__(self):
        super().__init__('gstreamer_camera_publisher')

        # Publisher for compressed JPEGs
        self.publisher_ = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', 10)

        # Jetson-optimized GStreamer pipeline with JPEG compression
        self.gst_pipeline = (
            '''v4l2src device=/dev/video0 ! 
            video/x-raw, width=1920, height=1080, framerate=30/1 ! 
            nvvidconv ! 
            video/x-raw(memory:NVMM), format=NV12 ! 
            nvjpegenc quality=80 ! 
            appsink emit-signals=true sync=false max-buffers=1 drop=true'''
        )

        self.cap = cv2.VideoCapture(self.gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera with GStreamer pipeline.')
            raise RuntimeError("Failed to open GStreamer pipeline")

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)  # 30 FPS

    def timer_callback(self):
        ret, jpeg_data = self.cap.read()
        if not ret or jpeg_data is None:
            self.get_logger().warning('Camera JPEG frame not received.')
            return

        # Construct CompressedImage message
        if isinstance(jpeg_data, np.ndarray):
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = 'jpeg'
            msg.data = jpeg_data.tobytes()
            self.publisher_.publish(msg)
            self.get_logger().info('Published a compressed JPEG frame.')

    def destroy_node(self):
        self.get_logger().info('Shutting down and releasing camera...')
        if self.cap:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GStreamerCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
