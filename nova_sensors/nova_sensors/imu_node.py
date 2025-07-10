import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion, Vector3
import serial
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy


class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_node')
       
        # Configure QoS for sensor data (best effort for real-time)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
       
        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', qos)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', qos)
       
        # Serial connection
        self.serial = self._init_serial('/dev/ttyACM4', 115200)
       
        # Diagnostics
        self.last_print = time.time()
        self.msg_count = 0
       
        # Main timer (50Hz)
        self.timer = self.create_timer(0.02, self.run)
       
        self.get_logger().info("BNO055 node started - Waiting for data...")


    def _init_serial(self, port, baudrate, retries=5):
        """Robust serial connection initialization"""
        for i in range(retries):
            try:
                ser = serial.Serial(port, baudrate, timeout=0.1)
                self.get_logger().info(f"Connected to {port} at {baudrate} baud")
                return ser
            except Exception as e:
                self.get_logger().warn(f"Attempt {i+1}/{retries} failed: {str(e)}")
                time.sleep(1)
        raise RuntimeError(f"Could not connect to {port}")


    def _parse_line(self, line):
        """Parse serial line into dictionary with validation"""
        data = {}
        try:
            for pair in line.strip().split(','):
                if ':' in pair:
                    key, val = pair.split(':')
                    data[key.strip()] = float(val)
        except ValueError as e:
            self.get_logger().warn(f"Parse error in '{line}': {str(e)}")
        return data


    def run(self):
        """Main processing loop"""
        try:
            # Read serial data
            if self.serial.in_waiting:
                raw = self.serial.readline().decode('ascii', errors='ignore').strip()
                if not raw:
                    return
                   
                data = self._parse_line(raw)
                if not data:
                    return
               
                # Publish IMU data (if any accel/gyro data exists)
                if any(k in data for k in ['ax', 'ay', 'az', 'gx', 'gy', 'gz']):
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = 'imu_link'
                   
                    # Linear acceleration
                    if all(k in data for k in ['ax', 'ay', 'az']):
                        imu_msg.linear_acceleration = Vector3(
                            x=data['ax'], y=data['ay'], z=data['az'])
                   
                    # Angular velocity
                    if all(k in data for k in ['gx', 'gy', 'gz']):
                        imu_msg.angular_velocity = Vector3(
                            x=data['gx'], y=data['gy'], z=data['gz'])
                   
                    self.imu_pub.publish(imu_msg)
                    self._log_publish('IMU', imu_msg.header.stamp)
               
                # Publish magnetometer
                if all(k in data for k in ['mx', 'my', 'mz']):
                    mag_msg = MagneticField()
                    mag_msg.header.stamp = self.get_clock().now().to_msg()
                    mag_msg.header.frame_id = 'imu_link'
                    mag_msg.magnetic_field = Vector3(
                        x=data['mx'], y=data['my'], z=data['mz'])
                    self.mag_pub.publish(mag_msg)
                    self._log_publish('MAG', mag_msg.header.stamp)
       
        except Exception as e:
            self.get_logger().error(f"Processing error: {str(e)}")
            if self.serial:
                self.serial.close()
            self.serial = self._init_serial('/dev/ttyACM4', 115200)


    def _log_publish(self, msg_type, stamp):
        """Controlled logging to avoid spamming"""
        self.msg_count += 1
        if time.time() - self.last_print > 1.0:  # Throttle to 1Hz
            self.get_logger().info(
                f"Published {self.msg_count} {msg_type} messages "
                f"(latest: {stamp.sec}.{stamp.nanosec:09d})",
                throttle_duration_sec=1.0
            )
            self.last_print = time.time()
            self.msg_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial.is_open:
            node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
