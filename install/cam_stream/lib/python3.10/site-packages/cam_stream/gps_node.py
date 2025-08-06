#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import threading


class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('simulated_data', True)

        self.publisher = self.create_publisher(NavSatFix, 'gps_data', 1)
        self.serial_port = None
        self.running = True

        self.init_serial_connection()

        if self.serial_port:
            self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
            self.read_thread.start()
            self.get_logger().info("Started background serial thread.")
        elif self.get_parameter('simulated_data').value:
            self.timer = self.create_timer(1.0, self.publish_simulated)
            self.get_logger().info("Using simulated GPS data.")
        else:
            self.get_logger().error("No serial and no simulated data. Node will not function.")

    def init_serial_connection(self):
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        try:
            self.serial_port = serial.Serial(port=port, baudrate=baudrate, timeout=1.0)
            self.get_logger().info(f"Connected to {port} at {baudrate} baud")
        except Exception as e:
            self.get_logger().warning(f"Failed to connect to {port}: {e}")
            self.serial_port = None

    def read_serial_loop(self):
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                raw = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                msg = self.parse_nmea(raw)
                if msg:
                    self.publisher.publish(msg)
                    self.get_logger().info(
                        f"Published: Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}, "
                        f"Alt={msg.altitude:.1f}m, Status={msg.status.status}"
                    )
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                break

    def parse_nmea(self, nmea):
        if not nmea.startswith('$GNGGA') and not nmea.startswith('$GPGGA'):
            return None
        parts = nmea.split(',')
        if len(parts) < 10 or parts[6] == '0':
            return None

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'

        try:
            lat_deg = float(parts[2][:2])
            lat_min = float(parts[2][2:])
            msg.latitude = lat_deg + lat_min / 60.0
            if parts[3] == 'S':
                msg.latitude *= -1

            lon_deg = float(parts[4][:3])
            lon_min = float(parts[4][3:])
            msg.longitude = lon_deg + lon_min / 60.0
            if parts[5] == 'W':
                msg.longitude *= -1

            msg.altitude = float(parts[9]) if parts[9] else 0.0
            msg.status.status = int(parts[6])
            return msg
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Parse error: {e}")
            return None

    def get_simulated_data(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.latitude = 48.8584
        msg.longitude = 2.2945
        msg.altitude = 32.0
        msg.status.status = 1
        msg.latitude += 0.000001
        msg.longitude += 0.000001
        return msg

    def publish_simulated(self):
        msg = self.get_simulated_data()
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Simulated: Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}, "
            f"Alt={msg.altitude:.1f}m, Status={msg.status.status}"
        )

    def destroy_node(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

