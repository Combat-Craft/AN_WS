#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial


class SensorsNode(Node):
    def __init__(self):
        super().__init__('sensors_node')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        # Get parameters
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        # Publishers
        self.status_pub = self.create_publisher(String, 'sensor_status', 10)
        self.data_pub = self.create_publisher(String, 'sensor_data', 10)
        self.gps_pub = self.create_publisher(NavSatFix, 'gps_data', 10)

        # Connect to serial
        try:
            self.serial = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"Connected to {port} at {baudrate} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to serial port {port}: {e}")
            self.serial = None

        # Timer to poll serial data
        self.timer = self.create_timer(1.0, self.read_serial)

        # Temp storage for multi-line parsing
        self.current_block = []

    def read_serial(self):
        if not self.serial or not self.serial.is_open:
            return

        while self.serial.in_waiting:
            line = self.serial.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            # Start a new block ONLY when we see the first sensor readings line
            if line.startswith('--- Sensor Readings ---'):
                self.current_block = [line]
                continue

            # If we haven't started a block yet, ignore incoming lines
            if not self.current_block:
                continue

            # Always append lines once we're in a block
            self.current_block.append(line)

            # End of block: UTC timestamp means we have all the data
            if line.startswith("UTC Date/Time:"):
                self.parse_block(self.current_block)
                self.current_block = []  # Reset for next cycle

    def parse_block(self, lines):
        ozone_ppm = None
        hydrogen_ppm = None
        cpm = None
        usvh = None
        gps_lat = None
        gps_lon = None
        utc_time = None
        gps_valid = False

        gps_line_raw = None

        for line in lines:
            if line.startswith("Ozone:"):
                ozone_ppm = float(line.split(":")[1].replace("ppm", "").strip())
            elif line.startswith("Hydrogen:"):
                hydrogen_ppm = float(line.split(":")[1].replace("ppm", "").strip())
            elif line.startswith("Radiation:"):
                parts = line.replace("Radiation:", "").split("|")
                if len(parts) == 2:
                    cpm = int(parts[0].replace("CPM", "").strip())
                    usvh = float(parts[1].replace("µSv/h", "").strip())
            elif line.startswith("--- GPS ---(Lat/Long)"):
                gps_line_raw = line
                gps_parts = line.split(";")
                if len(gps_parts) >= 3 and "invalid" not in gps_parts[1]:
                    try:
                        gps_lat = float(gps_parts[1])
                        gps_lon = float(gps_parts[2])
                        gps_valid = True
                    except ValueError:
                        gps_valid = False
            elif line.startswith("UTC Date/Time:"):
                utc_time = line.replace("UTC Date/Time:", "").strip()

        # Publish sensor status
        if ozone_ppm is not None or hydrogen_ppm is not None or cpm is not None:
            status_msg = String()
            status_msg.data = f"O₃={ozone_ppm} ppm | H₂={hydrogen_ppm} ppm | CPM={cpm} | uSv/h={usvh}"
            self.status_pub.publish(status_msg)
            self.get_logger().info(f"Published status: {status_msg.data}")

            table_msg = String()
            table_msg.data = (
                f"Sensor Readings:\n"
                f"  Ozone:     {ozone_ppm} ppm\n"
                f"  Hydrogen:  {hydrogen_ppm} ppm\n"
                f"  CPM:       {cpm}\n"
                f"  uSv/h:     {usvh}\n"
                f"  GPS:       {gps_line_raw}\n"
                f"  UTC Time:  {utc_time}"
            )
            self.data_pub.publish(table_msg)
            self.get_logger().info(f"Published data:\n{table_msg.data}")

        # Publish GPS if valid
        if gps_valid:
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = 'gps'
            gps_msg.latitude = gps_lat
            gps_msg.longitude = gps_lon
            gps_msg.altitude = 0.0  # Not provided
            gps_msg.status.status = NavSatStatus.STATUS_FIX
            self.gps_pub.publish(gps_msg)
            self.get_logger().info(f"Published GPS: Lat={gps_lat}, Lon={gps_lon}")

    def destroy_node(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
