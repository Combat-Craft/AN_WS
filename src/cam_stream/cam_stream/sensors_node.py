import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import serial


class SensorsNode(Node):
    def __init__(self):
        super().__init__('sensors_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)

        self.serial = serial.Serial(self.get_parameter('port').value, self.get_parameter('baudrate').value, timeout=1)

        self.status_pub = self.create_publisher(String, 'sensors_status', 10)
        self.data_pub = self.create_publisher(String, 'sensors_table', 10)
        
        self.timer = self.create_timer(1.0, self.read_serial)


    def read_serial(self):
        if self.serial.in_waiting:
            line = self.serial.readline().decode('utf-8').strip()

            if not line or line.startswith('===') or line.startswith('Time') or 'Warming up' in line:
                return

            try:
                parts = line.split()


                if len(parts) < 6:
                    self.get_logger().warn(f"Unexpected data format: {line}")
                    return


                # First 5 are numeric, the rest is status
                time_sec = int(parts[0])
                cpm = int(parts[1])
                usvh = float(parts[2])
                o3_ppm = float(parts[3])
                h2_ppm = float(parts[4])
                status = ' '.join(parts[5:])  #  the rest as status string


                
                msg = String()
                msg.data = f"t={time_sec}s | {status}"
                self.status_pub.publish(msg)


                
                self.get_logger().info(f"Published status: {msg.data}")


                # Publish numeric values
                #data_msg = Float32MultiArray()
                #data_msg.data = [float(time_sec), float(cpm), usvh, o3_ppm, h2_ppm]
                #self.data_pub.publish(data_msg)
                table_msg = String()
                table_msg.data = (
                    f"Sensor Readings:\n"
                    f"  Time:     {time_sec} s\n"
                    f"  CPM:      {cpm}\n"
                    f"  uSv/h:    {usvh:.3f}\n"
                    f"  O₃ (ppm): {o3_ppm:.3f}\n"
                    f"  H₂ (ppm): {h2_ppm:.3f}"
                )
                self.data_pub.publish(table_msg)

                self.get_logger().info(f"Published data: {table_msg.data}")


            except Exception as e:
                self.get_logger().error(f"Error parsing line: {line} -> {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SensorsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()