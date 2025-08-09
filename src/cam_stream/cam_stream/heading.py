#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

class Heading(Node):
    def __init__(self):
        super().__init__('heading')

        # Subscribers
        self.create_subscription(NavSatFix, 'gps_data', self.gps_callback, 10)
        self.create_subscription(NavSatFix, 'heading_input', self.heading_callback, 10)

        # Publisher
        self.heading_pub = self.create_publisher(String, 'heading_direction', 10)

        self.reach_latitude = None
        self.reach_longitude = None
        self.ns = ""
        self.ew = ""

    def gps_callback(self, msg):
        if self.reach_latitude is None or self.reach_longitude is None:
            return 

        if (msg.latitude + 0.000045) < self.reach_latitude:
            self.ns = "North"
        elif (msg.latitude - 0.000045) > self.reach_latitude:
            self.ns = "South"
        else:
            self.ns = "Same Lat"

        if (msg.longitude + 0.000062) < self.reach_longitude:
            self.ew = "East"
        elif (msg.longitude - 0.000062) > self.reach_longitude:
            self.ew = "West"
        else:
            self.ew = "Same Lon"

        self.publish_heading()

    def publish_heading(self):
        msg = String()
        
        if (self.ns == "Same Lat" and self.ew == "Same Lon"):
            msg.data = "You are here"
        else:
            msg.data = f"Head {self.ns} {self.ew}"
        
        self.heading_pub.publish(msg)

    def heading_callback(self, msg):
        self.reach_latitude = msg.latitude
        self.reach_longitude = msg.longitude
        self.get_logger().info(f"Target set: {self.reach_latitude}, {self.reach_longitude}")

def main(args=None):
    rclpy.init(args=args)
    node = Heading()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
