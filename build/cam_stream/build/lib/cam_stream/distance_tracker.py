#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty
from math import radians, sin, cos, sqrt, atan2


class DistanceTracker(Node):
    def __init__(self):
        super().__init__('distance_tracker')
        self.subscription = self.create_subscription(
            NavSatFix,
            'gps_data',
            self.gps_callback,
            10
        )
        self.reset_sub = self.create_subscription(
            Empty,
            'reset_distance',
            self.reset_callback,
            10
        )
        self.prev_lat = None
        self.prev_lon = None
        self.total_distance = 0.0

    def gps_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude

        if self.prev_lat is not None and self.prev_lon is not None:
            distance = self.haversine(self.prev_lat, self.prev_lon, lat, lon)
            self.total_distance += distance
            self.get_logger().info(
                f"Moved {distance:.2f} m | Total Distance: {self.total_distance:.2f} m"
            )

        self.prev_lat = lat
        self.prev_lon = lon

    def reset_callback(self, msg):
        self.total_distance = 0.0
        self.get_logger().info("Distance reset to 0.")

    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371000  # Earth radius in meters
        phi1 = radians(lat1)
        phi2 = radians(lat2)
        delta_phi = radians(lat2 - lat1)
        delta_lambda = radians(lon2 - lon1)

        a = sin(delta_phi / 2.0)**2 + \
            cos(phi1) * cos(phi2) * sin(delta_lambda / 2.0)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        return R * c


def main(args=None):
    rclpy.init(args=args)
    node = DistanceTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
