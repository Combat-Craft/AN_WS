#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty, Float64
from math import radians, sin, cos, sqrt, atan2
import time

class DistanceTracker(Node):
    def __init__(self):
        super().__init__('distance_tracker')
        
        # Subscribers
        self.create_subscription(NavSatFix, 'gps_data', self.gps_callback, 10)
        self.create_subscription(Empty, 'reset_distance', self.reset_callback, 10)
        
        # Publisher
        self.distance_pub = self.create_publisher(Float64, 'moved_distance', 10)
        
        # Tracking variables
        self.reference_point = None
        self.last_distance = None
        self.is_resetting = False
        self.last_publish_time = 0
        self.publish_interval = 0.5  # Minimum seconds between publishes

    def gps_callback(self, msg):
        current_time = time.time()
        current_point = (msg.latitude, msg.longitude)
        
        if self.is_resetting:
            self.reference_point = current_point
            self.is_resetting = False
            self.last_distance = 0.0
            self.publish_distance(0.0)
            return
            
        if self.reference_point is None:
            self.reference_point = current_point
            self.last_distance = 0.0
            self.publish_distance(0.0)
            return
            
        current_distance = self.haversine(*self.reference_point, *current_point)
        
        # Only publish if:
        # 1. Distance changed significantly (>0.1m)
        # AND
        # 2. Enough time has passed since last publish
        if (abs(current_distance - self.last_distance) > 0.1 and 
            current_time - self.last_publish_time >= self.publish_interval):
            self.publish_distance(current_distance)

    def reset_callback(self, msg):
        self.is_resetting = True
        self.get_logger().info("Reset requested")

    def publish_distance(self, distance):
        self.last_distance = distance
        self.last_publish_time = time.time()
        msg = Float64()
        msg.data = distance
        self.distance_pub.publish(msg)
        self.get_logger().info(f"Distance: {distance:.2f} m")

    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1 = radians(lat1)
        phi2 = radians(lat2)
        delta_phi = radians(lat2 - lat1)
        delta_lambda = radians(lon2 - lon1)

        a = sin(delta_phi/2)**2 + cos(phi1)*cos(phi2)*sin(delta_lambda/2)**2
        c = 2 * atan2(sqrt(a), sqrt(1-a))
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