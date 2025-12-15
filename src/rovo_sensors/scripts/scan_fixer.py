#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFixer(Node):
    def __init__(self):
        super().__init__('scan_fixer')
        # Listen to raw/laggy data
        self.create_subscription(LaserScan, 'scan_raw', self.callback, 10)
        # Publish fresh data
        self.pub = self.create_publisher(LaserScan, 'scan', 10)

    def callback(self, msg):
        # THE FIX: Force timestamp to be NOW
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ScanFixer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()