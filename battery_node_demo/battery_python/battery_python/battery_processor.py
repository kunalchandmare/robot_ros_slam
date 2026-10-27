#!/usr/bin/env python3
"""Processor node: receives battery level and publishes a warning when low."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class BatteryProcessor(Node):
    def __init__(self):
        super().__init__('battery_processor')
        self.subscription = self.create_subscription(
            Int32,
            'battery_level',
            self.battery_callback,
            10)
        self.warning_publisher = self.create_publisher(String, 'battery_warning', 10)
        self.get_logger().info('Battery processor started. Listening on /battery_level')

    def battery_callback(self, msg):
        level = msg.data
        self.get_logger().info(f'Received battery level: {level}%')

        # Simple logic: warn if battery is low
        if level <= 20:
            warning = String()
            warning.data = f'WARNING: Battery low! Only {level}% remaining.'
            self.warning_publisher.publish(warning)
            self.get_logger().warn(warning.data)
        elif level <= 50:
            self.get_logger().info('Battery is getting low.')


def main(args=None):
    rclpy.init(args=args)
    node = BatteryProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
