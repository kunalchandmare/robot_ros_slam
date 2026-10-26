#!/usr/bin/env python3
"""Simple publisher node: publishes a decreasing battery level."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        self.publisher_ = self.create_publisher(Int32, 'battery_level', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # every 1 second
        self.battery = 100
        self.get_logger().info('Battery publisher started. Publishing on /battery_level')

    def timer_callback(self):
        msg = Int32()
        msg.data = self.battery
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing battery level: {self.battery}%')

        # Decrease battery slowly (simulates usage)
        self.battery -= 2
        if self.battery < 0:
            self.battery = 100  # reset for continuous demo


def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
