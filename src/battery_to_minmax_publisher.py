#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from wifi_viz.msg import MinMaxCurr

class BatteryToMinMaxPublisher(Node):
    def __init__(self):
        super().__init__('battery_to_minmax_publisher')
        self.subscription = self.create_subscription(
            BatteryState,
            '/main_battery',
            self.battery_callback,
            10
        )
        self.publisher = self.create_publisher(MinMaxCurr, 'min_max_curr', 10)
        self.min_value = 0.0
        self.max_value = 45.0

    def battery_callback(self, msg):
        min_max_msg = MinMaxCurr()
        min_max_msg.min = self.min_value
        min_max_msg.max = self.max_value
        min_max_msg.current = msg.voltage

        # Optionally set a color based on the current value
        normalized_value = (msg.voltage - self.min_value) / (self.max_value - self.min_value)
        normalized_value = max(0.0, min(1.0, normalized_value))  # Clamp between 0 and 1
        min_max_msg.current_color.r = 1.0 - normalized_value
        min_max_msg.current_color.g = normalized_value
        min_max_msg.current_color.b = 0.0
        min_max_msg.current_color.a = 1.0

        self.publisher.publish(min_max_msg)
        self.get_logger().info(f'Published MinMaxCurr: min={min_max_msg.min}, max={min_max_msg.max}, current={min_max_msg.current}')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryToMinMaxPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()