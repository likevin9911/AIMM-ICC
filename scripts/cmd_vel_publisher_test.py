#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TestCmdVelPublisher(Node):
    def __init__(self):
        super().__init__('test_cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.step = 0
        self.test_sequence = [
            (0.2, 0.0),   # forward
            (0.0, 0.3),   # rotate right
            (0.0, -0.3),  # rotate left
            (-0.2, 0.0),  # backward
            (0.2, 0.2),   # forward right curve
            (0.2, -0.2),  # forward left curve
            (0.0, 0.0),   # stop
        ]

    def timer_callback(self):
        if self.step < len(self.test_sequence):
            twist = Twist()
            twist.linear.x = self.test_sequence[self.step][0]
            twist.angular.z = self.test_sequence[self.step][1]
            self.publisher_.publish(twist)
            self.get_logger().info(f"Published cmd_vel: linear.x={twist.linear.x}, angular.z={twist.angular.z}")
            self.step += 1
        else:
            self.get_logger().info("Test complete. Shutting down.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TestCmdVelPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

