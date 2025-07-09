#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn

class RCOverrideNode(Node):
    def __init__(self):
        super().__init__('rc_override_node')
        self.pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.timer = self.create_timer(0.1, self.publish_override)  # 10Hz

        # Customize these values
        self.left_pwm = 2000
        self.right_pwm = 900

    def publish_override(self):
        msg = OverrideRCIn()
        msg.channels = [self.left_pwm, 0, self.right_pwm] + [0]*15
        self.pub.publish(msg)
        self.get_logger().info(f'RC Override → CH1: {self.left_pwm}, CH3: {self.right_pwm}')

def main(args=None):
    rclpy.init(args=args)
    node = RCOverrideNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

