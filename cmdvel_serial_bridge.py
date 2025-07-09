#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelSerialBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_serial_bridge')
        # adjust device & baud as needed:
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_cb, 10)

    def cmd_vel_cb(self, msg: Twist):
        # format as "v,w\n" with 3 decimal places
        line = f"{msg.linear.x:.3f},{msg.angular.z:.3f}\n"
        self.ser.write(line.encode('utf-8'))
        self.get_logger().debug(f"→ {line.strip()}")

    def destroy_node(self):
        self.ser.close()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSerialBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__== '__main__':
    main()
