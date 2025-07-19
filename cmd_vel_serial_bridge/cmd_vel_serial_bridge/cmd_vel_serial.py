import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelSerialBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_serial_bridge')
        self.get_logger().info("Initializing Serial")
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200)
        self.get_logger().info("Serial Connected")
        
        self.get_logger().info("Subsribing to /cmd_vel")
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.get_logger().info("Subscribed")

    def listener_callback(self, msg):
        data = f"{msg.linear.x:.2f},{msg.angular.z:.2f}\n"
        self.serial_port.write(data.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSerialBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
