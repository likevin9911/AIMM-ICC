import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class DummyOdomPublisher(Node):
    def __init__(self):
        super().__init__('dummy_odom_publisher')
        self.publisher = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.timer = self.create_timer(0.5, self.publish_odom)
        self.t = 0.0

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        radius = 1.0
        angular_speed = 0.1
        x = radius * math.cos(self.t)
        y = radius * math.sin(self.t)
        vx = -radius * math.sin(self.t) * angular_speed
        vy = radius * math.cos(self.t) * angular_speed

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.pose.covariance = [0.05] * 36
        msg.twist.covariance = [0.05] * 36

        self.publisher.publish(msg)
        self.t += angular_speed

def main(args=None):
    rclpy.init(args=args)
    node = DummyOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
