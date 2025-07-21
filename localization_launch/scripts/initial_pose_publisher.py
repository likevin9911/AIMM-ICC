#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from builtin_interfaces.msg import Time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(1.0, self.publish_initial_pose)
        self.has_published = False

    def publish_initial_pose(self):
        if self.has_published:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = 5.0
        msg.pose.pose.position.y = 10.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0  # Facing forward

        # Covariance: set high for Z and roll/pitch/yaw if unsure
        msg.pose.covariance = [
            0.25, 0.0,  0.0,   0.0, 0.0, 0.0,
            0.0, 0.25,  0.0,   0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0,  0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0,  0.0,   0.0, 99999.0, 0.0,
            0.0, 0.0,  0.0,   0.0, 0.0, 0.0685
        ]

        self.publisher.publish(msg)
        self.get_logger().info('Initial pose published.')
        self.has_published = True

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
