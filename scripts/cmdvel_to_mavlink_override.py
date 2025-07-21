#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pymavlink import mavutil
import time

# Define constants
PWM_NEUTRAL = 1500
PWM_MIN = 1100
PWM_MAX = 1900
CMD_VEL_MIN = -0.5
CMD_VEL_MAX = 0.5

class MavlinkCmdVelNode(Node):
    def __init__(self):
        super().__init__('mavlink_cmd_vel_listener')

        # Setup MAVLink connection
        self.master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
        self.master.wait_heartbeat()
        self.master.target_system = 1
        self.master.target_component = 1

        self.get_logger().info(f"Heartbeat from system {self.master.target_system}, component {self.master.target_component}")

        # Set neutral throttle to avoid initial spin
        self.set_rc_channel_pwm(1, PWM_NEUTRAL)
        self.set_rc_channel_pwm(3, PWM_NEUTRAL)
        time.sleep(1)

        # Subscribe to /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def set_rc_channel_pwm(self, channel_id, pwm=1500):
        if channel_id < 1 or channel_id > 8:
            self.get_logger().error("Channel out of range.")
            return
        rc_channel_values = [65535] * 8
        rc_channel_values[channel_id - 1] = pwm
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_channel_values
        )

    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        left_motor_speed = linear_vel - angular_vel
        right_motor_speed = linear_vel + angular_vel

        left_pwm = int((left_motor_speed - CMD_VEL_MIN) / (CMD_VEL_MAX - CMD_VEL_MIN) * (PWM_MAX - PWM_MIN) + PWM_MIN)
        right_pwm = int((right_motor_speed - CMD_VEL_MIN) / (CMD_VEL_MAX - CMD_VEL_MIN) * (PWM_MAX - PWM_MIN) + PWM_MIN)

        left_pwm = max(PWM_MIN, min(PWM_MAX, left_pwm))
        right_pwm = max(PWM_MIN, min(PWM_MAX, right_pwm))

        self.get_logger().info(f"cmd_vel: linear={linear_vel:.2f}, angular={angular_vel:.2f}")
        self.get_logger().info(f"Left PWM: {left_pwm}, Right PWM: {right_pwm}")

        self.set_rc_channel_pwm(1, left_pwm)
        self.set_rc_channel_pwm(4, right_pwm)

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkCmdVelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

