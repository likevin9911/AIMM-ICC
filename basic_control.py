#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pymavlink import mavutil
import time

# autopilot’s system/component IDs
TARGET_SYSTEM_ID    = 1
TARGET_COMPONENT_ID = 1

# your companion’s (GCS) system ID
SOURCE_SYSTEM_ID    = 255

PWM_NEUTRAL = 1500
PWM_MIN     = 1100
PWM_MAX     = 1900

CMD_VEL_MIN = -0.5
CMD_VEL_MAX =  0.5

class MavlinkCmdVelListener(Node):
    def __init__(self):
        super().__init__('mavlink_cmd_vel_listener')
        # ==== Dial into MAVROS on localhost:14550 as system 255 ====
        self.master = mavutil.mavlink_connection(
            'udpout:127.0.0.1:14550',
            source_system=SOURCE_SYSTEM_ID
        )
        self.master.target_system    = TARGET_SYSTEM_ID
        self.master.target_component = TARGET_COMPONENT_ID

        self.get_logger().info('Waiting for MAVLink heartbeat…')
        self.master.wait_heartbeat()
        self.get_logger().info(
            f"Heartbeat from {self.master.target_system}/"
            f"{self.master.target_component}"
        )

        # neutral both motors
        self._set_rc(1, PWM_NEUTRAL)
        self._set_rc(3, PWM_NEUTRAL)
        time.sleep(1.0)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

    def _set_rc(self, ch, pwm):
        if not (1 <= ch <= 8):
            self.get_logger().warn(f"Invalid channel {ch}")
            return
        vals = [65535]*8
        vals[ch-1] = pwm
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *vals
        )

    def cmd_vel_cb(self, msg: Twist):
        lin = msg.linear.x
        ang = msg.angular.z
        l_eff = lin - ang
        r_eff = lin + ang

        def effort_to_pwm(e):
            raw = int((e - CMD_VEL_MIN) /
                      (CMD_VEL_MAX - CMD_VEL_MIN) *
                      (PWM_MAX - PWM_MIN) +
                      PWM_MIN)
            return max(PWM_MIN, min(PWM_MAX, raw))

        l_pwm = effort_to_pwm(l_eff)
        r_pwm = effort_to_pwm(r_eff)

        self.get_logger().info(f"cmd_vel → L={l_pwm} R={r_pwm}")
        self._set_rc(1, l_pwm)
        self._set_rc(3, r_pwm)

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkCmdVelListener()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
