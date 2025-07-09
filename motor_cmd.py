#!/usr/bin/env python3
"""
ROS 2 node that converts /cmd_vel to MAVLink DO_SET_SERVO commands for a
Cube Orange Plus (ArduPilot 4.6) **without stealing the serial port**.  Instead
of opening /dev/ttyACM0 directly, this version defaults to a UDP link (e.g.
`udpout:127.0.0.1:14555`) so it can coexist with any PX4‑ROS or MAVROS launch
file already using the USB CDC device.

If you really need the direct serial link, pass `--device /dev/ttyACM0` on the
command line—just remember only one process can own that port at a time.

Tested with:
  • ROS 2 Humble  (rclpy)
  • pymavlink >= 2.4.30
  • mavlink‑router (optional, for UDP fan‑out)

Quick start (leave your existing MAVROS / px4 launch running):
  1.  `sudo systemctl start mavlink-routerd`   # or run it manually
  2.  Ensure mavlink‑router is forwarding the Cube USB link to 14555
  3.  `$ python3 cmd_vel_to_servo.py`          # defaults to udp:…:14555

"""
from __future__ import annotations

import argparse
from threading import Lock
from typing import Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────────────────────
# User‑tunable constants
# ─────────────────────────────────────────────────────────────────────────────
PWM_NEUTRAL       = 1495
PWM_FORWARD_MIN   = 1520
PWM_FORWARD_MAX   = 1748
PWM_BACKWARD_MAX  = 1470
PWM_BACKWARD_MIN  = 1243

LINEAR_SCALE  = 1.2   # –1 … +1 → PWM band
ANGULAR_SCALE = 500   # rad/s → PWM delta between left/right

RIGHT_MOTOR_CHANNEL = 1   # SERVO1_FUNCTION = 1 (RC passthrough) or 74
LEFT_MOTOR_CHANNEL  = 4   # SERVO4_FUNCTION = 1 (RC passthrough) or 73

# ─────────────────────────────────────────────────────────────────────────────
# Helper functions
# ─────────────────────────────────────────────────────────────────────────────

def scale_velocity_to_pwm(vel: float) -> int:
    """Map –1 … +1 linear velocity to PWM, respecting dead‑zone."""
    v = max(-1.0, min(1.0, vel * LINEAR_SCALE))
    if v > 0:
        pwm = PWM_FORWARD_MIN + v * (PWM_FORWARD_MAX - PWM_FORWARD_MIN)
    elif v < 0:
        pwm = PWM_BACKWARD_MAX + v * (PWM_BACKWARD_MAX - PWM_BACKWARD_MIN)
    else:
        pwm = PWM_NEUTRAL
    return int(round(max(PWM_BACKWARD_MIN, min(PWM_FORWARD_MAX, pwm))))


def send_set_servo(master: mavutil.mavfile, channel: int, pwm: int) -> None:
    msg = master.mav.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel, pwm, 0, 0, 0, 0, 0)
    master.mav.send(msg)


def send_motor_pwms(master: mavutil.mavfile, left: int, right: int) -> None:
    send_set_servo(master, LEFT_MOTOR_CHANNEL,  left)
    send_set_servo(master, RIGHT_MOTOR_CHANNEL, right)

# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 node
# ─────────────────────────────────────────────────────────────────────────────
class CmdVelBridge(Node):
    def __init__(self, master: mavutil.mavfile, topic: str) -> None:
        super().__init__('cmd_vel_to_servo')
        self.master = master
        self.lock = Lock()
        self.create_subscription(Twist, topic, self.cb, 10)
        self.get_logger().info(
            f"Bridging {topic} → channels {LEFT_MOTOR_CHANNEL}/{RIGHT_MOTOR_CHANNEL}")

    def cb(self, msg: Twist) -> None:
        with self.lock:
            base   = scale_velocity_to_pwm(msg.linear.x)
            offset = msg.angular.z * ANGULAR_SCALE
            left  = int(round(base + offset))
            right = int(round(base - offset))
        left  = max(PWM_BACKWARD_MIN, min(PWM_FORWARD_MAX, left))
        right = max(PWM_BACKWARD_MIN, min(PWM_FORWARD_MAX, right))
        try:
            send_motor_pwms(self.master, left, right)
        except Exception as e:
            self.get_logger().error(f"MAVLink send failed: {e}")

# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="/cmd_vel → DO_SET_SERVO bridge")
    p.add_argument('--device', default='udpout:127.0.0.1:14555',
                   help='MAVLink device/URL (serial or UDP).')
    p.add_argument('--baud', type=int, default=115200,
                   help='Baud rate for serial links (ignored for UDP).')
    p.add_argument('--source-system', type=int, default=255,
                   help='Our MAVLink system ID.')
    p.add_argument('--topic', default='/cmd_vel', help='Twist topic.')
    return p.parse_args()


def main() -> None:
    args = parse_args()

    master = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        source_system=args.source_system)
    print(f"Connecting via {args.device}…")
    master.wait_heartbeat()
    print(f"Heartbeat from sys {master.target_system}")

    rclpy.init()
    node = CmdVelBridge(master, args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        master.close()


if __name__ == '__main__':
    main()
