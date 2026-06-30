#!/usr/bin/env python3
"""
teleop_diagnostic.py
Monitors /cmd_vel AND /wheel_ticks simultaneously.
Prints a timestamped log so we can see if commands are arriving
and if the ESP32 is responding with correct tick counts.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point32
import time

class TeleopDiagnostic(Node):
    def __init__(self):
        super().__init__('teleop_diagnostic')

        self.last_cmd = None
        self.last_ticks = None
        self.cmd_count = 0
        self.tick_count = 0

        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)
        self.sub_ticks = self.create_subscription(
            Point32, '/wheel_ticks', self.ticks_callback, 10)

        self.timer = self.create_timer(1.0, self.print_status)

        print("=" * 70)
        print("  TELEOP DIAGNOSTIC — monitoring /cmd_vel and /wheel_ticks")
        print("=" * 70)
        print(f"{'TIME':>8} | {'CMD lin.x':>10} {'CMD ang.z':>10} | {'TICKS L':>12} {'TICKS R':>12} | {'NOTES'}")
        print("-" * 70)

    def cmd_callback(self, msg):
        self.cmd_count += 1
        lin = msg.linear.x
        ang = msg.angular.z
        t = time.strftime("%H:%M:%S")

        if lin > 0.01:
            direction = "FORWARD"
        elif lin < -0.01:
            direction = "BACKWARD"
        elif ang > 0.01:
            direction = "TURN LEFT"
        elif ang < -0.01:
            direction = "TURN RIGHT"
        else:
            direction = "STOP"

        # Compute expected wheel RPMs so user can cross-check
        WHEEL_SEP = 0.44
        WHEEL_RADIUS = 0.05
        MS_TO_RPM = 60.0 / (2.0 * 3.14159 * WHEEL_RADIUS)
        left_ms  = lin - (ang * WHEEL_SEP / 2.0)
        right_ms = lin + (ang * WHEEL_SEP / 2.0)
        left_rpm  = left_ms  * MS_TO_RPM
        right_rpm = right_ms * MS_TO_RPM

        print(f"[CMD ] {t} | lin={lin:+.3f}  ang={ang:+.3f} | → {direction:12} | expected RPM: L={left_rpm:+.1f} R={right_rpm:+.1f}")
        self.last_cmd = msg

    def ticks_callback(self, msg):
        self.tick_count += 1
        t = time.strftime("%H:%M:%S")
        delta_l = 0
        delta_r = 0
        if self.last_ticks:
            delta_l = msg.x - self.last_ticks.x
            delta_r = msg.y - self.last_ticks.y
        self.last_ticks = msg

        # Only print if there is actual movement
        if abs(delta_l) > 0.5 or abs(delta_r) > 0.5:
            print(f"[TICK] {t} | L={msg.x:+10.1f}  R={msg.y:+10.1f} | delta L={delta_l:+.1f}  R={delta_r:+.1f}")

    def print_status(self):
        t = time.strftime("%H:%M:%S")
        print(f"[STAT] {t} | cmd_vel msgs received: {self.cmd_count} | wheel_tick msgs received: {self.tick_count}")
        if self.tick_count == 0:
            print("       *** WARNING: No /wheel_ticks received — ESP32 may not be connected or micro_ros_agent not running")
        if self.cmd_count == 0:
            print("       *** WARNING: No /cmd_vel received — is the teleop node running?")
        self.cmd_count = 0
        self.tick_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = TeleopDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
