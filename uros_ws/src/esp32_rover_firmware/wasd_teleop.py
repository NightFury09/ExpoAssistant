#!/usr/bin/env python3
"""
wasd_teleop.py — WASD Teleop for the Expo Rover

Controls:
    W / S  — Forward / Backward
    A / D  — Turn Left / Turn Right
    Q / E  — Strafe-turn Left / Strafe-turn Right
    SPACE  — Emergency Stop
    X      — Quit

Speed controls:
    1-9    — Set speed (1=slow, 9=fast)
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ---- Key bindings → (linear.x, angular.z) multipliers ----
KEY_BINDINGS = {
    'w': ( 1.0,  0.0),   # Forward
    's': (-1.0,  0.0),   # Backward
    'a': ( 0.0,  1.0),   # Turn left (in place)
    'd': ( 0.0, -1.0),   # Turn right (in place)
    'q': ( 1.0,  1.0),   # Forward + turn left
    'e': ( 1.0, -1.0),   # Forward + turn right
    ' ': ( 0.0,  0.0),   # Stop
}

HELP = """
╔══════════════════════════════════╗
║     WASD Rover Teleop            ║
╠══════════════════════════════════╣
║   Q   W   E                     ║
║   A       D                     ║
║       S                         ║
║                                 ║
║   W/S : Forward / Backward      ║
║   A/D : Turn Left / Right       ║
║   Q/E : Arc Left  / Arc Right   ║
║ SPACE : STOP                    ║
║   1-9 : Speed level              ║
║   X   : Quit                    ║
╚══════════════════════════════════╝
"""

MAX_LIN = 0.5   # m/s at speed level 9
MAX_ANG = 2.0   # rad/s at speed level 9


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed_level = 5
        self.get_logger().info(HELP)

    def run(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            print(HELP)
            print(f"\r\nSpeed level: {self.speed_level}  |  Press a key...\r\n")

            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()

                    if key == 'x' or key == '\x03':  # x or Ctrl-C
                        # Send stop before quitting
                        self._publish(0.0, 0.0)
                        break

                    if key in '123456789':
                        self.speed_level = int(key)
                        print(f"\r  Speed level: {self.speed_level}       \r", end='')
                        continue

                    if key in KEY_BINDINGS:
                        lin_mult, ang_mult = KEY_BINDINGS[key]
                        frac = self.speed_level / 9.0
                        lin = lin_mult * MAX_LIN * frac
                        ang = ang_mult * MAX_ANG * frac
                        self._publish(lin, ang)
                        status = "STOP" if (lin == 0 and ang == 0) else f"lin={lin:+.2f} ang={ang:+.2f}"
                        print(f"\r  [{key.upper()}] {status}  (speed {self.speed_level})       \r", end='')

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print("\n\rBye!")

    def _publish(self, lin, ang):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = WasdTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
