#!/usr/bin/env python3
"""
rover_teleop_v2.py — WASD keyboard teleoperation (simplified)

Key bindings (latching — press once, rover keeps moving):
  W    Forward
  S    Backward
  A    Turn Left
  D    Turn Right
  SPACE / X    Stop
  ESC          Quit

Published: /cmd_vel  geometry_msgs/Twist  at 5 Hz
"""

import curses
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LINEAR_SPEED  = 0.20   # m/s
ANGULAR_SPEED = 0.60   # rad/s
PUBLISH_HZ    = 5.0

MOVE_KEYS = {
    ord('w'): ( LINEAR_SPEED,  0.0),
    ord('W'): ( LINEAR_SPEED,  0.0),
    ord('s'): (-LINEAR_SPEED,  0.0),
    ord('S'): (-LINEAR_SPEED,  0.0),
    ord('a'): ( 0.0,  ANGULAR_SPEED),
    ord('A'): ( 0.0,  ANGULAR_SPEED),
    ord('d'): ( 0.0, -ANGULAR_SPEED),
    ord('D'): ( 0.0, -ANGULAR_SPEED),
}

STOP_KEYS = {ord(' '), ord('x'), ord('X')}


class TeleopV2(Node):
    def __init__(self):
        super().__init__('rover_teleop_v2')
        self.pub   = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self.lin   = 0.0
        self.ang   = 0.0

    def _publish(self):
        msg = Twist()
        msg.linear.x  = self.lin
        msg.angular.z = self.ang
        self.pub.publish(msg)

    def _stop(self):
        self.lin = 0.0
        self.ang = 0.0
        msg = Twist()
        self.pub.publish(msg)

    def run(self, stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(0)
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN,  -1)
        curses.init_pair(2, curses.COLOR_YELLOW, -1)
        curses.init_pair(3, curses.COLOR_RED,    -1)
        curses.init_pair(4, curses.COLOR_CYAN,   -1)

        def draw():
            try:
                stdscr.erase()
                h, w = stdscr.getmaxyx()

                def p(row, col, text, attr=0):
                    try:
                        stdscr.addstr(row, col, text[:max(0, w - col - 1)], attr)
                    except curses.error:
                        pass

                p(0, 0, "=== ROVER TELEOP v2 ===", curses.A_BOLD | curses.color_pair(4))
                p(2, 0, "W  Forward        S  Backward")
                p(3, 0, "A  Turn Left      D  Turn Right")
                p(4, 0, "SPACE / X  Stop   ESC  Quit")
                p(6, 0, "─" * min(w - 1, 40))

                # State
                if   self.lin > 0:   state = "FORWARD";     sc = 1
                elif self.lin < 0:   state = "BACKWARD";    sc = 2
                elif self.ang > 0:   state = "TURNING LEFT"; sc = 2
                elif self.ang < 0:   state = "TURNING RIGHT"; sc = 2
                else:                state = "STOPPED";     sc = 3

                p(7,  0, "State  : ", curses.A_BOLD)
                p(7,  9, state, curses.A_BOLD | curses.color_pair(sc))
                p(8,  0, f"Cmd    : linear.x={self.lin:+.2f}  angular.z={self.ang:+.2f}",
                  curses.color_pair(4))

                # Show what Modbus commands will be sent
                if self.lin != 0.0 or self.ang != 0.0:
                    left_ms  = self.lin - (self.ang * 0.44 / 2.0)
                    right_ms = self.lin + (self.ang * 0.44 / 2.0)
                    l_rpm = int(left_ms  * 60.0 / (2.0 * 3.14159 * 0.05))
                    r_rpm = int(right_ms * 60.0 / (2.0 * 3.14159 * 0.05))
                    l_dir = "CW " if l_rpm >= 0 else "CCW"
                    r_dir = "CCW" if r_rpm >= 0 else "CW "  # right motor physically inverted
                    p(9, 0, f"Motors : L={l_dir} {abs(l_rpm):3d}RPM   R={r_dir} {abs(r_rpm):3d}RPM",
                      curses.color_pair(4))

                stdscr.refresh()
            except curses.error:
                pass

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

                while True:
                    key = stdscr.getch()
                    if key == -1:
                        break

                    if key == 27:           # ESC
                        stdscr.timeout(10)
                        nk = stdscr.getch()
                        stdscr.timeout(0)
                        if nk == -1:
                            return

                    elif key == 3:          # Ctrl+C
                        return

                    elif key in MOVE_KEYS:
                        self.lin, self.ang = MOVE_KEYS[key]

                    elif key in STOP_KEYS:
                        self._stop()

                draw()
        finally:
            self._stop()


def main(args=None):
    if not sys.stdin.isatty():
        print("Run in an interactive terminal: ros2 run rover_core rover_teleop_v2")
        sys.exit(1)
    rclpy.init(args=args)
    node = TeleopV2()
    try:
        curses.wrapper(node.run)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        print("rover_teleop_v2 exited.")


if __name__ == '__main__':
    main()
