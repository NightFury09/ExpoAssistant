#!/usr/bin/env python3
"""
rover_teleop_v2.py — WASD keyboard teleoperation with live speed control

Key bindings (latching — press once, rover keeps moving):
  W    Forward          S    Backward
  A    Turn Left        D    Turn Right
  +/=  Speed UP         -/_  Speed DOWN     (change on the go, while driving)
  SPACE / X    Stop
  ESC          Quit

Speed is a single adjustable knob: linear speed in m/s, with angular speed
kept proportional (ANG_RATIO x linear). +/- change it live; a held direction
immediately follows the new speed on the next publish.

Published: /cmd_vel  geometry_msgs/Twist  at 5 Hz
"""

import curses
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

PUBLISH_HZ = 5.0

# Speed knob (linear m/s). Angular tracks it via ANG_RATIO.
V_LIN_START = 0.40    # starting linear speed (m/s)
V_LIN_MIN   = 0.05
V_LIN_MAX   = 1.50    # firmware caps motor RPM well above this
V_LIN_STEP  = 0.05    # change per +/- keypress
ANG_RATIO   = 3.0     # angular (rad/s) = ANG_RATIO * linear (m/s)

# Movement keys set a DIRECTION only; speed is applied at publish time.
#   (lin_dir, ang_dir) each in {-1, 0, +1}
MOVE_KEYS = {
    ord('w'): ( 1,  0), ord('W'): ( 1,  0),
    ord('s'): (-1,  0), ord('S'): (-1,  0),
    ord('a'): ( 0,  1), ord('A'): ( 0,  1),
    ord('d'): ( 0, -1), ord('D'): ( 0, -1),
}

SPEED_UP_KEYS   = {ord('+'), ord('=')}
SPEED_DOWN_KEYS = {ord('-'), ord('_')}
STOP_KEYS       = {ord(' '), ord('x'), ord('X')}


class TeleopV2(Node):
    def __init__(self):
        super().__init__('rover_teleop_v2')
        self.pub   = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self.lin_dir = 0
        self.ang_dir = 0
        self.v_lin   = V_LIN_START

    @property
    def v_ang(self):
        return self.v_lin * ANG_RATIO

    @property
    def lin(self):
        return self.lin_dir * self.v_lin

    @property
    def ang(self):
        return self.ang_dir * self.v_ang

    def _publish(self):
        msg = Twist()
        msg.linear.x  = float(self.lin)
        msg.angular.z = float(self.ang)
        self.pub.publish(msg)

    def _stop(self):
        self.lin_dir = 0
        self.ang_dir = 0
        self.pub.publish(Twist())

    def _speed_up(self):
        self.v_lin = min(V_LIN_MAX, round(self.v_lin + V_LIN_STEP, 3))

    def _speed_down(self):
        self.v_lin = max(V_LIN_MIN, round(self.v_lin - V_LIN_STEP, 3))

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
                p(4, 0, "+/=  Faster       -/_  Slower")
                p(5, 0, "SPACE / X  Stop   ESC  Quit")
                p(7, 0, "─" * min(w - 1, 40))

                if   self.lin_dir > 0: state, sc = "FORWARD",       1
                elif self.lin_dir < 0: state, sc = "BACKWARD",      2
                elif self.ang_dir > 0: state, sc = "TURNING LEFT",  2
                elif self.ang_dir < 0: state, sc = "TURNING RIGHT", 2
                else:                  state, sc = "STOPPED",       3

                p(8,  0, "State  : ", curses.A_BOLD)
                p(8,  9, state, curses.A_BOLD | curses.color_pair(sc))

                # Speed knob (with a simple bar)
                frac = (self.v_lin - V_LIN_MIN) / (V_LIN_MAX - V_LIN_MIN)
                bar_n = int(round(frac * 20))
                bar = "█" * bar_n + "·" * (20 - bar_n)
                p(9,  0, f"Speed  : {self.v_lin:.2f} m/s  [{bar}]",
                  curses.A_BOLD | curses.color_pair(1))
                p(10, 0, f"         (turn {self.v_ang:.2f} rad/s)  +/- to change",
                  curses.color_pair(4))

                p(11, 0, f"Cmd    : linear.x={self.lin:+.2f}  angular.z={self.ang:+.2f}",
                  curses.color_pair(4))

                if self.lin != 0.0 or self.ang != 0.0:
                    left_ms  = self.lin - (self.ang * 0.44 / 2.0)
                    right_ms = self.lin + (self.ang * 0.44 / 2.0)
                    l_rpm = int(left_ms  * 60.0 / (2.0 * 3.14159 * 0.05))
                    r_rpm = int(right_ms * 60.0 / (2.0 * 3.14159 * 0.05))
                    p(12, 0, f"Motors : L={abs(l_rpm):3d}RPM   R={abs(r_rpm):3d}RPM",
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
                        self.lin_dir, self.ang_dir = MOVE_KEYS[key]

                    elif key in SPEED_UP_KEYS:
                        self._speed_up()

                    elif key in SPEED_DOWN_KEYS:
                        self._speed_down()

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
