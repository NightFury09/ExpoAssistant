#!/usr/bin/env python3
"""
rover_teleop.py  —  WASD keyboard teleoperation for the rover.
Publishes: /cmd_vel (geometry_msgs/Twist)

Key Bindings (latching — press once, rover keeps moving):
  W / S    — Forward / Backward
  A / D    — Turn Left / Turn Right
  Q / E    — Linear speed  ▲ / ▼
  Z / C    — Angular speed ▲ / ▼
  SPACE    — Emergency stop
  X        — Full stop + reset speeds
  ESC      — Quit

Architecture:
  - curses handles the terminal (works in any real terminal).
  - A ROS timer publishes /cmd_vel at PUBLISH_RATE_HZ.
  - The main loop calls rclpy.spin_once(timeout_sec=SPIN_SEC) so the
    timer fires reliably even when no key is pressed.
"""

import curses
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ---------------------------------------------------------------------------
# Speed limits
# ---------------------------------------------------------------------------
LINEAR_SPEED_DEFAULT  = 0.20   # m/s
ANGULAR_SPEED_DEFAULT = 0.60   # rad/s
LINEAR_SPEED_STEP     = 0.05
ANGULAR_SPEED_STEP    = 0.10
LINEAR_SPEED_MIN      = 0.05
LINEAR_SPEED_MAX      = 0.60
ANGULAR_SPEED_MIN     = 0.10
ANGULAR_SPEED_MAX     = 2.00

PUBLISH_RATE_HZ = 10.0          # /cmd_vel publish frequency
SPIN_SEC        = 0.05          # rclpy spin window per loop iteration (50ms)
                                # Must be ≤ 1/PUBLISH_RATE_HZ for reliable publish

# ---------------------------------------------------------------------------
# Key map  (curses keycode → motion factors)
# ---------------------------------------------------------------------------
MOVE_KEYS = {
    ord('w'): ( 1.0,  0.0),   # forward
    ord('W'): ( 1.0,  0.0),
    ord('s'): (-1.0,  0.0),   # backward
    ord('S'): (-1.0,  0.0),
    ord('a'): ( 0.0,  1.0),   # turn left (CCW)
    ord('A'): ( 0.0,  1.0),
    ord('d'): ( 0.0, -1.0),   # turn right (CW)
    ord('D'): ( 0.0, -1.0),
}

SPEED_KEYS = {
    ord('q'): ('linear',  +1),
    ord('Q'): ('linear',  +1),
    ord('e'): ('linear',  -1),
    ord('E'): ('linear',  -1),
    ord('z'): ('angular', +1),
    ord('Z'): ('angular', +1),
    ord('c'): ('angular', -1),
    ord('C'): ('angular', -1),
}

STOP_KEYS = {ord(' '), ord('x'), ord('X')}


class RoverTeleop(Node):
    def __init__(self):
        super().__init__('rover_teleop')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer fires at PUBLISH_RATE_HZ; serviced by rclpy.spin_once in run()
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_cmd)

        self.linear_speed  = LINEAR_SPEED_DEFAULT
        self.angular_speed = ANGULAR_SPEED_DEFAULT
        self.lin_x = 0.0   # direction factors (−1, 0, +1)
        self.ang_z = 0.0

    # -----------------------------------------------------------------------
    def _publish_cmd(self):
        """Timer callback — publishes /cmd_vel at a fixed rate."""
        msg = Twist()
        msg.linear.x  = self.lin_x * self.linear_speed
        msg.angular.z = self.ang_z * self.angular_speed
        self.pub.publish(msg)

    # -----------------------------------------------------------------------
    def _send_stop(self):
        """Immediately publish a zero-velocity message (safety stop)."""
        self.lin_x = 0.0
        self.ang_z = 0.0
        try:
            msg = Twist()        # all fields default to 0.0
            self.pub.publish(msg)
        except Exception:
            pass                # ignore if ROS context already gone

    # -----------------------------------------------------------------------
    def run(self, stdscr):
        """Main curses event loop — owns the terminal while active."""
        curses.curs_set(0)
        stdscr.nodelay(True)    # getch() does not block
        stdscr.timeout(0)       # return immediately if no key; spin handles timing

        # Colour pairs
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN,  -1)   # moving
        curses.init_pair(2, curses.COLOR_YELLOW, -1)   # turning / labels
        curses.init_pair(3, curses.COLOR_RED,    -1)   # stop / warnings
        curses.init_pair(4, curses.COLOR_CYAN,   -1)   # header / info

        def draw():
            try:
                stdscr.erase()
                h, w = stdscr.getmaxyx()
                max_col = max(0, w - 2)

                def safe_str(row, col, text, attr=0):
                    try:
                        stdscr.addstr(row, col, text[:max_col - col], attr)
                    except curses.error:
                        pass

                title = " ROVER WASD TELEOP — ROS 2 "
                safe_str(0, max(0, (w - len(title)) // 2), title,
                         curses.A_BOLD | curses.color_pair(4))
                safe_str(1, 0, "─" * min(w - 1, 54))

                safe_str(2,  2, "W / S",  curses.A_BOLD | curses.color_pair(2))
                safe_str(2,  9, "Forward / Backward")
                safe_str(3,  2, "A / D",  curses.A_BOLD | curses.color_pair(2))
                safe_str(3,  9, "Turn Left / Turn Right")
                safe_str(4,  2, "Q / E",  curses.A_BOLD | curses.color_pair(2))
                safe_str(4,  9, "Linear speed  ▲ / ▼")
                safe_str(5,  2, "Z / C",  curses.A_BOLD | curses.color_pair(2))
                safe_str(5,  9, "Angular speed ▲ / ▼")
                safe_str(6,  2, "SPACE",  curses.A_BOLD | curses.color_pair(3))
                safe_str(6,  9, "Emergency Stop")
                safe_str(7,  2, "X    ",  curses.A_BOLD | curses.color_pair(3))
                safe_str(7,  9, "Stop + reset speeds")
                safe_str(8,  2, "ESC  ",  curses.A_BOLD)
                safe_str(8,  9, "Quit")
                safe_str(9,  0, "─" * min(w - 1, 54))

                # Current state
                lv = self.lin_x * self.linear_speed
                av = self.ang_z * self.angular_speed
                if self.lin_x > 0:
                    state_str, state_col = "▲  FORWARD",      curses.color_pair(1)
                elif self.lin_x < 0:
                    state_str, state_col = "▼  BACKWARD",     curses.color_pair(2)
                elif self.ang_z > 0:
                    state_str, state_col = "◄  TURNING LEFT",  curses.color_pair(2)
                elif self.ang_z < 0:
                    state_str, state_col = "►  TURNING RIGHT", curses.color_pair(2)
                else:
                    state_str, state_col = "■  STOPPED",      curses.color_pair(3)

                safe_str(10, 2, "State     : ", curses.A_BOLD)
                safe_str(10, 14, state_str, curses.A_BOLD | state_col)

                safe_str(11, 2,
                         f"Lin: {self.linear_speed:.2f} m/s   "
                         f"Ang: {self.angular_speed:.2f} rad/s",
                         curses.color_pair(4))

                pub_col = curses.color_pair(1) if (lv != 0 or av != 0) else curses.color_pair(3)
                safe_str(12, 2,
                         f"Cmd  →  linear.x={lv:+.3f}  angular.z={av:+.3f}",
                         pub_col)
                safe_str(13, 0, "─" * min(w - 1, 54))
                stdscr.refresh()
            except curses.error:
                pass    # terminal too small — skip this frame

        # -------------------------------------------------------------------
        # Main event loop
        # -------------------------------------------------------------------
        try:
            while rclpy.ok():
                # Let ROS service its timer (publishes /cmd_vel) and any
                # other callbacks. timeout_sec keeps this bounded so curses
                # stays responsive.
                rclpy.spin_once(self, timeout_sec=SPIN_SEC)

                # Read all pending keys (non-blocking)
                while True:
                    key = stdscr.getch()
                    if key == -1:
                        break   # no more pending keys

                    if key == 27:           # ESC — check for escape sequences
                        stdscr.timeout(10)
                        nk = stdscr.getch()
                        stdscr.timeout(0)
                        if nk == -1:
                            # Standalone ESC → quit
                            return
                        # else it was an arrow-key sequence — ignore

                    elif key == 3:          # Ctrl+C
                        return

                    elif key in MOVE_KEYS:
                        self.lin_x, self.ang_z = MOVE_KEYS[key]

                    elif key in SPEED_KEYS:
                        axis, delta = SPEED_KEYS[key]
                        if axis == 'linear':
                            self.linear_speed = round(
                                max(LINEAR_SPEED_MIN,
                                    min(LINEAR_SPEED_MAX,
                                        self.linear_speed + delta * LINEAR_SPEED_STEP)), 3)
                        else:
                            self.angular_speed = round(
                                max(ANGULAR_SPEED_MIN,
                                    min(ANGULAR_SPEED_MAX,
                                        self.angular_speed + delta * ANGULAR_SPEED_STEP)), 3)

                    elif key in STOP_KEYS:
                        self.lin_x = 0.0
                        self.ang_z = 0.0
                        if key in (ord('x'), ord('X')):
                            self.linear_speed  = LINEAR_SPEED_DEFAULT
                            self.angular_speed = ANGULAR_SPEED_DEFAULT

                draw()

        finally:
            self._send_stop()


# ---------------------------------------------------------------------------
def main(args=None):
    if not sys.stdin.isatty():
        print("[rover_teleop] ERROR: stdin is not a real terminal.")
        print("Run this in an interactive terminal window:")
        print("  ros2 run rover_core rover_teleop")
        sys.exit(1)

    rclpy.init(args=args)
    node = RoverTeleop()

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
        print("[rover_teleop] Exited cleanly.")


if __name__ == '__main__':
    main()
