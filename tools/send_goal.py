#!/usr/bin/env python3
"""Send a Nav2 goal to /goal_pose.

    python3 send_goal.py 1.68 0.02          # drive there, any final heading
    python3 send_goal.py 1.68 0.02 90       # arrive facing +Y
    python3 send_goal.py --cancel           # stop: goal = current position

Before sending:
  * localisation above ~70%    -> python3 check_localization.py
  * NO teleop running          -> ros2 topic info /cmd_vel must show exactly
                                  ONE publisher. rover_teleop_v2 publishes zeros
                                  on a timer and will fight the controller, so
                                  the robot twitches and goes nowhere.
  * pick a reachable target    -> python3 find_goal.py

To stop in a hurry, Ctrl+C the launch: the ESP32's 1-second watchdog halts the
motors as soon as /cmd_vel goes quiet.
"""
import argparse, math, sys, time
import rclpy, tf2_ros
from geometry_msgs.msg import PoseStamped


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('x', type=float, nargs='?', help='map X in metres')
    ap.add_argument('y', type=float, nargs='?', help='map Y in metres')
    ap.add_argument('yaw', type=float, nargs='?', default=0.0,
                    help='final heading in DEGREES (default 0)')
    ap.add_argument('--cancel', action='store_true',
                    help='send the robot to where it already is, halting it')
    a = ap.parse_args()

    rclpy.init()
    n = rclpy.create_node('send_goal')
    pub = n.create_publisher(PoseStamped, '/goal_pose', 10)

    if a.cancel:
        buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
        tf = None
        for _ in range(50):
            try:
                tf = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                break
            except Exception:
                rclpy.spin_once(n, timeout_sec=0.1)
        if tf is None:
            sys.exit("No TF map->base_footprint; cannot determine current pose.")
        x, y, yaw = tf.transform.translation.x, tf.transform.translation.y, 0.0
        print(f"cancelling: goal set to current position ({x:+.2f}, {y:+.2f})")
    else:
        if a.x is None or a.y is None:
            sys.exit("Need x and y. See --help, or run find_goal.py for options.")
        x, y, yaw = a.x, a.y, a.yaw

    m = PoseStamped()
    m.header.frame_id = 'map'
    m.pose.position.x = x
    m.pose.position.y = y
    th = math.radians(yaw)
    m.pose.orientation.z = math.sin(th / 2)
    m.pose.orientation.w = math.cos(th / 2)
    for _ in range(5):
        m.header.stamp = n.get_clock().now().to_msg()
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.05)
        time.sleep(0.1)

    print(f"published /goal_pose  x={x:.2f} y={y:.2f} yaw={yaw} deg")
    print("Expect 'Begin navigating from current location to (...)' from bt_navigator,")
    print("and a path on /plan. If nothing moves, check /cmd_vel has one publisher.")
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
