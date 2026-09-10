#!/usr/bin/env python3
"""Tell AMCL where the robot is, so it can publish map->odom.

Until an initial pose is set, the `map` frame does not exist and the whole Nav2
stack floods the console with:

    [amcl] AMCL cannot publish a pose or update the transform. Please set the initial pose
    [local_costmap] "map" passed to lookupTransform argument target_frame does not exist

Those costmap errors are downstream noise, not separate faults. They all stop
once this runs.

    python3 set_initial_pose.py 0 0 0          # map origin, facing +X
    python3 set_initial_pose.py 1.5 -0.5 90    # x, y, yaw in DEGREES

slam_toolbox starts a mapping run with the robot at map (0,0) facing +X, so if
the robot is back near where you began mapping, "0 0 0" is a good first guess.

Afterwards check it worked - do not trust the console:
    python3 check_localization.py
"""
import argparse, math, time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('x', type=float, help='map X in metres')
    ap.add_argument('y', type=float, help='map Y in metres')
    ap.add_argument('yaw', type=float, nargs='?', default=0.0,
                    help='heading in DEGREES, 0 = +X (default 0)')
    ap.add_argument('--uncertainty', type=float, default=0.25,
                    help='position variance. Raise it if unsure - a wider spread '
                         'lets AMCL search, it just needs more motion to converge '
                         '(default 0.25)')
    a = ap.parse_args()

    rclpy.init()
    n = rclpy.create_node('set_initial_pose')
    pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    m = PoseWithCovarianceStamped()
    m.header.frame_id = 'map'
    m.pose.pose.position.x = a.x
    m.pose.pose.position.y = a.y
    th = math.radians(a.yaw)
    m.pose.pose.orientation.z = math.sin(th / 2)
    m.pose.pose.orientation.w = math.cos(th / 2)
    cov = [0.0] * 36
    cov[0] = cov[7] = a.uncertainty
    cov[35] = 0.068                      # ~15 deg heading uncertainty
    m.pose.covariance = cov

    # Republished a few times: AMCL may not have finished discovery on the
    # first message, and a single --once publish is easily missed.
    for _ in range(12):
        m.header.stamp = n.get_clock().now().to_msg()
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.05)
        time.sleep(0.1)

    print(f"published /initialpose  x={a.x} y={a.y} yaw={a.yaw} deg")
    print("AMCL converges on MOTION - drive a metre and rotate, then run "
          "check_localization.py")
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
