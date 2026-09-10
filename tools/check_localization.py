#!/usr/bin/env python3
"""Score how well the live /scan matches the loaded map.

Answers the question every Nav2 problem starts with: "is the robot actually
localised, or is the initial pose wrong?" Eyeballing Foxglove is subjective;
this gives a number.

Run it with master_navigation.launch.py up and an initial pose already set.

    python3 check_localization.py
    python3 check_localization.py --tolerance 0.15 --max-range 6.0

Reading the score (fraction of scan points landing on a mapped wall):
    >70%   well localised - safe to send goals
    40-70% roughly right - drive around, AMCL converges on MOTION not while parked
    <40%   wrong pose - re-set /initialpose where the robot physically is

A stationary robot never converges. If the score will not climb, the initial
guess is in the wrong place and no amount of driving will rescue it.
"""
import argparse, math, sys, time
import rclpy, rclpy.qos, tf2_ros
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid


class Checker(Node):
    def __init__(self):
        super().__init__('check_localization')
        self.scan = None
        self.grid = None
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(LaserScan, '/scan', self._scan, 10)
        # /map is latched (TRANSIENT_LOCAL) - a default subscription silently
        # receives nothing.
        self.create_subscription(
            OccupancyGrid, '/map', self._grid,
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))

    def _scan(self, m): self.scan = m
    def _grid(self, m): self.grid = m


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tolerance', type=float, default=0.10,
                    help='how near a mapped wall a point must land, metres (default 0.10)')
    ap.add_argument('--min-range', type=float, default=0.6,
                    help='ignore returns closer than this - excludes self-hits (default 0.6)')
    ap.add_argument('--max-range', type=float, default=8.0,
                    help='ignore returns beyond this; far points are noisy (default 8.0)')
    a = ap.parse_args()

    rclpy.init()
    n = Checker()
    t0 = time.time()
    while time.time() - t0 < 15 and (n.scan is None or n.grid is None):
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.grid is None:
        sys.exit("No /map. Is master_navigation.launch.py running?")
    if n.scan is None:
        sys.exit("No /scan. Is the lidar up?")

    g = n.grid
    W, H, res = g.info.width, g.info.height, g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    occ = {(i, j) for j in range(H) for i in range(W) if g.data[j * W + i] > 65}
    print(f"map {W}x{H} @ {res} m/cell, {len(occ)} occupied cells")

    tf = None
    for _ in range(50):
        try:
            tf = n.buf.lookup_transform('map', 'laser_frame', rclpy.time.Time())
            break
        except Exception:
            rclpy.spin_once(n, timeout_sec=0.1)
    if tf is None:
        sys.exit("No TF map->laser_frame.\n"
                 "AMCL does not publish map->odom until an initial pose is set.\n"
                 "Run:  python3 set_initial_pose.py <x> <y> <yaw_deg>")

    q = tf.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    tx, ty = tf.transform.translation.x, tf.transform.translation.y
    print(f"laser in map: x={tx:+.2f} y={ty:+.2f} yaw={math.degrees(yaw):+.1f} deg")

    tol = max(1, int(round(a.tolerance / res)))
    m = n.scan
    hit = tot = 0
    for k, r in enumerate(m.ranges):
        if not math.isfinite(r):
            continue
        if r < max(m.range_min, a.min_range) or r > min(m.range_max, a.max_range):
            continue
        ang = m.angle_min + k * m.angle_increment + yaw
        ci = int((tx + r * math.cos(ang) - ox) / res)
        cj = int((ty + r * math.sin(ang) - oy) / res)
        tot += 1
        if any((ci + di, cj + dj) in occ
               for di in range(-tol, tol + 1)
               for dj in range(-tol, tol + 1)):
            hit += 1

    pct = 100 * hit / max(tot, 1)
    print(f"\nSCAN POINTS ON MAPPED WALLS (within {a.tolerance*100:.0f} cm): "
          f"{hit}/{tot} = {pct:.1f}%")
    if pct > 70:
        print("  WELL LOCALISED - safe to send goals")
    elif pct >= 40:
        print("  ROUGHLY RIGHT - drive around; AMCL converges on motion, not while parked")
    else:
        print("  WRONG POSE - re-set /initialpose where the robot physically is")

    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
