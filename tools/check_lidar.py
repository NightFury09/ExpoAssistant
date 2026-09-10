#!/usr/bin/env python3
"""Judge lidar data quality while the robot is STATIONARY.

The average valid-return rate hides the problem that matters. On 2026-09-07 the
lidar averaged 36% valid returns and looked merely "a bit noisy" - but NOT ONE
of 360 bins returned reliably, so every wall flickered in and out and scan
matching could never converge. Per-bin reliability is the number to read.

    python3 check_lidar.py
    python3 check_lidar.py --scans 30

Keep the robot still - motion invalidates the comparison.

What it reports
  1. Per-bin reliability - how often each angle returns anything.
     Healthy: a large "always" group. If NOTHING is reliable, the scan mode is
     probably too low-resolution. This rover is an A1M8: use Express (4 kHz),
     never Standard (2 kHz), and never Sensitivity/Boost (A2/A3 only - they
     produce error 80008004).
  2. Median range per sector - a sector stuck under ~0.6 m is the robot seeing
     ITSELF. Chassis, bumpers or a mast in the beam become a phantom obstacle
     that travels with the robot and smears the map on every turn.
     Fix by moving the lidar; failing that raise min_laser_range in
     config/slam_params.yaml.
"""
import argparse, math, statistics, sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class L(Node):
    def __init__(self):
        super().__init__('check_lidar')
        self.ms = []
        self.create_subscription(LaserScan, '/scan', lambda m: self.ms.append(m), 10)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--scans', type=int, default=20, help='scans to collect (default 20)')
    ap.add_argument('--sectors', type=int, default=12, help='sectors for the range table (default 12)')
    a = ap.parse_args()

    rclpy.init()
    n = L()
    t0 = time.time()
    while time.time() - t0 < 40 and len(n.ms) < a.scans:
        rclpy.spin_once(n, timeout_sec=0.1)
    if len(n.ms) < 3:
        sys.exit("Not enough scans on /scan. Is the lidar running?")

    ms = n.ms[-a.scans:]
    K, N = len(ms), len(ms[0].ranges)
    inc = math.degrees(ms[0].angle_increment)
    print(f"{K} scans, {N} bins, {inc:.2f} deg/bin, range {ms[0].range_min:.2f}-{ms[0].range_max:.1f} m")

    def ok(m, r):
        return math.isfinite(r) and m.range_min < r < m.range_max

    hit = [0] * N
    for m in ms:
        for i, r in enumerate(m.ranges):
            if ok(m, r):
                hit[i] += 1
    total = sum(hit)
    print(f"overall valid returns: {total}/{K*N} = {100*total/(K*N):.1f}%")

    buckets = {'never (0%)': 0, '1-25%': 0, '26-50%': 0,
               '51-75%': 0, '76-99%': 0, 'ALWAYS (100%)': 0}
    for h in hit:
        p = 100 * h / K
        if p == 0:              buckets['never (0%)'] += 1
        elif p <= 25:           buckets['1-25%'] += 1
        elif p <= 50:           buckets['26-50%'] += 1
        elif p <= 75:           buckets['51-75%'] += 1
        elif p < 100:           buckets['76-99%'] += 1
        else:                   buckets['ALWAYS (100%)'] += 1
    print("\nPER-BIN RELIABILITY (the number that matters):")
    for k, v in buckets.items():
        print(f"  {k:15s} {v:4d} bins  {'#' * (v * 40 // max(N,1))}")
    always = buckets['ALWAYS (100%)']
    if always == 0:
        print("  !! NO bin is reliable. Walls flicker; scan matching cannot converge.")
        print("     Check scan_mode - on an A1M8 use Express, not Standard.")
    else:
        print(f"  {always} bins ({100*always/N:.0f}%) return on every scan.")

    print("\nMEDIAN RANGE PER SECTOR (0 deg = lidar forward):")
    S = a.sectors
    width = 360 / S
    for s in range(S):
        lo = -180 + s * width
        vals = []
        for m in ms:
            for i, r in enumerate(m.ranges):
                ang = math.degrees(m.angle_min + i * m.angle_increment)
                if lo <= ang < lo + width and ok(m, r):
                    vals.append(r)
        if not vals:
            print(f"  {lo:+5.0f}..{lo+width:+5.0f}      no returns  <== fully blocked?")
            continue
        md = statistics.median(vals)
        near = 100 * sum(1 for v in vals if v < 0.6) / len(vals)
        flag = "  <== SELF-HIT (robot seeing itself)" if near > 70 else ""
        print(f"  {lo:+5.0f}..{lo+width:+5.0f}  {len(vals):5d} pts  median {md:5.2f} m  "
              f"{near:5.1f}% under 0.6 m{flag}")

    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
