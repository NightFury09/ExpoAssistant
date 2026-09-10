#!/usr/bin/env python3
"""Remove the rover's own structure from /scan, and nothing else.

Publishes /scan_filtered for SLAM, AMCL and the Nav2 costmaps.

Why this exists
---------------
The lidar sees the rover's front deck corner bumpers at about 0.43 m. Left in,
they become a phantom obstacle that travels with the robot: SLAM smears it
across the map on every turn, AMCL tries to match it against walls that are not
there, and the costmap inflates it over the robot's own footprint so the
controller believes it is boxed in.

The obvious fix - a minimum range - is wrong here. The lidar is mounted at the
REAR facing backwards, so those bumper returns sit at lidar +/-149 deg, which is
+/-30 deg from the robot's FORWARD direction. A range filter therefore blinds the
whole front arc: with obstacle_min_range 0.55 the rover drove into a 30 cm box
it could not see, having only 24 cm of margin beyond its own 31 cm footprint.

So filter on angle AND range together. A return is dropped only if it falls in a
known self-hit sector *and* is closer than the cutoff. A box straight ahead at
0.4 m is at 0 deg, outside the sectors, and passes through untouched. A distant
wall seen through a sector also passes, because it is beyond the cutoff.

Measured sectors (2026-09-07, robot stationary, per-bin medians):
    -149.5 .. -138.6 deg   median 0.449 m
    +144.5 .. +159.0 deg   median 0.419 m
Widened slightly for mounting tolerance.

Re-measure after ANY change to the lidar mount:
    python3 tools/check_lidar.py
and look for sectors whose median sits under ~0.6 m.

The real fix is mechanical: raise the lidar ~5 cm to clear the bumpers, then
this node can be dropped from the chain entirely.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        # Sectors given as [start_deg, end_deg] pairs, flattened.
        self.declare_parameter('blank_sectors_deg',
                               [-155.0, -134.0, 140.0, 163.0])
        self.declare_parameter('self_hit_max_range', 0.60)
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')

        flat = list(self.get_parameter('blank_sectors_deg').value)
        self.sectors = [(math.radians(flat[i]), math.radians(flat[i + 1]))
                        for i in range(0, len(flat) - 1, 2)]
        self.cut = float(self.get_parameter('self_hit_max_range').value)
        inp = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(LaserScan, out, qos)
        self.create_subscription(LaserScan, inp, self.cb, qos)

        self.n_msgs = 0
        self.n_dropped = 0
        secs = ', '.join(f'[{math.degrees(a):.0f},{math.degrees(b):.0f}]'
                         for a, b in self.sectors)
        self.get_logger().info(
            f"scan_filter: {inp} -> {out}; dropping returns under {self.cut} m "
            f"in sectors {secs} deg")

    def cb(self, msg: LaserScan):
        ranges = list(msg.ranges)
        dropped = 0
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r >= self.cut:
                continue
            a = msg.angle_min + i * msg.angle_increment
            for lo, hi in self.sectors:
                if lo <= a <= hi:
                    ranges[i] = float('inf')
                    dropped += 1
                    break
        msg.ranges = ranges
        self.pub.publish(msg)

        self.n_msgs += 1
        self.n_dropped += dropped
        if self.n_msgs % 200 == 0:
            self.get_logger().info(
                f"filtered {self.n_msgs} scans, "
                f"{self.n_dropped / self.n_msgs:.1f} self-hits removed per scan")


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
