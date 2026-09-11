#!/usr/bin/env python3
"""Remove the rover's own structure from /scan, and nothing else.

Sits between the lidar and everything else: rplidar publishes /scan_raw, this
publishes /scan. SLAM, AMCL, both costmaps and the console are unchanged.

Why this exists
---------------
The mast that carries the screen stands in the lidar's field of view and
returns a hit on every sweep. Nothing downstream can tell that from a real
obstacle, so the costmap marks it -- and because it travels with the robot, the
mark travels too. Inflated by the robot's own inscribed radius it blankets the
footprint from every side. Measured on the running rover, parked in open floor:

    cost 99 -- inscribed, "any pose here collides" -- in EVERY direction, as
    close as 7 cm from the robot centre, 70% of the local costmap at cost >= 90

RegulatedPurePursuit throttles to its minimum speed when costs are that high
(`use_cost_regulated_linear_velocity_scaling`), the planner keeps routing around
a phantom, and recoveries fire against an obstacle that cannot be escaped
because it moves with you. That is the "there was space but it struggled to
move and was very slow to correct" symptom, and no amount of controller tuning
fixes it while the phantom is there.

The obvious fix -- a global minimum range -- is wrong, and has already been
tried here. Setting obstacle_min_range to 0.55 m blinded the rover inside 55 cm
in EVERY direction and it drove into a 30 cm box, having only 24 cm of margin
beyond its own footprint. See TROUBLESHOOTING.md.

So filter on angle AND range together, as narrowly as the measurement allows:

  * a few degrees wide, centred where the mast actually measures;
  * only below the cutoff -- a real obstacle further out on the same bearing
    passes through untouched;
  * every other bearing keeps full sensitivity down to the lidar's own 0.15 m.

Measured over 40 consecutive scans with the rover stationary:

    lidar-frame -3.2 .. +0.7 deg, range 0.213-0.215 m, in 84% of scans,
    total spread 2 mm

A 2 mm spread across 40 sweeps is a rigid body co-moving with the lidar, not
furniture -- which is exactly the check the disproved bumper theory failed. The
defaults add ~3 deg of margin either side and cut off well above the mast.

Re-measure after ANY change to the mast, the lidar mount or the deck:
    python3 tools/check_lidar.py
and look for a bearing whose returns are both close and unchanging.

Blanked rays are published as +inf, which every consumer reads as "no
measurement". The costmap will not mark there, and just as important will not
raytrace-CLEAR through there, so this cannot erase a real obstacle behind the
wedge.

NOTE: the wedge points backwards. The rover does not reverse in normal driving
(`allow_reversing: false`); the BackUp recovery does, 0.25 m at 0.10 m/s, and
during that it is blind to anything inside 35 cm directly astern. That is still
strictly better than the status quo, where the mast made the whole rear
permanently lethal and BackUp was refused outright.

The real fix is mechanical: move the mast out of the lidar plane, or raise the
lidar above it, and this node can be dropped from the chain entirely.
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        # Sectors given as [start_deg, end_deg] pairs, flattened. These were
        # the bumper sectors, which a later stationary sweep disproved -- 0.0%
        # of returns under 0.6 m in every sector meant those were furniture the
        # rover happened to be parked beside. The mast is the real self-hit and
        # is the only thing blanked now.
        self.declare_parameter('blank_sectors_deg', [-6.0, 4.0])
        self.declare_parameter('self_hit_max_range', 0.35)
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')

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
        self._mask = None
        secs = ', '.join(f'[{math.degrees(a):.0f},{math.degrees(b):.0f}]'
                         for a, b in self.sectors)
        self.get_logger().info(
            f"scan_filter: {inp} -> {out}; dropping returns under {self.cut} m "
            f"in sectors {secs} deg")

    def cb(self, msg: LaserScan):
        r = np.asarray(msg.ranges, dtype=np.float32)
        if self._mask is None or len(self._mask) != len(r):
            ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
            m = np.zeros(len(r), bool)
            for lo, hi in self.sectors:
                m |= (ang >= lo) & (ang <= hi)
            self._mask = m
            self.get_logger().info(
                f"{int(m.sum())} of {len(r)} rays fall in the blanked wedge")

        hit = self._mask & np.isfinite(r) & (r > 0.0) & (r < self.cut)
        dropped = int(hit.sum())
        if dropped:
            r = r.copy()
            r[hit] = float('inf')
            msg.ranges = r.tolist()
        self.pub.publish(msg)

        self.n_msgs += 1
        self.n_dropped += dropped
        if self.n_msgs % 400 == 0:
            per = self.n_dropped / self.n_msgs
            # Zero means the mast is no longer where it was measured. Silence
            # would look like "the filter is working"; it is not.
            log = self.get_logger().warn if per < 0.2 else self.get_logger().info
            log(f"filtered {self.n_msgs} scans, "
                f"{per:.1f} self-hits removed per scan")
            self.n_msgs = 0
            self.n_dropped = 0


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
