#!/usr/bin/env python3
"""Find safe Nav2 goals: cells the planner can actually reach.

Reads /global_costmap/costmap, NOT /map. That distinction matters: a cell can
be perfectly free on the map yet unreachable, because the costmap inflates every
obstacle by inflation_radius and the robot's CENTRE may not enter the inscribed
band. An earlier version of this script read /map and happily recommended goals
with costmap cost 99 - the planner could never reach them and Nav2 just span in
recovery behaviours.

Costmap values (Nav2 publishes OccupancyGrid scaled 0-100, not the internal
0-255):
      0     free
   1-89     inflation - passable but discouraged
  90-99     inscribed - the robot's CENTRE cannot be here
    100     lethal
     -1     unknown

    python3 find_goal.py
    python3 find_goal.py --min-dist 2.0 --max-dist 5.0 --clearance 0.5
    python3 find_goal.py --spread          # spread-out options, not 5 neighbours

Send one with:
    python3 send_goal.py <x> <y> [yaw_deg]
"""
import argparse, math, sys, time
import rclpy, rclpy.qos, tf2_ros
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class Finder(Node):
    def __init__(self):
        super().__init__('find_goal')
        self.grid = None
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._g,
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))

    def _g(self, m): self.grid = m


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--min-dist', type=float, default=1.5, help='metres (default 1.5)')
    ap.add_argument('--max-dist', type=float, default=3.0, help='metres (default 3.0)')
    ap.add_argument('--clearance', type=float, default=0.4,
                    help='surrounding cells must also be below --max-cost, metres '
                         '(default 0.4). The costmap already accounts for the '
                         'footprint via inflation, so this is margin ON TOP of that.')
    ap.add_argument('--max-cost', type=int, default=50,
                    help='highest acceptable costmap value, 0-100 (default 50). '
                         'Anything >=90 is inscribed and unreachable.')
    ap.add_argument('--count', type=int, default=5, help='how many to list (default 5)')
    ap.add_argument('--spread', action='store_true',
                    help='spread results at least 1 m apart instead of listing neighbours')
    a = ap.parse_args()

    rclpy.init()
    n = Finder()
    t0 = time.time()
    while time.time() - t0 < 15 and n.grid is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.grid is None:
        sys.exit("No /global_costmap/costmap.\n"
                 "Is master_navigation.launch.py running, and has an initial pose\n"
                 "been set? The costmap needs the map frame to exist.")

    g = n.grid
    W, H, res = g.info.width, g.info.height, g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y

    def val(i, j):
        return g.data[j * W + i] if 0 <= i < W and 0 <= j < H else -1

    def usable(i, j):
        v = val(i, j)
        return 0 <= v <= a.max_cost

    rx = ry = 0.0
    for _ in range(50):
        try:
            tf = n.buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            rx, ry = tf.transform.translation.x, tf.transform.translation.y
            break
        except Exception:
            rclpy.spin_once(n, timeout_sec=0.1)
    ri, rj = int((rx - ox) / res), int((ry - oy) / res)
    rc = val(ri, rj)
    print(f"robot at map ({rx:+.2f}, {ry:+.2f})  costmap cost here = {rc}")
    if rc >= 90:
        print("  WARNING: the robot's own cell is inscribed/lethal. Planning will\n"
              "  struggle. Move it somewhere opener, or reduce inflation_radius.")
    elif rc > 50:
        print("  NOTE: the robot sits deep in an inflation zone; paths out may be\n"
              "  convoluted.")

    c = int(a.clearance / res)
    step = max(1, c // 4)
    found = []
    for j in range(H):
        for i in range(W):
            if not usable(i, j):
                continue
            px, py = ox + i * res, oy + j * res
            d = math.hypot(px - rx, py - ry)
            if not (a.min_dist < d < a.max_dist):
                continue
            ok = True
            for dj in range(-c, c + 1, step):
                for di in range(-c, c + 1, step):
                    if not usable(i + di, j + dj):
                        ok = False
                        break
                if not ok:
                    break
            if ok:
                found.append((d, px, py))

    found.sort()
    print(f"\n{len(found)} cells have cost <= {a.max_cost} with "
          f"{a.clearance} m of equally clear surroundings, "
          f"{a.min_dist}-{a.max_dist} m away")
    if not found:
        print("None found. The robot may be boxed in by inflation. Try:\n"
              "  --clearance 0.2 --max-cost 70     accept tighter, costlier cells\n"
              "  --min-dist 0.8 --max-dist 5.0     widen the search\n"
              "If still nothing, inflation_radius in nav2_params.yaml is likely too\n"
              "large for this space, or the robot needs moving somewhere opener.")
        return

    shown = []
    for d, px, py in found:
        if a.spread and any(math.hypot(px - x, py - y) < 1.0 for _, x, y in shown):
            continue
        shown.append((d, px, py))
        if len(shown) >= a.count:
            break

    print()
    for d, px, py in shown:
        print(f"  x={px:+.2f}  y={py:+.2f}   ({d:.2f} m away)")
    d, px, py = shown[0]
    print(f"\nSend the first with:\n  python3 send_goal.py {px:.2f} {py:.2f}")

    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
