#!/usr/bin/env python3
"""Which sensor sees the chair legs? Run this with the rover STANDING STILL.

Put a chair (or just a few legs) in the box ahead of the rover, then:

    python3 tools/check_obstacle_visibility.py                  # box 0.3-1.5 m ahead, +-0.4 m
    python3 tools/check_obstacle_visibility.py --x0 0.5 --x1 1.0 --seconds 6

For each source it counts what falls in the box and says SAW IT or MISSED:

    lidar        /scan                       (the one plane at 27.5 cm)
    depth cam    /camera/camera/depth/color/points
    front cam    /front_cam/scan
    costmap      /local_costmap/costmap      (what the controller actually uses)

The answer decides the fix. Lidar missed but the costmap-from-lidar is the only
thing the controller has: the legs are below or between the beam. Depth cam
missed: they are inside its blind zone. Front cam saw it but the costmap did not:
a layer or TF problem. Repeat with the chair at 0.5, 1.0 and 1.5 m.

It only listens; nothing moves. Start the stack (and the front camera, if you want
it in the table) first.
"""
import argparse
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'rover_core'))
from rover_core.obstacle_probe import (  # noqa: E402
    apply_transform, costmap_cells_in_window, in_window, scan_points, summarize)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--x0', type=float, default=0.3, help='box start, m ahead of base_link')
    ap.add_argument('--x1', type=float, default=1.5)
    ap.add_argument('--half-width', type=float, default=0.4)
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--min-hits', type=int, default=2)
    a = ap.parse_args()

    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.duration import Duration
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan, PointCloud2
    from sensor_msgs_py import point_cloud2
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    n = Node('check_obstacle_visibility')
    buf = Buffer()
    TransformListener(buf, n)
    pts = {'lidar': [], 'depth camera': [], 'front camera': []}
    cm = {'grid': None}
    got = {k: 0 for k in ('lidar', 'depth camera', 'front camera', 'costmap')}

    def tf_pts(p, frame):
        try:
            t = buf.lookup_transform('base_link', frame, Time(), timeout=Duration(seconds=0.2))
        except Exception:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return apply_transform(p, (tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))

    def on_scan(key):
        def cb(m):
            got[key] += 1
            p = scan_points(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max)
            b = tf_pts(p, m.header.frame_id)
            if b is not None:
                pts[key].append(in_window(b, a.x0, a.x1, a.half_width))
        return cb

    def on_cloud(m):
        got['depth camera'] += 1
        arr = point_cloud2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
        p = np.stack([arr['x'], arr['y'], arr['z']], 1)[::8]
        b = tf_pts(p, m.header.frame_id)
        if b is not None:
            pts['depth camera'].append(in_window(b, a.x0, a.x1, a.half_width, 0.05, 1.5))

    n.create_subscription(LaserScan, '/scan', on_scan('lidar'), qos_profile_sensor_data)
    n.create_subscription(LaserScan, '/front_cam/scan', on_scan('front camera'), qos_profile_sensor_data)
    n.create_subscription(PointCloud2, '/camera/camera/depth/color/points', on_cloud,
                          qos_profile_sensor_data)

    def on_grid(m):
        got['costmap'] += 1
        cm['grid'] = m
    n.create_subscription(OccupancyGrid, '/local_costmap/costmap', on_grid, 1)

    print('listening for %.0f s; box: %.1f-%.1f m ahead, +-%.1f m' % (
        a.seconds, a.x0, a.x1, a.half_width))
    end = time.time() + a.seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)

    rows = []
    for key in ('lidar', 'depth camera', 'front camera'):
        if not got[key]:
            rows.append({'source': key, 'hits': None})
            continue
        # a point must show up in several sweeps to count, not once by chance
        allp = np.vstack(pts[key]) if pts[key] else np.zeros((0, 3))
        per_sweep = len(allp) / max(1, len(pts[key]))
        s = summarize(key, allp, 0)
        s['hits'] = round(per_sweep, 1)
        s['saw_it'] = per_sweep >= a.min_hits
        rows.append(s)
    g = cm['grid']
    if g is None:
        rows.append({'source': 'costmap', 'hits': None})
    else:
        try:
            t = buf.lookup_transform(g.header.frame_id, 'base_link', Time(), timeout=Duration(seconds=0.5))
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            cells = costmap_cells_in_window(
                g.data, g.info.width, g.info.height, g.info.resolution,
                (g.info.origin.position.x, g.info.origin.position.y),
                (t.transform.translation.x, t.transform.translation.y, yaw),
                a.x0, a.x1, a.half_width)
            s = summarize('costmap', cells, a.min_hits)
            rows.append(s)
        except Exception as e:
            rows.append({'source': 'costmap', 'hits': None, 'why': str(e)[:60]})

    print('\n%-14s %-10s %-10s %s' % ('source', 'in box', 'nearest', 'verdict'))
    for r in rows:
        if r['hits'] is None:
            print('%-14s %-10s %-10s %s' % (r['source'], '-', '-', 'no data (not running?)'))
            continue
        near = '-' if not r.get('nearest_x') else '%.2f m' % r['nearest_x']
        print('%-14s %-10s %-10s %s' % (r['source'], r['hits'], near,
                                        'SAW IT' if r['saw_it'] else 'MISSED'))
    live = [r for r in rows if r['hits'] is not None]
    saw = [r['source'] for r in live if r['saw_it']]
    print('\n' + ('Seen by: ' + ', '.join(saw) if saw else 'Nothing saw it.'))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
