"""Which sensor sees the thing in front of the rover? Pure geometry, no ROS.

Behind tools/check_obstacle_visibility.py: place a chair (or its legs) in front of
the stationary rover, and learn which of lidar, depth camera, front webcam and
costmap actually reports it, before deciding what to fix.
"""
import math

import numpy as np


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def apply_transform(pts, translation, quaternion):
    """(N,3) points in a child frame -> the parent frame."""
    R = quat_to_matrix(*quaternion)
    return np.asarray(pts, float) @ R.T + np.asarray(translation, float)


def scan_points(ranges, angle_min, angle_increment, range_min, range_max):
    """LaserScan fields -> (N,3) finite points in the scan frame (z = 0)."""
    r = np.asarray(ranges, float)
    ang = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    return np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok]), np.zeros(ok.sum())], 1)


def in_window(pts, x0, x1, half_width, z_min=-1.0, z_max=3.0):
    """Points inside the rectangle ahead of the rover (base frame)."""
    p = np.asarray(pts, float).reshape(-1, 3)
    m = ((p[:, 0] >= x0) & (p[:, 0] <= x1) & (np.abs(p[:, 1]) <= half_width) &
         (p[:, 2] >= z_min) & (p[:, 2] <= z_max))
    return p[m]


def summarize(name, pts, min_hits):
    """One row of the report: how many points, how near, did it 'see' it."""
    n = len(pts)
    return {'source': name, 'hits': n,
            'nearest_x': float(pts[:, 0].min()) if n else None,
            'saw_it': n >= min_hits}


def costmap_cells_in_window(data, width, height, resolution, origin_xy, base_xy_yaw,
                            x0, x1, half_width, threshold=99):
    """Occupied costmap cells (value >= threshold) whose centre is in the window
    ahead of the robot. The grid is in its own frame (odom); base_xy_yaw is the
    robot's pose in that frame. Returns (N,3) points in the BASE frame."""
    g = np.asarray(data, np.int16).reshape(height, width)
    rr, cc = np.nonzero(g >= threshold)
    if rr.size == 0:
        return np.zeros((0, 3))
    wx = origin_xy[0] + (cc + 0.5) * resolution
    wy = origin_xy[1] + (rr + 0.5) * resolution
    bx, by, yaw = base_xy_yaw
    dx, dy = wx - bx, wy - by
    x = dx * math.cos(-yaw) - dy * math.sin(-yaw)
    y = dx * math.sin(-yaw) + dy * math.cos(-yaw)
    return in_window(np.stack([x, y, np.zeros_like(x)], 1), x0, x1, half_width)
