"""Geometry behind the obstacle-visibility check. No ROS.

    python3 -m pytest ros2_ws/src/rover_core/test/test_obstacle_probe.py
"""
import math

import numpy as np
import pytest

from rover_core.obstacle_probe import (
    apply_transform, costmap_cells_in_window, in_window, scan_points, summarize)


def test_lidar_mounted_backwards_maps_its_forward_to_the_rovers_rear():
    # URDF: lidar at (-0.155, 0, 0.275), yaw pi. A return 1.0 m dead ahead of the
    # LIDAR is 1.155 m BEHIND base_link.
    q = (0.0, 0.0, 1.0, 0.0)                                   # yaw pi
    p = apply_transform([[1.0, 0.0, 0.0]], (-0.155, 0.0, 0.275), q)
    assert p[0] == pytest.approx([-1.155, 0.0, 0.275], abs=1e-9)


def test_scan_points_drop_invalid_returns():
    p = scan_points([1.0, math.inf, math.nan, 0.05, 20.0], 0.0, math.radians(10), 0.15, 12.0)
    assert len(p) == 1 and p[0][0] == pytest.approx(1.0)


def test_window_selects_a_box_ahead():
    pts = [[0.8, 0.1, 0.2], [0.8, 0.9, 0.2], [0.1, 0.0, 0.2], [2.0, 0.0, 0.2], [0.8, 0.0, 5.0]]
    got = in_window(pts, 0.3, 1.5, 0.4)
    assert got.shape[0] == 1 and got[0][0] == 0.8


def test_summary_counts_and_threshold():
    s = summarize('x', np.array([[0.9, 0, 0], [1.1, 0, 0]]), 2)
    assert s['saw_it'] and s['nearest_x'] == 0.9
    assert not summarize('x', np.zeros((0, 3)), 1)['saw_it']


def test_costmap_cells_are_found_in_the_robot_frame():
    # 20x20 grid, 0.1 m cells, origin (-1,-1); robot at (0,0) facing +x.
    g = np.zeros((20, 20), np.int16)
    g[10, 18] = 100        # cell centre x=0.85, y=0.05  -> ahead
    g[2, 5] = 100          # x=-0.45, y=-0.75            -> behind
    cells = costmap_cells_in_window(g.ravel(), 20, 20, 0.1, (-1.0, -1.0), (0.0, 0.0, 0.0),
                                    0.3, 1.5, 0.4)
    assert cells.shape[0] == 1 and cells[0][0] == pytest.approx(0.85)


def test_costmap_window_follows_the_robots_heading():
    g = np.zeros((20, 20), np.int16)
    g[18, 10] = 100        # cell centre x=0.05, y=0.85 (left of +x)
    cells = costmap_cells_in_window(g.ravel(), 20, 20, 0.1, (-1.0, -1.0),
                                    (0.0, 0.0, math.pi / 2), 0.3, 1.5, 0.4)
    assert cells.shape[0] == 1, 'facing +y, that cell is straight ahead'
