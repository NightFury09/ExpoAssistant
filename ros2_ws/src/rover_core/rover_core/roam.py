"""Where should the rover roam to next? Pure numpy/scipy -- no ROS.

Kept apart from rover_dashboard so it can be tested on a saved map, with no
rover and no ROS graph. The dashboard owns the safety rules and the goal
sending; this only answers "pick a spot".

A spot is good when it is
  * well clear of walls and of anything never surveyed. Nav2 treats the rover
    as a 0.33 m circle, so a goal needs more room than that or the planner
    rejects it, or accepts it and the rover grinds against the wall;
  * in the same connected piece of floor as the rover. A clear patch behind a
    closed door is not a destination, and sending one only teaches Nav2 to
    abort;
  * a worthwhile drive away, so "roaming" is not a series of shuffles;
  * away from the last few places it went, so it covers the room instead of
    ping-ponging between two corners.
"""
import math

import numpy as np
from scipy import ndimage

# A cell is unusable if it is unexplored or not clearly free. 25 matches how
# the console draws "drivable" (0-25); anything above is ambiguous or a wall.
FREE_MAX = 25


def clearance_map(grid, res):
    """Metres from each cell to the nearest wall or unexplored cell."""
    blocked = (grid < 0) | (grid > FREE_MAX)
    return ndimage.distance_transform_edt(~blocked) * res


def costmap_to_grid(raw):
    """Nav2's raw costmap bytes -> the occupancy scale pick_goal reads.

    Raw: 0 free, 1-252 inflation gradient, 253 inscribed (the rover cannot be
    centred here), 254 lethal, 255 unknown. Out: -1 unknown, 100 for 253+, and the
    gradient scaled onto 0-98, so the same FREE_MAX threshold means "low cost".
    """
    raw = np.asarray(raw, dtype=np.int32)
    return np.where(raw == 255, -1,
                    np.where(raw >= 253, 100, raw * 98 // 252)).astype(np.int16)


def pick_goal(grid, meta, pose, recent=(), rng=None, clearance_m=0.5,
              min_dist=1.0, max_dist=5.0, samples=40, seed_radius_m=1.0,
              rejected=()):
    """Choose the next roam goal.

    grid    int array (rows, cols); row 0 is the LOWEST y, as in an OccupancyGrid.
            The saved map, or -- better, because it is what the planner uses --
            Nav2's costmap through costmap_to_grid().
    meta    {'res', 'ox', 'oy'}  metres per cell and the map-frame origin
    pose    (x, y) of the rover in the map frame
    recent  (x, y) of recently visited goals, oldest first
    rejected (x, y) of spots the planner has already refused. Unlike `recent`,
            never relaxed: re-offering a spot the planner said no to only
            wastes a round trip.

    Returns ({'x', 'y', 'yaw'}, None) -- yaw in degrees, facing the direction of
    travel -- or (None, reason) when there is nowhere sensible to go.
    """
    rng = rng or np.random.default_rng()
    res, ox, oy = float(meta['res']), float(meta['ox']), float(meta['oy'])
    h, w = grid.shape

    ok = clearance_map(grid, res) >= clearance_m
    if not ok.any():
        return None, 'no part of the map is clear enough to drive in'

    # Where the rover is, in cells. It may be parked closer to a wall than the
    # goal clearance, so snap to the nearest clear cell -- but not from far
    # away: a rover sitting off the map is a localisation fault, not a spot.
    rc = int((pose[0] - ox) / res)
    rr = int((pose[1] - oy) / res)
    if not (0 <= rr < h and 0 <= rc < w):
        return None, 'the rover is outside the map'
    if grid[rr, rc] > FREE_MAX:
        # Never right: the lidar clears the cell it sits in. It means the pose
        # is wrong, and a goal picked from a wrong pose is a goal in a wall.
        return None, 'the rover appears to be inside a wall on the map'
    dist, (ir, ic) = ndimage.distance_transform_edt(~ok, return_indices=True)
    if dist[rr, rc] * res > seed_radius_m:
        return None, 'the rover is not on a clear part of the map'
    labels, _ = ndimage.label(ok, structure=np.ones((3, 3)))
    region = labels == labels[ir[rr, rc], ic[rr, rc]]

    rows, cols = np.nonzero(region)
    xs = ox + (cols + 0.5) * res
    ys = oy + (rows + 0.5) * res
    d_robot = np.hypot(xs - pose[0], ys - pose[1])

    def candidates(lo, hi, avoid):
        m = (d_robot >= lo) & (d_robot <= hi)
        for rx, ry in rejected:
            m &= np.hypot(xs - rx, ys - ry) >= 0.6
        # Not on top of somewhere it just was.
        for rx, ry in avoid:
            m &= np.hypot(xs - rx, ys - ry) >= 1.0
        return np.nonzero(m)[0]

    # Prefer a real drive that is clear of everywhere it has been lately. In a
    # small room that can be impossible -- six remembered spots rule out the
    # whole floor -- so give ground in steps: first the distance band, then
    # forget the oldest spots. Giving up is for a floor with nowhere to go at
    # all, not for one it has simply been to recently.
    idx = np.empty(0, int)
    for keep in sorted({len(recent), 3, 1, 0}, reverse=True):
        avoid = list(recent)[len(recent) - keep:] if keep else []
        idx = candidates(min_dist, max_dist, avoid)
        if idx.size == 0:
            idx = candidates(0.6, float('inf'), avoid)
        if idx.size:
            break
    if idx.size == 0:
        return None, 'nowhere new to go from here'

    # Of a random handful, take the one furthest from where it has been lately:
    # random enough to look like roaming, spread enough to cover the room.
    pick = rng.choice(idx, size=min(samples, idx.size), replace=False)
    seen = [pose] + list(recent)
    far = [min(math.hypot(xs[i] - sx, ys[i] - sy) for sx, sy in seen)
           for i in pick]
    i = int(pick[int(np.argmax(far))])
    gx, gy = float(xs[i]), float(ys[i])
    yaw = math.degrees(math.atan2(gy - pose[1], gx - pose[0]))
    return {'x': round(gx, 3), 'y': round(gy, 3), 'yaw': round(yaw, 1)}, None


def load_pgm_map(yaml_path):
    """A saved map as (grid, meta), shaped like the live /map -- for tests and
    for trying the picker offline."""
    import os
    import yaml
    from PIL import Image
    with open(yaml_path) as f:
        y = yaml.safe_load(f)
    img = np.array(Image.open(os.path.join(os.path.dirname(yaml_path), y['image'])))
    # trinary pgm: 254 free, 0 occupied, 205 unknown. Flip: pgm row 0 is the top.
    p = (255 - img.astype(np.float32)) / 255.0
    if y.get('negate'):
        p = 1.0 - p
    grid = np.full(img.shape, -1, np.int16)
    grid[p < y.get('free_thresh', 0.25)] = 0
    grid[p > y.get('occupied_thresh', 0.65)] = 100
    # Same thresholds map_server applies. Unknown is 205 (p ~ 0.196), which
    # stays -1 only because this map's free_thresh is 0.19: at the usual 0.25 it
    # would read as free, exactly as map_server would load it.
    meta = {'res': y['resolution'], 'ox': y['origin'][0], 'oy': y['origin'][1]}
    return grid[::-1].copy(), meta
