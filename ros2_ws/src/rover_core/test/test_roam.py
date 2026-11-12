"""The free-roam goal picker, on synthetic maps. No ROS, no rover.

    python3 -m pytest ros2_ws/src/rover_core/test/test_roam.py
"""
import math

import numpy as np
import pytest

from rover_core.roam import clearance_map, pick_goal

RES = 0.05
META = {'res': RES, 'ox': 0.0, 'oy': 0.0}


def room():
    """6 m x 4 m of free floor inside a wall, with a sealed 1 m x 1 m cell in
    one corner that the rover cannot reach."""
    g = np.zeros((80, 120), np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    g[20:40, 100:104] = 100                 # wall of the sealed cell
    g[20:40, 116:118] = 100
    g[20:22, 100:118] = 100
    g[38:40, 100:118] = 100
    return g


def split_room():
    """Two halves joined only by a slot too narrow to drive, past an
    unsurveyed strip."""
    g = np.zeros((80, 120), np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    g[:, 60:64] = -1
    g[:15, 60:64] = 0                       # a 0.75 m gap: under 2 x 0.5 m clearance
    return g


def test_goal_is_on_clear_free_floor():
    g = room()
    clr = clearance_map(g, RES)
    rng = np.random.default_rng(0)
    for _ in range(40):
        goal, why = pick_goal(g, META, (1.0, 1.0), rng=rng, clearance_m=0.5)
        assert goal is not None, why
        r, c = int(goal['y'] / RES), int(goal['x'] / RES)
        assert g[r, c] == 0
        assert clr[r, c] >= 0.5 - 1e-6


def test_never_picks_the_sealed_cell():
    rng = np.random.default_rng(1)
    for _ in range(80):
        goal, _ = pick_goal(room(), META, (1.0, 1.0), rng=rng, clearance_m=0.3)
        assert not (5.0 <= goal['x'] <= 5.9 and 1.0 <= goal['y'] <= 2.0)


def test_stays_on_its_own_side_of_a_gap_it_cannot_drive_through():
    rng = np.random.default_rng(1)
    for start, side in (((1.0, 2.0), lambda x: x < 3.0), ((5.0, 2.0), lambda x: x > 3.2)):
        for _ in range(40):
            goal, why = pick_goal(split_room(), META, start, rng=rng)
            assert goal is not None, why
            assert side(goal['x'])


def test_spreads_out_and_avoids_recent_spots():
    g = room()
    rng = np.random.default_rng(2)
    pose, recent, seen = (1.0, 1.0), [], set()
    for _ in range(25):
        goal, why = pick_goal(g, META, pose, recent, rng)
        assert goal is not None, why
        for rx, ry in recent:
            assert math.hypot(goal['x'] - rx, goal['y'] - ry) >= 1.0 - 1e-6
        seen.add((int(goal['x']), int(goal['y'])))
        recent = (recent + [(goal['x'], goal['y'])])[-6:]
        pose = (goal['x'], goal['y'])
    assert len(seen) >= 8, 'it should cover the room, not shuffle in a corner'


def test_a_small_floor_never_runs_dry_just_because_it_was_visited():
    # A 2.2 x 2.2 m clear patch: six remembered spots rule out all of it.
    g = np.full((60, 60), 100, np.int16)
    g[5:55, 5:55] = 0
    rng = np.random.default_rng(4)
    pose, recent = (1.5, 1.5), []
    for _ in range(40):
        goal, why = pick_goal(g, META, pose, recent, rng, clearance_m=0.3)
        assert goal is not None, why
        recent = (recent + [(goal['x'], goal['y'])])[-6:]
        pose = (goal['x'], goal['y'])


def test_yaw_faces_the_direction_of_travel():
    goal, _ = pick_goal(room(), META, (1.0, 1.0), rng=np.random.default_rng(3))
    want = math.degrees(math.atan2(goal['y'] - 1.0, goal['x'] - 1.0))
    assert abs(((goal['yaw'] - want + 180) % 360) - 180) < 0.2


@pytest.mark.parametrize('pose,fragment', [
    ((50.0, 50.0), 'outside the map'),          # off the grid
    ((0.025, 0.025), 'inside a wall'),          # a pose in the wall is a pose fault
])
def test_refuses_a_bad_start(pose, fragment):
    goal, why = pick_goal(room(), META, pose)
    assert goal is None and fragment in why


def test_refuses_an_unsurveyed_map():
    goal, why = pick_goal(np.full((40, 40), -1, np.int16), META, (1.0, 1.0))
    assert goal is None and 'clear enough' in why


def test_small_room_relaxes_the_minimum_distance():
    g = np.full((30, 30), 100, np.int16)        # 1.5 m square
    g[5:25, 5:25] = 0
    goal, why = pick_goal(g, META, (0.5, 0.5), clearance_m=0.2, min_dist=2.0)
    assert goal is not None, why
