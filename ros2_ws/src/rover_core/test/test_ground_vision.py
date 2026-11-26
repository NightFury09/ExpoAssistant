"""Floor-boundary vision, on synthetic scenes. No ROS, no camera.

    python3 -m pytest ros2_ws/src/rover_core/test/test_ground_vision.py
"""
import math

import numpy as np
import pytest

from rover_core.ground_vision import (
    Camera, Mount, VisionParams, floor_visibility, horizon_row,
    pixel_to_ground, point_to_pixel, scan_from_floor, segment_floor)
from rover_core.synthetic_scene import render_scene

CAM = Camera.nominal(640, 480, 60.0)
# Front-edge mount: 20 cm up, tilted 20 deg at the floor.
MOUNT = Mount(x=0.20, y=0.0, z=0.22, pitch_deg=22.0)
LEG = (35, 35, 45)              # dark chair leg, BGR


def _scan(img, mount=MOUNT, **kw):
    vp = VisionParams()
    res = segment_floor(img, vp, horizon_row(CAM, mount))
    a0, inc, r = scan_from_floor(res, CAM, mount, vp, **kw)
    return res, a0, inc, r


def _bearing_bin(a0, inc, bearing):
    return int((bearing - a0) / inc)


# ---------------- projection ----------------
def test_ground_projection_round_trips():
    pts = np.array([[0.8, 0.0, 0.0], [1.2, 0.3, 0.0], [0.6, -0.25, 0.0]])
    px = point_to_pixel(CAM, MOUNT, pts)
    back = pixel_to_ground(CAM, MOUNT, px)
    assert np.allclose(back, pts[:, :2], atol=1e-6)


def test_round_trip_with_lens_distortion_and_yaw():
    cam = Camera(640, 480, 600, 600, 322, 238, (-0.12, 0.03, 0.0005, -0.0003, 0.0))
    m = Mount(x=0.1, y=0.05, z=0.3, pitch_deg=15.0, yaw_deg=10.0)
    pts = np.array([[1.0, 0.1, 0.0], [1.5, -0.2, 0.0], [0.9, 0.4, 0.0]])
    back = pixel_to_ground(cam, m, point_to_pixel(cam, m, pts))
    assert np.allclose(back, pts[:, :2], atol=2e-3)


def test_rays_above_the_horizon_have_no_floor_point():
    row = horizon_row(CAM, MOUNT)
    px = np.array([[320.0, row - 5.0], [320.0, row + 5.0]])
    g = pixel_to_ground(CAM, MOUNT, px)
    assert np.isnan(g[0]).all() and not np.isnan(g[1]).any()


def test_straight_ahead_distance_matches_trigonometry():
    # pitch 22 deg, centre pixel looks 22 deg down from 0.22 m: floor at z/tan(pitch)
    g = pixel_to_ground(CAM, MOUNT, np.array([[320.0, 240.0]]))
    assert g[0, 0] == pytest.approx(0.20 + 0.22 / math.tan(math.radians(22.0)), abs=1e-6)


# ---------------- can the camera see the floor from here? ----------------
def test_a_camera_at_the_deck_top_at_the_rear_cannot_see_the_floor_ahead():
    # "Under the lidar": 4.5 cm above the deck, 37 cm behind the front edge.
    m = Mount(x=-0.155, z=0.25, pitch_deg=10.0)
    v = floor_visibility(CAM, m)
    assert v['nearest_m'] is None or v['nearest_m'] > 1.5
    assert v['blocked_fraction'] > 0.5


def test_a_camera_at_the_front_edge_sees_the_floor_close_in():
    v = floor_visibility(CAM, Mount(x=0.25, z=0.25, pitch_deg=22.0))
    assert v['nearest_m'] is not None and v['nearest_m'] < 0.4
    assert v['blocked_fraction'] == 0


def test_raising_the_rear_camera_uncovers_the_floor():
    low = floor_visibility(CAM, Mount(x=-0.155, z=0.26, pitch_deg=25.0))
    high = floor_visibility(CAM, Mount(x=-0.155, z=0.55, pitch_deg=25.0))
    assert high['blocked_fraction'] < low['blocked_fraction']
    assert high['nearest_m'] is not None and high['nearest_m'] < 1.0


# ---------------- segmentation and scan ----------------
@pytest.mark.parametrize('floor', ['plain', 'tile', 'wood'])
def test_an_empty_floor_reads_clear(floor):
    img = render_scene(CAM, MOUNT, [], floor=floor, seed=1)
    res, _, _, r = _scan(img)
    assert res.confidence > 0.5, res.reason
    assert np.isinf(r).all(), 'false obstacle on an empty %s floor: %s' % (floor, r[np.isfinite(r)])


@pytest.mark.parametrize('dist', [0.6, 0.9, 1.3, 1.8])
def test_a_chair_leg_is_found_at_the_right_distance(dist):
    img = render_scene(CAM, MOUNT, [(MOUNT.x + dist, 0.0, 0.035, 0.45, LEG)],
                       floor='tile', seed=2)
    _, a0, inc, r = _scan(img)
    found = r[np.isfinite(r)]
    assert found.size, 'a 3.5 cm leg at %.1f m was not seen' % dist
    # contact is one pixel of floor-distance wide; allow 10% or 6 cm
    assert abs(found.min() - dist) <= max(0.06, 0.10 * dist), (found.min(), dist)
    assert abs(math.degrees(a0 + inc * (np.argmin(r) + 0.5))) < 6.0, 'wrong bearing'


def test_two_legs_keep_their_own_bearings():
    posts = [(MOUNT.x + 1.0, 0.30, 0.04, 0.45, LEG), (MOUNT.x + 1.0, -0.30, 0.04, 0.45, LEG)]
    _, a0, inc, r = _scan(render_scene(CAM, MOUNT, posts, floor='plain', seed=3))
    hit = np.nonzero(np.isfinite(r))[0]
    bearings = sorted(math.degrees(a0 + inc * (b + 0.5)) for b in hit)
    want = math.degrees(math.atan2(0.30, 1.0))
    assert bearings[0] == pytest.approx(-want, abs=4.0)
    assert bearings[-1] == pytest.approx(want, abs=4.0)
    assert np.isinf(r[len(r) // 2]), 'the gap between the legs must read clear'


def test_the_nearer_of_two_obstacles_wins_a_bearing():
    posts = [(MOUNT.x + 1.8, 0.0, 0.05, 0.5, LEG), (MOUNT.x + 0.8, 0.0, 0.05, 0.5, LEG)]
    _, _, _, r = _scan(render_scene(CAM, MOUNT, posts, floor='plain', seed=4))
    # The near face of a 5 cm post is 2.5 cm closer than its centre line.
    assert np.nanmin(r[np.isfinite(r)]) == pytest.approx(0.8 - 0.025, abs=0.08)


def test_far_things_are_left_clear():
    img = render_scene(CAM, MOUNT, [(MOUNT.x + 3.5, 0.0, 0.2, 0.6, LEG)], seed=5)
    _, _, _, r = _scan(img, max_range=2.5)
    assert np.isinf(r).all()


def test_a_dark_frame_is_not_trusted():
    for level in (0, 255):                           # lens covered / blown out
        res, _, _, r = _scan(np.full((480, 640, 3), level, np.uint8))
        assert res.confidence == 0.0 and 'dark or too bright' in res.reason
        assert np.isinf(r).all()


def test_a_seed_strip_that_is_not_one_colour_is_refused():
    img = render_scene(CAM, MOUNT, [], floor='plain', seed=6)
    blocks = np.random.default_rng(0).integers(0, 255, (3, 16, 3), np.uint8)
    img[-60:, :, :] = np.repeat(np.repeat(blocks, 20, axis=0), 40, axis=1)
    res, _, _, r = _scan(img)
    assert res.confidence == 0.0 and 'one colour' in res.reason
    assert np.isinf(r).all()


def test_scan_angles_follow_the_camera_yaw():
    m = Mount(x=0.2, z=0.22, pitch_deg=22.0, yaw_deg=20.0)
    img = render_scene(CAM, m, [(0.2 + 1.0 * math.cos(math.radians(20)),
                                 1.0 * math.sin(math.radians(20)), 0.04, 0.45, LEG)],
                       floor='plain', seed=7)
    vp = VisionParams()
    res = segment_floor(img, vp, horizon_row(CAM, m))
    a0, inc, r = scan_from_floor(res, CAM, m, vp)
    assert abs(math.degrees(a0 + inc * (np.argmin(r) + 0.5))) < 6.0, \
        'the scan frame is aligned with the camera heading, so dead ahead is ~0'
