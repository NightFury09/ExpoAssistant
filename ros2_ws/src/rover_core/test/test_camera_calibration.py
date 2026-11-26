"""Calibration and mount-solving, on synthetic chessboards and posts. No camera.

    python3 -m pytest ros2_ws/src/rover_core/test/test_camera_calibration.py
"""
import cv2
import numpy as np
import pytest

from rover_core.camera_calibration import (
    calibrate, depression_deg, solve_pitch, solve_pitch_and_height, write_intrinsics)
from rover_core.ground_vision import (
    Camera, Mount, VisionParams, horizon_row, segment_floor)
from rover_core.synthetic_scene import render_scene

TRUE = Camera(640, 480, 580.0, 585.0, 322.0, 241.0)
COLS, ROWS, SQ = 9, 6, 0.03


def board_image(rvec, tvec, cam=TRUE):
    """A chessboard seen by `cam` at the given pose."""
    s = 80
    pat = np.zeros(((ROWS + 1) * s, (COLS + 1) * s), np.uint8)
    for r in range(ROWS + 1):
        for c in range(COLS + 1):
            if (r + c) % 2 == 0:
                pat[r * s:(r + 1) * s, c * s:(c + 1) * s] = 255
    pat = cv2.copyMakeBorder(pat, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
    # board plane corners in metres, centred on the inner-corner grid
    w, h = (COLS + 1) * SQ, (ROWS + 1) * SQ
    margin = 40.0 / s * SQ
    corners3d = np.array([[-SQ - margin, -SQ - margin, 0], [w - SQ + margin, -SQ - margin, 0],
                          [w - SQ + margin, h - SQ + margin, 0], [-SQ - margin, h - SQ + margin, 0]],
                         np.float32)
    px, _ = cv2.projectPoints(corners3d, rvec, tvec, cam.matrix, None)
    src = np.array([[0, 0], [pat.shape[1], 0], [pat.shape[1], pat.shape[0]], [0, pat.shape[0]]],
                   np.float32)
    H = cv2.getPerspectiveTransform(src, px.reshape(4, 2).astype(np.float32))
    img = cv2.warpPerspective(pat, H, (cam.width, cam.height), borderValue=128)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def poses():
    rng = np.random.default_rng(3)
    out = []
    for _ in range(14):
        rv = np.array([rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), rng.uniform(-0.3, 0.3)])
        tv = np.array([rng.uniform(-0.1, 0.05), rng.uniform(-0.08, 0.04), rng.uniform(0.35, 0.7)])
        out.append((rv, tv))
    return out


def test_calibration_recovers_the_intrinsics():
    imgs = [board_image(rv, tv) for rv, tv in poses()]
    cam, rms, used = calibrate(imgs, COLS, ROWS, SQ)
    assert used >= 10 and rms < 0.6, (used, rms)
    assert cam.fx == pytest.approx(TRUE.fx, rel=0.02)
    assert cam.fy == pytest.approx(TRUE.fy, rel=0.02)
    assert cam.cx == pytest.approx(TRUE.cx, abs=4.0)
    assert cam.cy == pytest.approx(TRUE.cy, abs=4.0)


def test_calibration_refuses_too_few_views():
    imgs = [board_image(rv, tv) for rv, tv in poses()[:4]]
    with pytest.raises(ValueError, match='at least 8'):
        calibrate(imgs, COLS, ROWS, SQ)


def test_calibration_refuses_images_with_no_board():
    with pytest.raises(ValueError, match='at least 8'):
        calibrate([np.full((480, 640, 3), 90, np.uint8)] * 10, COLS, ROWS, SQ)


YAML = """front_cam:
  ros__parameters:
    hfov_deg: 52.0   # nominal
    fx: 0.0
    fy: 0.0
    cx: 0.0
    cy: 0.0
    dist: [0.0, 0.0, 0.0, 0.0, 0.0]
"""


def test_writing_intrinsics_keeps_the_comments_and_other_lines():
    cam = Camera(640, 480, 581.25, 584.5, 321.5, 240.25, (-0.1, 0.02, 0.0, 0.0, 0.0))
    out = write_intrinsics(YAML, cam)
    assert 'fx: 581.250' in out and 'cy: 240.250' in out
    assert 'dist: [-0.100000, 0.020000' in out
    assert 'hfov_deg: 52.0   # nominal' in out


def test_writing_into_a_file_without_the_keys_is_an_error():
    with pytest.raises(ValueError):
        write_intrinsics('front_cam:\n  ros__parameters:\n    device: x\n', TRUE)


# ---------------- tilt and height from known distances ----------------
CAM = Camera.nominal(640, 480, 52.0)
LEG = (35, 35, 45)


def foot_row(mount, dist):
    """The image row where a post `dist` m from the camera meets the floor."""
    img = render_scene(CAM, mount, [(mount.x + dist, 0.0, 0.05, 0.6, LEG)], 'plain', seed=1)
    res = segment_floor(img, VisionParams(), horizon_row(CAM, mount))
    scale = CAM.height / res.mask.shape[0]
    cols = np.nonzero(res.contact_rows >= 0)[0]
    return float(np.median(res.contact_rows[cols]) * scale)


def test_depression_is_zero_on_the_axis_and_grows_downward():
    assert depression_deg(CAM, CAM.cy) == pytest.approx(0.0, abs=1e-9)
    assert depression_deg(CAM, CAM.cy + 100) > 0 > depression_deg(CAM, CAM.cy - 100)


def test_pitch_is_recovered_from_one_known_distance():
    true = Mount(x=0.25, z=0.25, pitch_deg=22.0)
    d = 1.2
    pitch, _ = solve_pitch(CAM, true.z, [(foot_row(true, d), d - 0.025)])
    assert pitch == pytest.approx(22.0, abs=1.5)


def test_pitch_and_height_from_two_distances():
    true = Mount(x=0.25, z=0.28, pitch_deg=18.0)
    pts = [(foot_row(true, d), d - 0.025) for d in (0.7, 1.6)]
    pitch, z, rms = solve_pitch_and_height(CAM, pts)
    assert pitch == pytest.approx(18.0, abs=2.0)
    assert z == pytest.approx(0.28, abs=0.04)
    assert rms < 0.08


def test_a_wrong_assumed_pitch_is_caught_by_the_known_distance():
    # The camera really is at 22 degrees; the config says 18. A post at a known
    # 1.2 m is then reported at the wrong range, and solve_pitch finds the 22.
    true, assumed = Mount(x=0.25, z=0.25, pitch_deg=22.0), 18.0
    row = foot_row(true, 1.2)
    got, _ = solve_pitch(CAM, 0.25, [(row, 1.2 - 0.025)])
    assert abs(got - assumed) > 2.0 and abs(got - 22.0) < 1.5
