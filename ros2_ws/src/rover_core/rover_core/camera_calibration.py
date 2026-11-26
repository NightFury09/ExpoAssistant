"""Calibration and mount checks for the front webcam. No ROS.

tools/calibrate_front_cam.py and tools/check_front_cam_mount.py are thin command
lines over this, so the maths can be tested on synthetic images.

Two separate jobs, and both matter for "how far away is that":
  * intrinsics (focal length, centre, lens distortion): a printed chessboard and
    cv2.calibrateCamera. Tells the camera how pixels map to angles.
  * extrinsics (height and tilt): measured on the rover. A measuring tape gives
    height to a centimetre; tilt is hard to read off a webcam, so it is solved
    from an object at a known distance.
"""
import math
import re

import cv2
import numpy as np

from rover_core.ground_vision import Camera


# --------------------------------------------------------------------------- #
# Intrinsics
# --------------------------------------------------------------------------- #
def find_corners(gray, cols, rows):
    """Inner-corner grid of a chessboard, or None."""
    ok, corners = cv2.findChessboardCorners(
        gray, (cols, rows),
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4)
    return cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1), crit)


def calibrate(images, cols, rows, square_m):
    """images: list of BGR arrays of a (cols x rows)-inner-corner chessboard.

    Returns (Camera, rms_px, used). Raises ValueError with a reason when the
    result would not be trustworthy (too few views, or a poor fit)."""
    obj = np.zeros((rows * cols, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    obj_pts, img_pts, size = [], [], None
    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        size = (gray.shape[1], gray.shape[0])
        c = find_corners(gray, cols, rows)
        if c is not None:
            obj_pts.append(obj)
            img_pts.append(c)
    if len(obj_pts) < 8:
        raise ValueError('only %d of %d images show the whole board; need at least 8, '
                         'at different distances and tilts' % (len(obj_pts), len(images)))
    rms, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
    if rms > 1.0:
        raise ValueError('reprojection error %.2f px is too high (board not flat, '
                         'blurred, or the lens moved); retake the photos' % rms)
    d = [float(v) for v in dist.ravel()[:5]] + [0.0] * (5 - min(5, dist.size))
    cam = Camera(size[0], size[1], float(K[0, 0]), float(K[1, 1]),
                 float(K[0, 2]), float(K[1, 2]), tuple(d[:5]))
    return cam, float(rms), len(obj_pts)


def write_intrinsics(yaml_text, cam):
    """Put calibrated values into config/front_cam.yaml's front_cam section,
    keeping every comment. Returns the new text."""
    def sub(key, value, text):
        pat = re.compile(r'^(\s+%s:\s*)\[?[^\n#]*' % key, re.M)
        if not pat.search(text):
            raise ValueError('no "%s:" line to update in the yaml' % key)
        return pat.sub(lambda m: m.group(1) + value, text, count=1)
    t = yaml_text
    t = sub('fx', '%.3f' % cam.fx, t)
    t = sub('fy', '%.3f' % cam.fy, t)
    t = sub('cx', '%.3f' % cam.cx, t)
    t = sub('cy', '%.3f' % cam.cy, t)
    t = sub('dist', '[' + ', '.join('%.6f' % v for v in cam.dist) + ']', t)
    return t


# --------------------------------------------------------------------------- #
# Extrinsics: tilt (and height) from something at a known distance
# --------------------------------------------------------------------------- #
def depression_deg(cam, v_px):
    """Angle BELOW the camera's optical axis for image row v (straight ahead)."""
    n = cam.normalised(np.array([[cam.cx, float(v_px)]]))[0]
    return math.degrees(math.atan(n[1]))


def solve_pitch(cam, z, contacts):
    """Nose-down pitch (deg) that makes each contact row land at its known
    distance, with the lens height z fixed. contacts: [(row_px, distance_m)]
    where distance_m is the floor distance FROM THE CAMERA to the object's foot.
    Returns the mean over the measurements and their spread."""
    out = []
    for v, d in contacts:
        if d <= 0:
            continue
        below_horizon = math.degrees(math.atan2(z, d))      # angle of the foot
        out.append(below_horizon - depression_deg(cam, v))  # minus off-axis part
    if not out:
        raise ValueError('no usable measurement')
    return float(np.mean(out)), float(np.ptp(out)) if len(out) > 1 else 0.0


def solve_pitch_and_height(cam, contacts):
    """Two or more contacts at different distances: solve pitch AND lens height.

    foot angle below horizon: atan(z/d) = pitch + off_axis(v). Linearised by a
    small search; returns (pitch_deg, z_m, rms_m)."""
    if len(contacts) < 2:
        raise ValueError('need at least two distances to solve height and tilt')
    best = None
    for z in np.arange(0.05, 1.5, 0.002):
        pitches = [math.degrees(math.atan2(z, d)) - depression_deg(cam, v)
                   for v, d in contacts]
        spread = float(np.ptp(pitches))
        if best is None or spread < best[0]:
            best = (spread, z, float(np.mean(pitches)))
    _, z, p = best
    err = [z / math.tan(math.radians(p + depression_deg(cam, v))) - d for v, d in contacts]
    return p, float(z), float(math.sqrt(np.mean(np.square(err))))
