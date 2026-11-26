#!/usr/bin/env python3
"""Check the front camera's mount numbers against an object at a known distance.

A distance read from a floor camera is only as good as the camera's height and
tilt in config/front_cam.yaml. Height you can measure with a tape; tilt you
cannot read off a webcam. This solves it from something you place.

    # Stand a thin tall object (a pole, a bottle, a chair leg) on the floor, dead
    # ahead, and measure from the rover's FRONT EDGE to its near face:
    python3 tools/check_front_cam_mount.py --live --distance 1.20
    # ...or from saved photos, one or more (two distances also solve the height):
    python3 tools/check_front_cam_mount.py --measure near.png:0.80 --measure far.png:1.60

It reports where the pipeline puts the object now, what tilt would put it where
you measured it, and a line to paste into the yaml. The camera must be free for
--live. Nothing is written for you.

    python3 tools/check_front_cam_mount.py --demo     # no camera: a synthetic check
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'rover_core'))
from rover_core.camera_calibration import solve_pitch, solve_pitch_and_height  # noqa: E402
from rover_core.ground_vision import (  # noqa: E402
    Camera, Mount, VisionParams, horizon_row, segment_floor)

CONFIG = os.path.join(HERE, '..', 'ros2_ws', 'src', 'my_robot_bringup', 'config',
                      'front_cam.yaml')
FRONT_EDGE = 0.215


def load_config(path=CONFIG):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    c = cfg['front_cam']['ros__parameters']
    s = cfg['front_cam_scan']['ros__parameters']
    w, h = int(c['width']), int(c['height'])
    if c['fx'] > 0:
        cam = Camera(w, h, c['fx'], c['fy'], c['cx'], c['cy'], tuple(c['dist']))
        calibrated = True
    else:
        cam, calibrated = Camera.nominal(w, h, c['hfov_deg']), False
    mount = Mount(s['x'], s['y'], s['z'], s['pitch_deg'], s['yaw_deg'])
    return cam, mount, calibrated


def contact_row(img, cam, mount, half_width_deg=20.0):
    """Median image row where the nearest obstacle in the central columns meets
    the floor, in full-resolution pixels; None if none was found."""
    vp = VisionParams()
    res = segment_floor(img, vp, horizon_row(cam, mount))
    if res.reason:
        raise SystemExit('the picture is not usable: ' + res.reason)
    w = res.mask.shape[1]
    scale = cam.width / float(w)
    half = int(w * math.tan(math.radians(half_width_deg)) /
               (2 * math.tan(math.radians(cam.hfov_deg) / 2)))
    cols = [c for c in range(w // 2 - half, w // 2 + half + 1) if res.contact_rows[c] >= 0]
    if not cols:
        return None, res
    rows = np.array([res.contact_rows[c] for c in cols], float)
    # the cluster that is NEAREST (lowest in the image): ignore stray far hits
    near = rows[rows >= np.percentile(rows, 50)]
    return float(np.median(near) * scale), res


def report(measures, cam, mount, calibrated):
    contacts = []
    for label, row, d_edge in measures:
        d_cam = (FRONT_EDGE - mount.x) + d_edge     # lens to near face, along the floor
        contacts.append((row, d_cam))
        now = scan_range_for_row(cam, mount, row)
        print('%-14s object %.2f m past the front edge (%.2f m from the lens)'
              % (label, d_edge, d_cam))
        print('%-14s pipeline now says %.2f m from the lens: error %+.2f m (%+.0f%%)'
              % ('', now, now - d_cam, 100 * (now - d_cam) / d_cam))
    if not calibrated:
        print('\nNote: the camera is NOT calibrated, so part of any error is the focal '
              'length. Run tools/calibrate_front_cam.py first.')
    pitch, spread = solve_pitch(cam, mount.z, contacts)
    print('\nWith the lens height fixed at z=%.3f m:' % mount.z)
    print('  pitch that matches your measurement: %.1f deg   (config says %.1f)'
          % (pitch, mount.pitch_deg))
    if len(contacts) > 1:
        print('  spread between your measurements:   %.1f deg (small = consistent)' % spread)
        p2, z2, rms = solve_pitch_and_height(cam, contacts)
        print('  solving height too:                  pitch %.1f deg, z %.3f m, fit %.2f m rms'
              % (p2, z2, rms))
        print('  (measure z with a tape; trust this only if it agrees with the tape)')
    if abs(pitch - mount.pitch_deg) < 1.0:
        print('\nThe mount in the yaml agrees with what you measured (within 1 deg). Good.')
    else:
        print('\nSet this in config/front_cam.yaml, front_cam_scan section:\n'
              '    pitch_deg: %.1f' % pitch)


def scan_range_for_row(cam, mount, row):
    from rover_core.ground_vision import pixel_to_ground
    g = pixel_to_ground(cam, mount, np.array([[cam.cx, row + 0.5]]))[0]
    return float('nan') if np.isnan(g[0]) else math.hypot(g[0] - mount.x, g[1] - mount.y)


def grab(device, cam):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit('cannot open %s. Is another program using the camera?' % device)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam.height)
    frame = None
    for _ in range(15):                     # let auto-exposure settle
        ok, f = cap.read()
        if ok:
            frame = f
    cap.release()
    if frame is None:
        sys.exit('the camera gave no frames')
    return frame


def demo():
    from rover_core.synthetic_scene import render_scene
    cam = Camera.nominal(640, 480, 52.0)
    truth = Mount(x=0.25, z=0.25, pitch_deg=22.0)
    assumed = Mount(x=0.25, z=0.25, pitch_deg=17.0)
    measures = []
    for d_edge in (0.9, 1.5):
        d_cam = (FRONT_EDGE - truth.x) + d_edge
        img = render_scene(cam, truth, [(truth.x + d_cam + 0.025, 0.0, 0.05, 0.6, (35, 35, 45))],
                           'tile', seed=1)
        row, _ = contact_row(img, cam, assumed)
        measures.append(('demo %.1f m' % d_edge, row, d_edge))
    print('DEMO: the camera is really tilted 22 deg; the yaml says 17.\n')
    report(measures, cam, assumed, False)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--live', action='store_true')
    ap.add_argument('--distance', type=float, help='metres, front edge to the object (--live)')
    ap.add_argument('--measure', action='append', default=[], metavar='IMAGE:METRES')
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--config', default=CONFIG)
    ap.add_argument('--demo', action='store_true')
    a = ap.parse_args()
    if a.demo:
        return demo()
    cam, mount, calibrated = load_config(a.config)
    measures = []
    if a.live:
        if not a.distance:
            sys.exit('--live needs --distance')
        row, _ = contact_row(grab(a.device, cam), cam, mount)
        if row is None:
            sys.exit('no object found dead ahead; is it in view and darker/lighter than the floor?')
        measures.append(('live', row, a.distance))
    for m in a.measure:
        path, _, d = m.rpartition(':')
        img = cv2.imread(path)
        if img is None:
            sys.exit('cannot read ' + path)
        row, _ = contact_row(img, cam, mount)
        if row is None:
            sys.exit('no object found in ' + path)
        measures.append((os.path.basename(path), row, float(d)))
    if not measures:
        ap.error('give --live --distance D, or --measure IMAGE:D, or --demo')
    report(measures, cam, mount, calibrated)


if __name__ == '__main__':
    main()
