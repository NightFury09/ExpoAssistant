#!/usr/bin/env python3
"""Calibrate the front webcam's lens with a printed chessboard.

Why: distance from a camera pixel depends on the focal length and lens distortion,
and a webcam's real numbers are not the datasheet's. Do this once per camera (and
again if you change resolution).

    # 1. Print a chessboard (9 x 6 inner corners works with an A4 print:
    #    10 x 7 squares). Measure one square with a ruler, in metres.
    # 2. Free the camera (it has one owner), then take ~20 photos of the board at
    #    different distances, tilts and positions in the frame, corners to corners:
    python3 tools/calibrate_front_cam.py capture --out ~/front_cam_calib
    # 3. Solve, and write the result into config/front_cam.yaml:
    python3 tools/calibrate_front_cam.py solve --images ~/front_cam_calib \\
        --square 0.025 --write

`capture` only keeps a photo when the whole board is found, so every saved image
counts. `solve` refuses to write a calibration it does not trust (fewer than 8
usable views, or a reprojection error over 1 px).
"""
import argparse
import glob
import os
import sys
import time

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'rover_core'))
from rover_core.camera_calibration import (  # noqa: E402
    calibrate, find_corners, write_intrinsics)

CONFIG = os.path.join(HERE, '..', 'ros2_ws', 'src', 'my_robot_bringup', 'config',
                      'front_cam.yaml')


def capture(a):
    os.makedirs(a.out, exist_ok=True)
    cap = cv2.VideoCapture(a.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit('cannot open %s. Is another program using the camera? '
                 '(the reception greeter does)' % a.device)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    saved, last = 0, 0.0
    print('Hold the board in view. Move it between photos: near and far, tilted left, '
          'right, up and down, and into each corner of the frame. Ctrl+C to stop.')
    try:
        while saved < a.count:
            ok, frame = cap.read()
            if not ok:
                continue
            if time.time() - last < a.interval:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if find_corners(gray, a.cols, a.rows) is None:
                print('\r  no full board in view...      ', end='', flush=True)
                continue
            last = time.time()
            cv2.imwrite(os.path.join(a.out, 'calib_%02d.png' % saved), frame)
            saved += 1
            print('\r  saved %d / %d                   ' % (saved, a.count), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
    print('\n%d photos in %s. Next: solve.' % (saved, a.out))


def solve(a):
    paths = sorted(glob.glob(os.path.join(os.path.expanduser(a.images), '*.png')) +
                   glob.glob(os.path.join(os.path.expanduser(a.images), '*.jpg')))
    if not paths:
        sys.exit('no images in %s' % a.images)
    imgs = [cv2.imread(p) for p in paths]
    try:
        cam, rms, used = calibrate(imgs, a.cols, a.rows, a.square)
    except ValueError as e:
        sys.exit('NOT calibrated: %s' % e)
    print('used %d of %d images, reprojection error %.3f px' % (used, len(imgs), rms))
    print('fx=%.2f fy=%.2f cx=%.2f cy=%.2f' % (cam.fx, cam.fy, cam.cx, cam.cy))
    print('dist=%s' % [round(v, 5) for v in cam.dist])
    print('field of view: %.1f x %.1f degrees' % (cam.hfov_deg, cam.vfov_deg))
    if a.write:
        with open(CONFIG) as f:
            text = f.read()
        with open(CONFIG, 'w') as f:
            f.write(write_intrinsics(text, cam))
        print('written to %s. Restart front_cam.launch.py to use it.' % os.path.abspath(CONFIG))
    else:
        print('(not written: add --write to put it in config/front_cam.yaml)')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    for name in ('capture', 'solve'):
        p = sub.add_parser(name)
        p.add_argument('--cols', type=int, default=9, help='inner corners across')
        p.add_argument('--rows', type=int, default=6, help='inner corners down')
        if name == 'capture':
            p.add_argument('--out', required=True)
            p.add_argument('--device', default='/dev/video0')
            p.add_argument('--width', type=int, default=640)
            p.add_argument('--height', type=int, default=480)
            p.add_argument('--count', type=int, default=20)
            p.add_argument('--interval', type=float, default=1.5, help='seconds between photos')
        else:
            p.add_argument('--images', required=True)
            p.add_argument('--square', type=float, required=True, help='square size, metres')
            p.add_argument('--write', action='store_true')
    a = ap.parse_args()
    capture(a) if a.cmd == 'capture' else solve(a)


if __name__ == '__main__':
    main()
