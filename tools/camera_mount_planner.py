#!/usr/bin/env python3
"""Where can the front webcam go, and what will it actually see of the floor?

Run this BEFORE drilling. A camera that sits level with the rover's own deck sees
the deck, not the floor in front of it. For each mount this prints how much of
the camera's view is blocked by the rover's body and how close to the front edge
the first visible floor is, using the box sizes in rover.urdf.

    python3 tools/camera_mount_planner.py                       # a comparison table
    python3 tools/camera_mount_planner.py --x -0.155 --z 0.25 --pitch 20
    python3 tools/camera_mount_planner.py --hfov 52 --vfov 40   # your camera

It looks along the centre plane only and treats the deck as one box. It is a
screen for bad mounts, not a render: measure the real rover.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                                'ros2_ws', 'src', 'rover_core'))
from rover_core.ground_vision import Camera, Mount, floor_visibility  # noqa: E402

# Logitech C270: the spec is 60 deg DIAGONAL. In a 4:3 mode that is about 49 x 38
# degrees if the whole sensor is used. Calibrate to know yours.
DEFAULT_HFOV, DEFAULT_VFOV = 52.0, 40.0
GOOD_NEAREST_M = 0.8      # floor visible this close to the bumper or nearer: useful
FRONT_EDGE = 0.215


def camera(hfov, vfov, w=640, h=480):
    fx = (w / 2) / math.tan(math.radians(hfov) / 2)
    fy = (h / 2) / math.tan(math.radians(vfov) / 2)
    return Camera(w, h, fx, fy, w / 2, h / 2)


def verdict(v):
    near = v['nearest_m']
    if near is None:
        return 'BLIND to the floor: sees only the rover or the sky'
    if v['blocked_fraction'] > 0.5:
        return 'MOSTLY BLOCKED by the rover (%.0f%%)' % (100 * v['blocked_fraction'])
    if near > GOOD_NEAREST_M:
        return 'too far: first floor %.2f m past the bumper' % near
    return 'ok: floor from %.2f m past the bumper' % max(near, 0.0)


def row(label, cam, m):
    v = floor_visibility(cam, m, front_edge=FRONT_EDGE)
    near = 'never' if v['nearest_m'] is None else '%5.2f m' % v['nearest_m']
    print('%-34s x=%5.3f z=%4.2f pitch=%3.0f   blocked %3.0f%%   nearest floor %-8s %s' % (
        label, m.x, m.z, m.pitch_deg, 100 * v['blocked_fraction'], near, verdict(v)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--x', type=float)
    ap.add_argument('--z', type=float)
    ap.add_argument('--pitch', type=float, default=20.0, help='nose-down degrees')
    ap.add_argument('--hfov', type=float, default=DEFAULT_HFOV)
    ap.add_argument('--vfov', type=float, default=DEFAULT_VFOV)
    a = ap.parse_args()
    cam = camera(a.hfov, a.vfov)
    print('camera %.0f x %.0f deg   rover deck top 0.23 m, front edge x=%.3f, lidar plane 0.275 m (URDF)\n'
          % (a.hfov, a.vfov, FRONT_EDGE))
    if a.x is not None and a.z is not None:
        row('your mount', cam, Mount(a.x, 0.0, a.z, a.pitch))
        return
    print('-- under the lidar (x = -0.155), the spot you asked about')
    for z, p in ((0.25, 0), (0.25, 20), (0.25, 35), (0.27, 20)):
        row('under the lidar', cam, Mount(-0.155, 0, z, p))
    print('\n-- same spot, raised on a short mast')
    for z, p in ((0.35, 25), (0.45, 25), (0.55, 25)):
        row('under the lidar, raised', cam, Mount(-0.155, 0, z, p))
    print('\n-- on the front of the deck')
    for x, z, p in ((0.22, 0.25, 15), (0.22, 0.25, 25), (0.22, 0.30, 25)):
        row('front edge', cam, Mount(x, 0, z, p))
    print('\nThe floor is only visible where the camera looks over the deck. A lens '
          'a few centimetres above the deck top sees the deck for every ray that '
          'points down, and the floor only far ahead.')


if __name__ == '__main__':
    main()
