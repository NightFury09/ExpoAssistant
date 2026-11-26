#!/usr/bin/env python3
"""End to end, no rover and no camera: synthetic frames -> front_cam_scan -> scan.

Runs the REAL front_cam_scan node as a subprocess, publishes rendered camera
frames and a camera_info at it, and checks the LaserScan that comes back:

  * a chair leg at a known distance shows up at that distance and bearing;
  * an empty tiled floor gives a scan that is all +inf ("clear");
  * a covered lens gives NO scan at all (a silent camera must not mark or clear);
  * /front_cam/status says what happened.

    ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 python3 tools/test_front_cam_pipeline.py

Uses an unused domain id so it cannot talk to a running rover.
"""
import json
import math
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..',
                                'ros2_ws', 'src', 'rover_core'))
from rover_core.ground_vision import Camera, Mount  # noqa: E402
from rover_core.synthetic_scene import render_scene  # noqa: E402

CAM = Camera.nominal(640, 480, 60.0)
MOUNT = Mount(x=0.25, y=0.0, z=0.25, pitch_deg=22.0)
LEG = (35, 35, 45)
PASS, FAIL = [], []


def check(name, ok, detail=''):
    (PASS if ok else FAIL).append(name)
    print(('PASS  ' if ok else 'FAIL  ') + name + (('  -- ' + detail) if detail else ''))


class Rig(Node):
    def __init__(self):
        super().__init__('front_cam_test_rig')
        self.pub_img = self.create_publisher(Image, 'front_cam/image_raw', qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, 'front_cam/camera_info', 1)
        self.scans, self.status = [], None
        self.create_subscription(LaserScan, 'front_cam/scan', self.scans.append, qos_profile_sensor_data)
        self.create_subscription(String, 'front_cam/status',
                                 lambda m: setattr(self, 'status', json.loads(m.data)), 1)

    def info(self):
        m = CameraInfo()
        m.width, m.height = CAM.width, CAM.height
        m.distortion_model = 'nominal'
        m.d = [0.0] * 5
        m.k = [CAM.fx, 0.0, CAM.cx, 0.0, CAM.fy, CAM.cy, 0.0, 0.0, 1.0]
        return m

    def show(self, frame, seconds=2.0):
        """Publish `frame` repeatedly and return only the scans made FROM it.

        A scan carries its image's timestamp, so scans stamped before this call
        belong to the previous picture, still in flight when the picture
        changed; they are not counted against this one."""
        t_switch = self.get_clock().now().nanoseconds
        end = time.time() + seconds
        while time.time() < end:
            m = Image()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'front_cam_optical_frame'
            m.height, m.width, m.encoding, m.step = frame.shape[0], frame.shape[1], 'bgr8', frame.shape[1] * 3
            m.data = np.ascontiguousarray(frame).tobytes()
            self.pub_img.publish(m)
            self.pub_info.publish(self.info())
            rclpy.spin_once(self, timeout_sec=0.05)
        for _ in range(10):                       # let the node's backlog drain
            rclpy.spin_once(self, timeout_sec=0.1)
        return [s for s in self.scans
                if s.header.stamp.sec * 10**9 + s.header.stamp.nanosec >= t_switch]


def main():
    env = dict(os.environ)
    env.setdefault('ROS_DOMAIN_ID', '77')
    env['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.update(env)
    pkg = os.path.join(os.path.dirname(__file__), '..', 'ros2_ws', 'src', 'rover_core')
    node_env = dict(env, PYTHONPATH=os.path.abspath(pkg) + os.pathsep + env.get('PYTHONPATH', ''))
    proc = subprocess.Popen(
        [sys.executable, '-m', 'rover_core.front_cam_scan', '--ros-args',
         '-p', 'x:=%f' % MOUNT.x, '-p', 'z:=%f' % MOUNT.z, '-p', 'pitch_deg:=%f' % MOUNT.pitch_deg,
         '-p', 'max_hz:=20.0'],
        env=node_env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)
    rclpy.init()
    rig = Rig()
    try:
        time.sleep(1.0)

        leg_d = 1.1
        frame = render_scene(CAM, MOUNT, [(MOUNT.x + leg_d, 0.20, 0.04, 0.45, LEG)], 'tile', seed=1)
        scans = rig.show(frame)
        check('the node publishes scans for good frames', len(scans) >= 3, '%d scans' % len(scans))
        if scans:
            s = scans[-1]
            r = np.array(s.ranges)
            fin = np.nonzero(np.isfinite(r))[0]
            check('scan is in the gravity-aligned camera frame', s.header.frame_id == 'front_cam_scan')
            check('one cluster of hits for one leg', len(fin) >= 1 and fin[-1] - fin[0] <= 4,
                  'bins %s' % fin.tolist())
            if len(fin):
                want = math.hypot(leg_d, 0.20)
                got = float(r[fin].min())
                check('leg distance within 10 percent', abs(got - want) <= 0.10 * want,
                      'got %.2f want %.2f' % (got, want))
                bearing = s.angle_min + s.angle_increment * (fin.mean() + 0.5)
                check('leg bearing within 5 degrees',
                      abs(math.degrees(bearing) - math.degrees(math.atan2(0.20, leg_d))) < 5.0,
                      '%.1f deg' % math.degrees(bearing))
            check('range_max is the configured max range', abs(s.range_max - 2.5) < 1e-6)
        check('status reports ok with hits', bool(rig.status and rig.status['ok'] and rig.status['hits'] >= 1),
              str(rig.status))

        scans = rig.show(render_scene(CAM, MOUNT, [], 'tile', seed=2))
        check('an empty floor is all clear (+inf)',
              bool(scans) and all(np.isinf(np.array(s.ranges)).all() for s in scans))

        scans = rig.show(np.zeros((480, 640, 3), np.uint8))
        check('a covered lens publishes no scan', len(scans) == 0, '%d scans' % len(scans))
        check('status says why', bool(rig.status) and not rig.status['ok'] and
              'dark' in rig.status['reason'], str(rig.status))

        scans = rig.show(frame)
        check('and it recovers when the picture does', len(scans) >= 3)
    finally:
        rig.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(proc.pid, 15)
        except ProcessLookupError:
            pass
        proc.wait(timeout=10)

    print('\n%d passed, %d failed' % (len(PASS), len(FAIL)))
    return 1 if FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
