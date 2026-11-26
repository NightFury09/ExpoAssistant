#!/usr/bin/env python3
"""/front_cam/image_raw -> /front_cam/scan, a virtual laser scan of the floor.

See ground_vision.py for the method and its limits. This node adds the rules
that make it safe to run beside the real sensors:

  * It publishes nothing when it does not trust the picture (covered lens, a
    seed strip that is not one floor, too little floor in view). A silent scan
    changes nothing in the costmap; a wrong one would mark or clear things.
  * The scan is in its own frame, `front_cam_scan`: the camera's position with
    its heading and no tilt, so Nav2 can treat it like a lidar plane.
  * Ranges beyond max_range, and columns that are floor all the way, are +inf
    ("clear to max range"). The costmap layer must set inf_is_valid for that.
  * /front_cam/status says what it is doing, for the console and for tools.

The mount parameters (x, y, z, pitch_deg, yaw_deg) must match the TF that
front_cam.launch.py publishes; both read config/front_cam.yaml.
"""
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String

from rover_core.ground_vision import (
    Camera, Mount, VisionParams, floor_visibility, horizon_row, scan_from_floor,
    segment_floor)


class FrontCamScan(Node):
    def __init__(self):
        super().__init__('front_cam_scan')
        d = self.declare_parameter
        self.mount = Mount(float(d('x', 0.0).value), float(d('y', 0.0).value),
                           float(d('z', 0.25).value),
                           float(d('pitch_deg', 0.0).value),
                           float(d('yaw_deg', 0.0).value))
        self.scan_frame = d('scan_frame', 'front_cam_scan').value
        self.bins = int(d('bins', 90).value)
        self.min_range = float(d('min_range', 0.25).value)
        self.max_range = float(d('max_range', 2.5).value)
        self.max_hz = float(d('max_hz', 10.0).value)
        self.min_confidence = float(d('min_confidence', 0.5).value)
        self.calibrated_hint = bool(d('calibrated', False).value)
        vp = VisionParams()
        for name in ('work_width', 'seed_rows', 'gate_sigmas', 'light_slack',
                     'blur_px', 'close_rows', 'min_width_px', 'min_run_px',
                     'min_floor_fraction'):
            default = getattr(vp, name)
            setattr(vp, name, type(default)(d(name, default).value))
        self.vp = vp

        self.cam = None
        self.create_subscription(CameraInfo, 'front_cam/camera_info', self._on_info, 1)
        self.create_subscription(Image, 'front_cam/image_raw', self._on_image,
                                 qos_profile_sensor_data)
        self.pub_scan = self.create_publisher(LaserScan, 'front_cam/scan',
                                              qos_profile_sensor_data)
        self.pub_status = self.create_publisher(String, 'front_cam/status', 1)
        self.pub_debug = self.create_publisher(Image, 'front_cam/debug', 1)
        self._last = 0.0
        self._n, self._t0, self._fps = 0, time.monotonic(), 0.0
        self._sight = None
        self._warned_mount = False

    def _on_info(self, m):
        if self.cam is None or (self.cam.width, self.cam.height) != (m.width, m.height):
            dist = tuple(list(m.d) + [0.0] * 5)[:5] if m.d else (0.0,) * 5
            self.cam = Camera(m.width, m.height, m.k[0], m.k[4], m.k[2], m.k[5], dist)
            self.nominal = (m.distortion_model == 'nominal')
            self._sight = floor_visibility(self.cam, self.mount)
            self._check_mount()

    def _check_mount(self):
        """Say it loudly if this spot cannot see the floor ahead."""
        s = self._sight
        near = s['nearest_m']
        if near is None or near > 1.0 or s['blocked_fraction'] > 0.5:
            self.get_logger().error(
                'MOUNT: from x=%.2f z=%.2f pitch=%.0f deg the camera mostly sees the '
                'rover\'s own deck (%.0f%% of vertical rays blocked); nearest visible '
                'floor is %s ahead of the front edge. The scan will be empty or '
                'far-field only. Raise it, move it forward, or tilt it down: '
                'tools/camera_mount_planner.py.'
                % (self.mount.x, self.mount.z, self.mount.pitch_deg,
                   100 * s['blocked_fraction'],
                   'never' if near is None else '%.2f m' % near))

    def _on_image(self, msg):
        if self.cam is None:
            return
        now = time.monotonic()
        if now - self._last < 1.0 / self.max_hz:
            return
        self._last = now
        if msg.encoding != 'bgr8':
            self._status(False, 0.0, 'unsupported encoding %s' % msg.encoding)
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        t0 = time.monotonic()
        res = segment_floor(img, self.vp, horizon_row(self.cam, self.mount))
        trusted = res.confidence >= self.min_confidence and not res.reason
        hits, nearest = 0, None
        if trusted:
            a0, inc, ranges = scan_from_floor(res, self.cam, self.mount, self.vp,
                                              self.bins, self.min_range, self.max_range)
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = msg.header.stamp, self.scan_frame
            scan.angle_min, scan.angle_increment = float(a0), float(inc)
            scan.angle_max = float(a0 + inc * (self.bins - 1))
            scan.range_min, scan.range_max = self.min_range, self.max_range
            scan.time_increment = scan.scan_time = 0.0
            scan.ranges = [float(r) for r in ranges]
            self.pub_scan.publish(scan)
            fin = ranges[np.isfinite(ranges)]
            hits = int(fin.size)
            nearest = float(fin.min()) if hits else None
        self._n += 1
        if now - self._t0 >= 1.0:
            self._fps, self._n, self._t0 = self._n / (now - self._t0), 0, now
        self._status(trusted, res.confidence, res.reason, hits, nearest,
                     (time.monotonic() - t0) * 1000.0, res.seed)
        if self.pub_debug.get_subscription_count():
            self._debug(img, res, msg.header)

    def _status(self, ok, conf, reason, hits=0, nearest=None, ms=0.0, seed=()):
        s = String()
        s.data = json.dumps({
            'ok': bool(ok), 'confidence': round(float(conf), 2), 'reason': reason,
            'hits': hits, 'nearest_m': None if nearest is None else round(nearest, 2),
            'fps': round(self._fps, 1), 'ms': round(ms, 1),
            'calibrated': bool(self.cam is not None and not getattr(self, 'nominal', True)),
            'floor_seed_lab': [round(float(v), 0) for v in seed],
            'sees_floor_from': None if self._sight is None else self._sight['nearest_m']})
        self.pub_status.publish(s)

    def _debug(self, img, res, header):
        h, w = res.mask.shape
        small = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
        over = small.copy()
        over[res.mask] = (0.6 * over[res.mask] + 0.4 * np.array([0, 255, 0])).astype(np.uint8)
        for c, r in enumerate(res.contact_rows):
            if r >= 0:
                cv2.circle(over, (c, int(r)), 1, (0, 0, 255), -1)
        out = Image()
        out.header = header
        out.height, out.width, out.encoding, out.step = h, w, 'bgr8', w * 3
        out.data = np.ascontiguousarray(over).tobytes()
        self.pub_debug.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FrontCamScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
