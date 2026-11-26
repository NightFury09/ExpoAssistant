#!/usr/bin/env python3
"""The front webcam as a ROS camera: /front_cam/image_raw and /front_cam/camera_info.

Plain OpenCV capture (V4L2), so it needs no usb_cam / v4l2_camera package. A
reader thread keeps only the newest frame, so a slow consumer sees the present
rather than a queue of the past.

The camera is a one-owner device. If another program holds it (the reception
greeter, for one, opens /dev/video0) the open fails; this node keeps retrying and
says so, instead of dying, so it comes up on its own when the camera is freed.

Intrinsics: from parameters (fx, fy, cx, cy, dist) when they are set, which
tools/calibrate_front_cam.py writes into config/front_cam.yaml. Otherwise a
pinhole is assumed from hfov_deg, and camera_info says so through its
`distortion_model` ('nominal') -- distances from an uncalibrated camera are
approximate, and the scan node reports it.
"""
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from rover_core.ground_vision import Camera


def camera_from_params(width, height, fx, fy, cx, cy, dist, hfov_deg):
    """(Camera, calibrated). A zero fx means "not calibrated"."""
    if fx > 0 and fy > 0:
        return Camera(width, height, fx, fy, cx or width / 2.0,
                      cy or height / 2.0, tuple(list(dist) + [0.0] * 5)[:5]), True
    return Camera.nominal(width, height, hfov_deg), False


class FrontCam(Node):
    def __init__(self):
        super().__init__('front_cam')
        d = self.declare_parameter
        self.device = d('device', '/dev/video0').value
        self.width = int(d('width', 640).value)
        self.height = int(d('height', 480).value)
        self.fps = float(d('fps', 15.0).value)
        self.fourcc = str(d('fourcc', 'MJPG').value)
        self.frame_id = d('frame_id', 'front_cam_optical_frame').value
        self.fx = float(d('fx', 0.0).value)
        self.fy = float(d('fy', 0.0).value)
        self.cx = float(d('cx', 0.0).value)
        self.cy = float(d('cy', 0.0).value)
        self.dist = list(d('dist', [0.0, 0.0, 0.0, 0.0, 0.0]).value)
        self.hfov = float(d('hfov_deg', 60.0).value)

        self.cam, self.calibrated = camera_from_params(
            self.width, self.height, self.fx, self.fy, self.cx, self.cy,
            self.dist, self.hfov)
        if not self.calibrated:
            self.get_logger().warn(
                'no calibration set: assuming a %.0f deg pinhole. Distances will be '
                'approximate; run tools/calibrate_front_cam.py.' % self.hfov)

        self.pub_img = self.create_publisher(Image, 'front_cam/image_raw',
                                             qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, 'front_cam/camera_info', 1)
        self.pub_status = self.create_publisher(String, 'front_cam/capture_status', 1)

        self._lock = threading.Lock()
        self._frame, self._frame_t, self._seq = None, 0.0, 0
        self._last_pub_seq = 0
        self._stop = False
        self._opened = False
        self._reason = 'starting'
        self._n_pub, self._t_rate = 0, time.monotonic()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()
        self.create_timer(1.0 / max(1.0, self.fps), self._publish)
        self.create_timer(1.0, self._status)

    # ---- capture thread -------------------------------------------------
    def _open(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        if len(self.fourcc) == 4:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _reader(self):
        cap = None
        while not self._stop:
            if cap is None:
                cap = self._open()
                if cap is None:
                    self._opened = False
                    self._reason = ('cannot open %s: no camera, or another program '
                                    'is using it' % self.device)
                    time.sleep(2.0)
                    continue
            ok, frame = cap.read()
            if not ok or frame is None:
                self._opened = False
                self._reason = 'camera stopped delivering frames'
                cap.release()
                cap = None
                time.sleep(1.0)
                continue
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height))
            self._opened, self._reason = True, ''
            with self._lock:
                self._frame, self._frame_t, self._seq = frame, time.monotonic(), self._seq + 1
        if cap is not None:
            cap.release()

    # ---- ROS side ---------------------------------------------------------
    def _publish(self):
        with self._lock:
            frame, seq, t = self._frame, self._seq, self._frame_t
        if frame is None or seq == self._last_pub_seq:
            return
        if time.monotonic() - t > 1.0:
            return                              # stale: do not republish an old frame
        self._last_pub_seq = seq
        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, self.frame_id
        msg.height, msg.width, msg.encoding = frame.shape[0], frame.shape[1], 'bgr8'
        msg.step = frame.shape[1] * 3
        msg.data = np.ascontiguousarray(frame).tobytes()
        self.pub_img.publish(msg)

        c = self.cam
        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = c.width, c.height
        info.distortion_model = 'plumb_bob' if self.calibrated else 'nominal'
        info.d = [float(v) for v in c.dist]
        info.k = [c.fx, 0.0, c.cx, 0.0, c.fy, c.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [c.fx, 0.0, c.cx, 0.0, 0.0, c.fy, c.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)
        self._n_pub += 1

    def _status(self):
        now = time.monotonic()
        fps = self._n_pub / max(1e-3, now - self._t_rate)
        self._n_pub, self._t_rate = 0, now
        s = String()
        import json
        s.data = json.dumps({'open': self._opened, 'fps': round(fps, 1),
                             'calibrated': self.calibrated, 'reason': self._reason})
        self.pub_status.publish(s)
        if not self._opened:
            self.get_logger().warn(self._reason, throttle_duration_sec=30.0)

    def destroy_node(self):
        self._stop = True
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrontCam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
