#!/usr/bin/env python3
"""Does Nav2's costmap actually mark and clear from the camera scan? No rover.

Runs the real nav2_costmap_2d node with the SAME `front_cam_layer` block that is
in nav2_params.yaml (read from that file, not copied), feeds it LaserScans like
front_cam_scan makes, and reads the costmap back:

  * with no camera running the layer is inert: the costmap is empty and alive
    (the camera is additive, so a dead camera must change nothing);
  * a scan with one hit marks a lethal cell at that bearing and range, and the
    inflation layer spreads cost around it;
  * a later scan that is +inf there clears it (inf_is_valid), and clears ONLY
    this layer's own marks;
  * a scan that is +inf everywhere leaves unrelated marks alone, by living in
    its own layer: the lidar's marks are never erased by the camera.

    ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1 python3 tools/test_front_cam_costmap.py
"""
import math
import os
import subprocess
import sys
import tempfile
import time

import rclpy
import yaml
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.join(HERE, '..', 'ros2_ws', 'src', 'my_robot_bringup', 'config', 'nav2_params.yaml')
COSTMAP = '/opt/ros/humble/lib/nav2_costmap_2d/nav2_costmap_2d'
PASS, FAIL = [], []


def check(name, ok, detail=''):
    (PASS if ok else FAIL).append(name)
    print(('PASS  ' if ok else 'FAIL  ') + name + (('  -- ' + detail) if detail else ''))


def load_layer_block():
    """The front_cam_layer block from the real local costmap config."""
    with open(PARAMS) as f:
        p = yaml.safe_load(f)
    lc = p['local_costmap']['local_costmap']['ros__parameters']
    return lc


def test_params(tmp):
    lc = load_layer_block()
    assert 'front_cam_layer' in lc['plugins'], 'front_cam_layer is not in the local costmap plugins'
    cfg = {'costmap': {'costmap': {'ros__parameters': {
        'global_frame': 'odom', 'robot_base_frame': 'base_link', 'rolling_window': True,
        'width': 4, 'height': 4, 'resolution': 0.05, 'robot_radius': 0.33,
        'update_frequency': 10.0, 'publish_frequency': 10.0,
        # Without this Nav2 publishes only deltas on /costmap_updates once running.
        'always_send_full_costmap': True,
        # Only the camera layer and the inflation layer: what we are testing.
        'plugins': ['obstacle_layer', 'front_cam_layer', 'inflation_layer'],
        'obstacle_layer': lc['obstacle_layer'],      # the real lidar layer
        'front_cam_layer': lc['front_cam_layer'],
        'inflation_layer': lc['inflation_layer']}}}}
    path = os.path.join(tmp, 'costmap_test.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(cfg, f)
    return path


class Rig(Node):
    def __init__(self):
        super().__init__('front_cam_costmap_rig')
        self.pub = self.create_publisher(LaserScan, '/front_cam/scan', qos_profile_sensor_data)
        self.pub_lidar = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.grid = None
        self.create_subscription(
            OccupancyGrid, '/costmap/costmap', self._grid,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.lifecycle_clients = {}

    def _grid(self, m):
        self.grid = m

    def lifecycle(self, node, transition):
        c = self.lifecycle_clients.setdefault(node, self.create_client(ChangeState, node + '/change_state'))
        if not c.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('no lifecycle service for ' + node)
        req = ChangeState.Request()
        req.transition.id = transition
        fut = c.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        return fut.result() is not None and fut.result().success

    def scan(self, ranges, bearing_min_deg=-30.0, seconds=1.5):
        n = len(ranges)
        end = time.time() + seconds
        while time.time() < end:
            s = LaserScan()
            s.header.stamp = self.get_clock().now().to_msg()
            s.header.frame_id = 'front_cam_scan'
            s.angle_min = math.radians(bearing_min_deg)
            s.angle_increment = math.radians(60.0) / n
            s.angle_max = s.angle_min + s.angle_increment * (n - 1)
            s.range_min, s.range_max = 0.25, 2.5
            s.ranges = [float(r) for r in ranges]
            self.pub.publish(s)
            rclpy.spin_once(self, timeout_sec=0.05)

    def lidar_hit(self, x, y, seconds=1.5):
        """A 360 degree lidar scan with a single return at map point (x, y).
        The lidar is at (-0.155, 0, 0.275) facing backwards (yaw pi), as in the URDF."""
        lx, ly = -(x + 0.155), -y                    # the point in the laser frame
        n = 720
        ranges = [math.inf] * n
        bearing = math.atan2(ly, lx)
        ranges[int(round((bearing + math.pi) / (2 * math.pi / n))) % n] = math.hypot(lx, ly)
        end = time.time() + seconds
        while time.time() < end:
            s = LaserScan()
            s.header.stamp = self.get_clock().now().to_msg()
            s.header.frame_id = 'laser_frame'
            s.angle_min, s.angle_increment = -math.pi, 2 * math.pi / n
            s.angle_max = s.angle_min + s.angle_increment * (n - 1)
            s.range_min, s.range_max = 0.15, 12.0
            s.ranges = ranges
            self.pub_lidar.publish(s)
            rclpy.spin_once(self, timeout_sec=0.05)

    def cell(self, x, y):
        """Costmap value at (x, y) in the odom frame; -1 if outside."""
        g = self.grid
        if g is None:
            return None
        c = int((x - g.info.origin.position.x) / g.info.resolution)
        r = int((y - g.info.origin.position.y) / g.info.resolution)
        if not (0 <= c < g.info.width and 0 <= r < g.info.height):
            return -1
        return int(g.data[r * g.info.width + c])

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    os.environ.setdefault('ROS_DOMAIN_ID', '78')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    tmp = tempfile.mkdtemp(prefix='fc_costmap_')
    params = test_params(tmp)
    procs = []

    def start(cmd):
        out = subprocess.DEVNULL
        if os.environ.get('FC_TEST_LOG') and cmd[0] == COSTMAP:
            out = open(os.environ['FC_TEST_LOG'], 'w')   # to see Nav2's own warnings
        p = subprocess.Popen(cmd, stdout=out, stderr=out, start_new_session=True)
        procs.append(p)
    # camera 0.25 m ahead of base_link, 25 cm up, facing forward; odom == base
    start(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0', '0', '0', '0', '0', '0', 'odom', 'base_link'])
    start(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0.25', '0', '0.25', '0', '0', '0', 'base_link', 'front_cam_scan'])
    start(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '-0.155', '0', '0.275', '3.14159265', '0', '0', 'base_link', 'laser_frame'])
    start([COSTMAP, '--ros-args', '--params-file', params])
    rclpy.init()
    rig = None
    try:
        rig = Rig()
        time.sleep(3.0)
        check('costmap node configures', rig.lifecycle('/costmap/costmap', Transition.TRANSITION_CONFIGURE))
        check('costmap node activates', rig.lifecycle('/costmap/costmap', Transition.TRANSITION_ACTIVATE))
        rig.spin(2.0)

        check('costmap publishes with no camera running', rig.grid is not None)
        check('and the camera layer is inert: nothing is marked',
              rig.grid is not None and max(rig.grid.data) == 0, 'max %s' % (max(rig.grid.data) if rig.grid else None))

        # one hit dead ahead at 1.20 m from the camera = x 1.45 in base/odom
        n = 90
        ranges = [math.inf] * n
        ranges[n // 2] = 1.20            # bin straight ahead (-30 + 60*(45.5/90) ~ 0.3 deg)
        rig.scan(ranges)
        rig.spin(0.5)
        v = rig.cell(0.25 + 1.20, 0.0)
        check('a camera hit marks a lethal cell at that range', v is not None and v >= 99, 'cell=%s' % v)
        check('inflation spreads cost beside it', (rig.cell(0.25 + 1.20, 0.15) or 0) > 0,
              'cell=%s' % rig.cell(0.25 + 1.20, 0.15))
        check('cells well away stay free', rig.cell(0.25 + 0.60, 0.0) == 0 and rig.cell(0.25 + 1.20, 0.9) == 0)

        # the same bearing now reads clear (+inf): this layer's own mark goes
        rig.scan([math.inf] * n, seconds=2.0)
        rig.spin(0.5)
        v = rig.cell(0.25 + 1.20, 0.0)
        check('+inf clears the mark (inf_is_valid)', v == 0, 'cell=%s' % v)

        # The point of a separate layer: the lidar marks a cell, the camera's rays
        # pass straight through it reading "floor", and the lidar's mark survives.
        lx, ly = 1.0, 0.30
        rig.lidar_hit(lx, ly)
        rig.spin(0.5)
        before = rig.cell(lx, ly)
        check('the lidar layer marks its own hit', before is not None and before >= 99, 'cell=%s' % before)
        # camera rays from (0.25, 0) to beyond (1.0, 0.30): bearing ~21.8 deg left
        rig.scan([math.inf] * n, seconds=2.0)
        rig.spin(0.5)
        after = rig.cell(lx, ly)
        check('camera "floor" rays do not erase a lidar mark', after == before,
              'before=%s after=%s' % (before, after))
    finally:
        if rig is not None:
            rig.destroy_node()
        rclpy.shutdown()
        for p in procs:
            try:
                os.killpg(p.pid, 15)
            except ProcessLookupError:
                pass
        for p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, 9)

    print('\n%d passed, %d failed' % (len(PASS), len(FAIL)))
    return 1 if FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
