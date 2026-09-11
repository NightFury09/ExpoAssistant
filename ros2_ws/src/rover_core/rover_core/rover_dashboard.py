#!/usr/bin/env python3
"""Web dashboard: drive the rover from a browser, with live camera and metrics.

    ros2 run rover_core rover_dashboard
    -> open http://<jetson-ip>:8080  (this machine: http://192.168.3.224:8080)

Why a ROS node serving its own page, rather than a static web app: the page has
to publish /cmd_vel and read live topics, so something inside the ROS graph has
to bridge it. This uses only PIL, numpy and the Python standard library -- no
rosbridge, no aiohttp, nothing extra to install.

SAFETY -- two independent deadmen:
  * This node republishes the last browser command at CMD_HZ and zeroes it if
    the browser has been silent for DEADMAN_S. Close the tab, lose Wi-Fi, or
    let go of a key and the rover stops.
  * The ESP32's own 1.5 s watchdog stops the motors if /cmd_vel dries up.

CMD_HZ is deliberately 10, not 20+. The ESP32 loop does blocking Modbus and its
micro-ROS transport drops messages if flooded -- that failure looked exactly
like "Nav2 is broken" for hours. See TROUBLESHOOTING.md.
"""
import json, math, threading, time, io, os, re, socket, urllib.parse
import errno, subprocess
import yaml
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy, tf2_ros
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from PIL import Image as PImage

from rclpy.action import ActionClient
from geometry_msgs.msg import (Twist, Point32, PoseStamped,
                               PoseWithCovarianceStamped)
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.srv import GetParameters
from action_msgs.srv import CancelGoal
from slam_toolbox.srv import SerializePoseGraph
from std_msgs.msg import String

from rover_core.stack_supervisor import StackSupervisor, STACKS
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import Image, LaserScan

DEFAULT_WP_FILE = os.path.expanduser(
    '~/AGX_Orin_Backup/rover_project/config/demo_waypoints.yaml')
DEFAULT_MAP_DIR = os.path.expanduser('~/AGX_Orin_Backup/rover_project/maps')

# Stamp of this file. The page carries the same value and reloads itself when
# the two differ, so an open tab can never keep running yesterday's JavaScript
# after a rebuild -- a stale tab is indistinguishable from a broken feature.
try:
    BUILD = str(int(os.path.getmtime(os.path.abspath(__file__))))
except Exception:                                   # noqa: BLE001
    BUILD = str(int(time.time()))

CMD_HZ     = 10.0
DEADMAN_S  = 0.6
STREAM_W   = 640
JPEG_Q     = 70
PORT       = 8080          # default; override with -p port:=8081

SENSOR_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
# /map is latched. A default subscription silently receives nothing.
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)

DEPTH_MIN_M = 0.3      # below Min-Z the D455 has nothing useful
DEPTH_MAX_M = 4.0      # beyond this depth is too noisy to colour meaningfully


def _turbo_lut():
    """256-entry Turbo colormap.

    Turbo rather than the classic jet: it is perceptually monotonic, so a viewer
    reads "closer" and "further" correctly instead of being fooled by jet's
    false bright band in the middle. For a client demo that matters -- the
    picture should be self-explanatory without a legend.
    """
    stops = [(0.00, (48, 18, 59)),  (0.13, (65, 105, 225)),
             (0.25, (33, 168, 230)), (0.38, (27, 214, 181)),
             (0.50, (110, 240, 96)), (0.63, (200, 240, 47)),
             (0.75, (253, 190, 47)), (0.88, (240, 106, 26)),
             (1.00, (163, 22, 12))]
    lut = np.zeros((256, 3), np.uint8)
    for i in range(256):
        t = i / 255.0
        for k in range(len(stops) - 1):
            a, ca = stops[k]
            b, cb = stops[k + 1]
            if a <= t <= b:
                u = (t - a) / (b - a)
                lut[i] = [int(ca[j] + (cb[j] - ca[j]) * u) for j in range(3)]
                break
    return lut


TURBO = _turbo_lut()


class Dash(Node):
    def __init__(self):
        super().__init__('rover_dashboard')
        # The lidar is rear-mounted and yawed 180 deg, so the ROBOT's forward
        # direction is 180 deg in the scan. Used for the obstacle warning.
        self.declare_parameter('forward_lidar_angle_deg', 180.0)
        self.declare_parameter('forward_arc_deg', 60.0)
        self.declare_parameter('max_linear', 0.35)
        self.declare_parameter('max_angular', 1.0)
        self.fwd_ang = float(self.get_parameter('forward_lidar_angle_deg').value)
        self.fwd_arc = float(self.get_parameter('forward_arc_deg').value)
        self.max_lin = float(self.get_parameter('max_linear').value)
        self.max_ang = float(self.get_parameter('max_angular').value)

        self.lock = threading.Lock()
        # One JPEG per view. Only views someone is actually watching get
        # encoded -- see self.wanted -- so opening the page on 'colour' costs
        # nothing for depth/IR/blend.
        self.frames = {'color': None, 'depth': None, 'ir': None, 'blend': None}
        self.wanted = {}           # view -> last time a client asked for it
        self.depth_centre = None   # metres at the image centre, for the HUD
        self.depth_raw = None      # last raw depth frame, for the blend
        self.last_color = None     # PIL image, for the blend view
        # Live-adjustable so the colour ramp can be matched to the space. A
        # 6 m ceiling puts a typical indoor scene in the blue third of the
        # ramp and looks flat; ~4 m makes the same scene read clearly.
        self.d_min = DEPTH_MIN_M
        self.d_max = DEPTH_MAX_M
        self.jpeg = None           # kept: 'color' alias used by older callers
        self.cam_times, self.scan_times = [], []
        self.odom = None
        self.ticks = None
        self.diag = None
        self.enc = None
        self.scan_stats = None
        self.start_xy = None
        self.path_len = 0.0
        self._last_xy = None
        self.path = []            # (x, y) trace for the mini-map
        self.scan_pts = []        # (robot-frame angle deg, range m) for the radar
        self.t_start = time.time()
        self.v_hist = []          # (t, commanded, measured) for the sparkline
        self.max_v = 0.0

        self.cmd = (0.0, 0.0)
        self.cmd_time = 0.0
        self.estop = False
        # Manual-override state. The dashboard must be SILENT on /cmd_vel while
        # idle, otherwise its zero-velocity heartbeat overrides Nav2 and the
        # rover never moves autonomously. Publishing only while actually being
        # driven lets this run alongside master_navigation.launch.py: Nav2 has
        # the wheel by default, and grabbing a key takes over instantly.
        self.manual_until = 0.0     # keep publishing zeros until this time
        self.mode = 'idle'          # idle | manual | estop

        # --- map view state ---
        self.map_png = None         # rendered occupancy grid, PNG bytes
        self.map_grid = None         # raw occupancy values, for saving
        self.map_meta = None        # resolution / origin / size, for pixel<->metre
        self.map_seq = 0            # bumps on every new map, so the UI can cache
        self.map_name = ''          # which map the waypoints belong to
        self.map_pose = None        # robot in the MAP frame (x, y, yaw deg)
        self.plan = []              # current Nav2 path, decimated
        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)

        # --- navigation ---
        # The browser thread only ever writes a REQUEST here; the actual rclpy
        # calls happen on the executor thread in tick_nav(). Calling an action
        # client from the HTTP thread while rclpy.spin() runs in another is the
        # kind of race that fails once an hour and is never reproducible.
        self.nav_req = None         # {'x','y','yaw'} goal waiting to be sent
        self.nav_cancel_req = False
        self.nav = {'state': 'idle', 'goal': None, 'remaining': None,
                    'result': None, 'since': 0.0, 'place': None}
        self.nav_handle = None
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Cancelling through the goal handle only works for goals THIS console
        # sent. A goal published to /goal_pose, or sent from Foxglove, leaves us
        # with no handle at all -- and that is exactly when you most want a stop
        # button that works. The action's own cancel service takes an all-zero
        # request meaning "cancel every goal", whoever sent it.
        self.cancel_cli = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_timer(0.2, self.tick_nav)

        # --- demo waypoints ---
        # Coordinates are only meaningful against one map, so the file records
        # which map they were captured on and the UI warns if a different map
        # is loaded. A saved point on the wrong map is worse than no point --
        # it sends the rover confidently to the wrong place.
        self.declare_parameter('waypoints_file', DEFAULT_WP_FILE)
        self.wp_file = self.get_parameter('waypoints_file').value
        self.wp = {'map': '', 'points': {}}
        self.load_waypoints()

        # Which map is loaded? An OccupancyGrid carries no name, so ask
        # map_server for its yaml_filename. No map_server means SLAM is
        # building the map live, which is itself worth showing.
        self.map_param_cli = self.create_client(
            GetParameters, '/map_server/get_parameters')
        self.create_timer(5.0, self.tick_map_name)

        # --- stack supervisor ---
        # The graph query runs on a ROS timer and the browser thread only ever
        # reads the cached answer: get_node_names() from an HTTP worker thread
        # is a cross-thread rcl call for no benefit.
        # Overridable so a second console can be run for testing without
        # taking the port from the one that owns the rover.
        self.declare_parameter('port', PORT)
        self.port = int(self.get_parameter('port').value)
        self.declare_parameter('map_dir', DEFAULT_MAP_DIR)
        self.map_dir = self.get_parameter('map_dir').value
        self.graph_nodes = []
        self._maps, self._maps_at = [], 0.0
        self.sup = StackSupervisor(lambda: list(self.graph_nodes),
                                   self.get_logger())
        self.save_req = None
        self.saved_map = ''
        self.save_state = {'busy': False, 'msg': '', 'at': 0.0}
        self.slam_ser_cli = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.create_timer(2.0, self.tick_graph)
        self.create_timer(0.5, self.tick_save)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw',
                                 self.on_image, SENSOR_QOS)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw',
                                 self.on_depth, SENSOR_QOS)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw',
                                 self.on_aligned, SENSOR_QOS)
        self.create_subscription(Image, '/camera/camera/infra1/image_rect_raw',
                                 self.on_ir, SENSOR_QOS)
        self.create_subscription(LaserScan, '/scan', self.on_scan, SENSOR_QOS)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED_QOS)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_timer(0.2, self.tick_pose)
        self.create_subscription(Point32, '/wheel_ticks',
                                 lambda m: self._set('ticks', (m.x, m.y, m.z)), 10)
        self.create_subscription(Point32, '/rover_diag',
                                 lambda m: self._set('diag', (m.x, m.y, m.z)), 10)
        self.create_subscription(Point32, '/encoder_ticks',
                                 lambda m: self._set('enc', (m.x, m.y, m.z)), 10)
        self.create_timer(1.0 / CMD_HZ, self.tick_cmd)
        self.get_logger().info(f"dashboard on http://{local_ip()}:{self.port}")

    def _set(self, name, val):
        with self.lock:
            setattr(self, name, (val, time.time()))

    # ---------------- subscriptions ----------------
    def on_image(self, m):
        try:
            if m.encoding in ('rgb8', 'bgr8'):
                a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
                if m.encoding == 'bgr8':
                    a = a[:, :, ::-1]
            elif m.encoding == 'mono8':
                a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
                a = np.dstack([a] * 3)
            else:
                return
            im = PImage.fromarray(a)
            if im.width > STREAM_W:
                im = im.resize((STREAM_W, int(im.height * STREAM_W / im.width)),
                               PImage.BILINEAR)
            buf = io.BytesIO()
            im.save(buf, 'JPEG', quality=JPEG_Q)
            with self.lock:
                self.jpeg = buf.getvalue()
                self.frames['color'] = self.jpeg
                self.last_color = im
                self.cam_times.append(time.time())
                self.cam_times = self.cam_times[-30:]
        except Exception as e:
            self.get_logger().warn(f"image: {e}", throttle_duration_sec=5.0)

    def tick_graph(self):
        try:
            self.graph_nodes = [n for n, _ in self.get_node_names_and_namespaces()]
        except Exception:                           # noqa: BLE001
            return
        # Two nodes with the same name is legal in ROS 2 and quietly poisonous:
        # service calls and parameter sets go to whichever answers first. A
        # second console is a fine thing to run -- on another port, with its own
        # node name -- so say how rather than just complaining.
        n = sum(1 for x in self.graph_nodes if x == self.get_name())
        if n > 1:
            self.get_logger().warn(
                f'{n} nodes are called "{self.get_name()}". Services and '
                'parameters will go to whichever answers first. Give the '
                'second one its own name:\n'
                '    ros2 run rover_core rover_dashboard --ros-args '
                '-r __node:=rover_dashboard_test -p port:=8081',
                throttle_duration_sec=60.0)

    def list_maps(self):
        """Map YAMLs available to load, newest first. Cached: metrics() runs
        several times a second and this is a directory scan."""
        now = time.time()
        if now - self._maps_at < 5.0:
            return self._maps
        self._maps_at = now
        try:
            out = []
            for f in os.listdir(self.map_dir):
                if not f.endswith('.yaml'):
                    continue
                full = os.path.join(self.map_dir, f)
                pgm = full[:-5] + '.pgm'
                if not os.path.exists(pgm):
                    continue            # a yaml with no image cannot be loaded
                out.append({'name': f[:-5], 'path': full,
                            'mtime': int(os.path.getmtime(full))})
            self._maps = sorted(out, key=lambda m: -m['mtime'])
            return self._maps
        except Exception as e:                      # noqa: BLE001
            self.get_logger().warn(f'map list: {e}', throttle_duration_sec=30.0)
            return self._maps

    # ---- saving a map ------------------------------------------------
    def tick_save(self):
        """Write the map ourselves, from the grid we already have.

        This used to call slam_toolbox's /slam_toolbox/save_map and gate on
        service_is_ready(). That check returned False while slam_toolbox was
        demonstrably running and mapping, so a perfectly good map could not be
        saved -- and the same check works fine against map_server, so it is not
        something worth relying on either way.

        The console is already subscribed to /map and holds the whole grid, so
        it needs neither the service nor a subprocess: no discovery, no
        timeout, nothing to be unavailable. The pose graph still comes from
        slam_toolbox -- only it has that -- but it is requested best-effort and
        never blocks the save.
        """
        with self.lock:
            req = self.save_req
            self.save_req = None
        if req is None:
            return
        name = self.clean_name(req).replace(' ', '_')
        if not name:
            self._save_msg('name must contain a letter or a number')
            return
        with self.lock:
            grid, meta = self.map_grid, self.map_meta
        if grid is None or meta is None:
            self._save_msg('no map received yet — is a stack running?')
            return

        path = os.path.join(self.map_dir, name)
        try:
            os.makedirs(self.map_dir, exist_ok=True)
            self.write_map(grid, meta, path)
        except Exception as e:                      # noqa: BLE001
            self._save_msg(f'could not write {path}.pgm: {e}')
            return

        cells = grid.size
        unknown = int((grid < 0).sum())
        self._save_msg(f'saved {name} · {meta["w"]}x{meta["h"]} · '
                       f'{100.0 * unknown / cells:.0f}% unsurveyed')
        self._maps_at = 0.0                         # refresh the picker now
        self.saved_map = path + '.yaml'             # so "switch" can use it
        self.get_logger().info(f'map saved: {path}.yaml')

        # Best effort, and deliberately after the map is already on disk: the
        # pose graph only matters for CONTINUING this map in a later session.
        if self.slam_ser_cli.service_is_ready():
            self.slam_ser_cli.call_async(
                SerializePoseGraph.Request(filename=path))

    @staticmethod
    def write_map(grid, meta, path):
        """OccupancyGrid -> the .pgm/.yaml pair map_server loads.

        free_thresh is 0.19, not map_saver's 0.25: the unknown grey 205 works
        out to 0.196, so at 0.25 every never-surveyed cell is republished as
        FREE FLOOR and the planner routes straight through it. See
        TROUBLESHOOTING.md.
        """
        img = np.full(grid.shape, 205, np.uint8)    # unknown, and anything
        img[grid >= 0] = 205                        # ambiguous, stays grey
        img[(grid >= 0) & (grid <= 25)] = 254       # free
        img[grid >= 65] = 0                         # occupied
        # A PGM's first row is the TOP of the image; the grid's first row is
        # the LOWEST y in the map frame.
        PImage.fromarray(img[::-1], 'L').save(path + '.pgm')
        with open(path + '.yaml', 'w') as fh:
            fh.write(
                f"image: {os.path.basename(path)}.pgm\n"
                f"mode: trinary\n"
                f"resolution: {meta['res']:.6f}\n"
                f"origin: [{meta['ox']:.6f}, {meta['oy']:.6f}, 0.0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.19\n")

    def _save_msg(self, msg):
        with self.lock:
            self.save_state = {'busy': False, 'msg': msg, 'at': time.time()}
        self.get_logger().info(f'save map: {msg}')

    def tick_map_name(self):
        if not self.map_param_cli.service_is_ready():
            with self.lock:
                if self.map_png is not None and self.map_name != 'live SLAM':
                    self.map_name = 'live SLAM'
            return
        req = GetParameters.Request(names=['yaml_filename'])
        self.map_param_cli.call_async(req).add_done_callback(self._on_map_name)

    def _on_map_name(self, fut):
        try:
            vals = fut.result().values
            path = vals[0].string_value if vals else ''
        except Exception:                           # noqa: BLE001
            return
        name = os.path.splitext(os.path.basename(path))[0] if path else ''
        with self.lock:
            if name and name != self.map_name:
                self.map_name = name
                self.get_logger().info(f"map in use: {name}")

    # ---- demo waypoints ----------------------------------------------
    def load_waypoints(self):
        try:
            with open(self.wp_file) as fh:
                d = yaml.safe_load(fh) or {}
            pts = d.get('points') or {}
            with self.lock:
                self.wp = {'map': d.get('map', ''),
                           'points': {str(k): {'x': float(v['x']),
                                               'y': float(v['y']),
                                               'yaw': float(v.get('yaw', 0.0))}
                                      for k, v in pts.items()}}
            self.get_logger().info(
                f"waypoints: {len(self.wp['points'])} from {self.wp_file}")
        except FileNotFoundError:
            self.get_logger().info(f"waypoints: none yet ({self.wp_file})")
        except Exception as e:                      # noqa: BLE001
            self.get_logger().error(f"waypoints: cannot read {self.wp_file}: {e}")

    def save_waypoints(self):
        """Write via a temp file and rename, so a crash mid-write cannot leave
        a half-written file that loses every demo point."""
        os.makedirs(os.path.dirname(self.wp_file), exist_ok=True)
        with self.lock:
            data = {'map': self.wp['map'],
                    'points': {k: dict(v) for k, v in self.wp['points'].items()}}
        tmp = self.wp_file + '.tmp'
        with open(tmp, 'w') as fh:
            fh.write("# Demo waypoints for the rover console.\n"
                     "# Captured against the map named below -- coordinates do "
                     "not carry across maps.\n")
            yaml.safe_dump(data, fh, default_flow_style=False, sort_keys=True)
        os.replace(tmp, self.wp_file)

    @staticmethod
    def clean_name(n):
        n = re.sub(r'[^A-Za-z0-9 _-]', '', str(n)).strip()
        return n[:40]

    def set_waypoint(self, name, x, y, yaw):
        name = self.clean_name(name)
        if not name:
            return False, 'name must contain a letter or a number'
        with self.lock:
            self.wp['points'][name] = {'x': round(float(x), 3),
                                       'y': round(float(y), 3),
                                       'yaw': round(float(yaw), 1)}
            if not self.wp['map']:
                self.wp['map'] = self.map_name
        self.save_waypoints()
        self.get_logger().info(f"waypoint '{name}' -> ({x:.2f}, {y:.2f})")
        return True, name

    def del_waypoint(self, name):
        with self.lock:
            gone = self.wp['points'].pop(self.clean_name(name), None) is not None
        if gone:
            self.save_waypoints()
        return gone

    # ---- navigation --------------------------------------------------
    def _set_nav(self, state, **kw):
        with self.lock:
            self.nav['state'] = state
            self.nav['since'] = time.time()
            self.nav.update(kw)

    def tick_nav(self):
        """Drain browser requests on the ROS thread."""
        with self.lock:
            req, cancel = self.nav_req, self.nav_cancel_req
            self.nav_req, self.nav_cancel_req = None, False

        if cancel:
            h = self.nav_handle
            if h is not None:
                h.cancel_goal_async()
            sent = False
            if any(n.lstrip('/') == 'bt_navigator' for n in self.graph_nodes):
                # All-zero goal_id and stamp: cancel everything.
                fut = self.cancel_cli.call_async(CancelGoal.Request())
                fut.add_done_callback(self._nav_cancelled)
                sent = True
            if h is not None or sent:
                self._set_nav('cancelling')
                # Nav2 stops on its own, but one explicit zero removes any doubt
                # about a command already in flight to the ESP32.
                self.pub.publish(Twist())
                self.get_logger().info('cancel requested')
            else:
                self._set_nav('idle', goal=None, remaining=None,
                              result='nothing was running')

        if req is None:
            return
        if not self.nav_client.server_is_ready():
            self._set_nav('no server', result='navigate_to_pose not available')
            return

        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(req['x'])
        g.pose.pose.position.y = float(req['y'])
        th = math.radians(float(req['yaw'])) / 2.0
        g.pose.pose.orientation.z = math.sin(th)
        g.pose.pose.orientation.w = math.cos(th)
        self._set_nav('sending', goal=[round(req['x'], 3), round(req['y'], 3),
                                       round(req['yaw'], 1)],
                      place=req.get('place'), remaining=None, result=None)
        fut = self.nav_client.send_goal_async(g, feedback_callback=self._nav_fb)
        fut.add_done_callback(self._nav_accepted)
        self.get_logger().info(
            f"goal -> ({req['x']:.2f}, {req['y']:.2f}) @ {req['yaw']:.0f} deg")

    def _nav_accepted(self, fut):
        try:
            h = fut.result()
        except Exception as e:                      # noqa: BLE001
            self._set_nav('failed', result=f'send failed: {e}')
            return
        if not h.accepted:
            self._set_nav('rejected', result='goal rejected by Nav2')
            return
        self.nav_handle = h
        self._set_nav('active')
        h.get_result_async().add_done_callback(self._nav_done)

    def _nav_cancelled(self, fut):
        """Report what the action server actually cancelled.

        Without this the UI would sit on CANCELLING for ever whenever the
        button was pressed with no goal running -- which is most of the time
        someone presses it to check that it works.
        """
        try:
            n = len(fut.result().goals_canceling)
        except Exception:                           # noqa: BLE001
            n = 0
        if n:
            self._set_nav('idle', goal=None, remaining=None,
                          result=f'cancelled {n} goal' + ('s' if n > 1 else ''))
        else:
            self._set_nav('idle', goal=None, remaining=None,
                          result='nothing was running')
        self.get_logger().info(f'cancel: {n} goal(s) cancelling')

    def _nav_fb(self, fb):
        with self.lock:
            self.nav['remaining'] = round(
                float(fb.feedback.distance_remaining), 2)

    def _nav_done(self, fut):
        from action_msgs.msg import GoalStatus
        self.nav_handle = None
        try:
            st = fut.result().status
        except Exception as e:                      # noqa: BLE001
            self._set_nav('failed', result=str(e)); return
        name = {GoalStatus.STATUS_SUCCEEDED: ('arrived', 'reached the goal'),
                GoalStatus.STATUS_CANCELED: ('idle', 'cancelled'),
                GoalStatus.STATUS_ABORTED: ('aborted', 'Nav2 gave up')
                }.get(st, ('idle', f'status {st}'))
        self._set_nav(name[0], result=name[1], remaining=None)
        self.get_logger().info(f"goal finished: {name[1]}")

    def set_initial_pose(self, x, y, yaw_deg):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = float(x)
        m.pose.pose.position.y = float(y)
        th = math.radians(float(yaw_deg)) / 2.0
        m.pose.pose.orientation.z = math.sin(th)
        m.pose.pose.orientation.w = math.cos(th)
        # Nav2's own RViz plugin uses these; zeros would tell AMCL the estimate
        # is exact and it would refuse to correct a sloppy click.
        c = list(m.pose.covariance)
        c[0] = c[7] = 0.25          # 0.5 m std dev in x and y
        c[35] = 0.068               # ~15 deg std dev in yaw
        m.pose.covariance = c
        self.pose_pub.publish(m)
        self.get_logger().info(
            f"initial pose -> ({x:.2f}, {y:.2f}) @ {yaw_deg:.0f} deg")

    # ---- map view ----------------------------------------------------
    def on_map(self, m):
        """Render the occupancy grid once per map, not per request.

        Three tones, the convention RViz users already read: light = surveyed
        and drivable, black = wall, mid slate = never seen. Unknown is PAINTED
        rather than left transparent -- an earlier transparent version put black
        walls straight onto the near-black panel, which made the outer wall of
        the room invisible exactly where it matters most.
        """
        try:
            w, h = m.info.width, m.info.height
            g = np.asarray(m.data, dtype=np.int16).reshape(h, w)
            rgba = np.zeros((h, w, 4), np.uint8)
            free = (g >= 0) & (g <= 25)
            occ = g >= 65
            mid = (g > 25) & (g < 65)
            rgba[...] = (44, 55, 68, 255)          # unknown (-1): mid slate
            rgba[free] = (232, 238, 245, 255)      # light: drivable
            rgba[mid] = (140, 158, 176, 255)       # ambiguous
            rgba[occ] = (5, 7, 10, 255)            # black: wall
            # The grid's row 0 is the LOWEST y in the map frame, but an image's
            # row 0 is the top, so flip before encoding.
            im = PImage.fromarray(rgba[::-1], 'RGBA')
            buf = io.BytesIO()
            im.save(buf, 'PNG', optimize=True)
            with self.lock:
                self.map_grid = g
                self.map_png = buf.getvalue()
                self.map_meta = {
                    'w': w, 'h': h,
                    'res': m.info.resolution,
                    'ox': m.info.origin.position.x,
                    'oy': m.info.origin.position.y,
                }
                self.map_seq += 1
            self.get_logger().info(
                f"map: {w}x{h} @ {m.info.resolution:.3f} m/px "
                f"({w*m.info.resolution:.1f} x {h*m.info.resolution:.1f} m)")
        except Exception as e:
            self.get_logger().warn(f"map render: {e}", throttle_duration_sec=10.0)

    def on_plan(self, m):
        pts = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)]
               for p in m.poses]
        if len(pts) > 300:
            step = len(pts) // 300 + 1
            pts = pts[::step]
        with self.lock:
            self.plan = pts

    def tick_pose(self):
        """Robot pose in the MAP frame.

        Separate from /odom: odom drifts, and everything drawn on the map --
        the robot marker, scan points, waypoints -- has to agree with the map,
        not with the odometry origin. Absent until AMCL is localised, which is
        itself useful to show.
        """
        try:
            t = self.tf_buf.lookup_transform('map', 'base_footprint',
                                             rclpy.time.Time())
        except Exception:
            # Clear it. Returning early kept the LAST known pose on screen for
            # ever, so a rover that had lost localisation -- or never had it,
            # because AMCL was still waiting for an initial pose -- went on
            # being drawn sitting confidently on the map. A stale pose is worse
            # than no pose: it is the one number everything else is judged
            # against.
            with self.lock:
                self.map_pose = None
            return
        q = t.transform.rotation
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                      1 - 2 * (q.y * q.y + q.z * q.z)))
        with self.lock:
            self.map_pose = {'x': round(t.transform.translation.x, 3),
                             'y': round(t.transform.translation.y, 3),
                             'yaw': round(yaw, 1)}

    # ---- depth views -------------------------------------------------
    def _want(self, view):
        """Only spend CPU on views someone is watching."""
        with self.lock:
            t = self.wanted.get(view, 0.0)
        return (time.time() - t) < 5.0

    def _encode(self, im, view, quality=JPEG_Q):
        if im.width > STREAM_W:
            im = im.resize((STREAM_W, int(im.height * STREAM_W / im.width)),
                           PImage.BILINEAR)
        buf = io.BytesIO()
        im.save(buf, 'JPEG', quality=quality)
        with self.lock:
            self.frames[view] = buf.getvalue()

    def _colourise(self, d_mm):
        """uint16 depth in mm -> Turbo RGB. Invalid pixels stay near-black."""
        valid = d_mm > 0
        d_m = d_mm.astype(np.float32) / 1000.0
        with self.lock:
            lo, hi = self.d_min, self.d_max
        t = (d_m - lo) / max(hi - lo, 0.1)
        idx = np.clip(t * 255.0, 0, 255).astype(np.uint8)
        rgb = TURBO[idx]
        rgb[~valid] = (14, 18, 24)      # matches the panel background
        return rgb

    def _depth_array(self, m):
        if m.encoding != '16UC1':
            return None
        a = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
        return a[::2, ::2] if m.width > 2 * STREAM_W else a

    def on_depth(self, m):
        try:
            d = self._depth_array(m)
            if d is None:
                return
            # Centre distance is cheap and always worth having for the HUD.
            h, w = d.shape
            patch = d[h // 2 - 4:h // 2 + 5, w // 2 - 4:w // 2 + 5]
            good = patch[patch > 0]
            with self.lock:
                self.depth_centre = (float(np.median(good)) / 1000.0
                                     if good.size else None)
            if self._want('depth'):
                self._encode(PImage.fromarray(self._colourise(d)), 'depth')
        except Exception as e:
            self.get_logger().warn(f"depth: {e}", throttle_duration_sec=5.0)

    def on_aligned(self, m):
        if not self._want('blend'):
            return
        try:
            d = self._depth_array(m)
            with self.lock:
                col = self.last_color
            if d is None or col is None:
                return
            dep = PImage.fromarray(self._colourise(d)).resize(col.size,
                                                             PImage.BILINEAR)
            # 55% depth over the colour frame: enough to read the depth field
            # while the scene stays recognisable, which is the point of the view.
            self._encode(PImage.blend(col.convert('RGB'), dep, 0.55), 'blend')
        except Exception as e:
            self.get_logger().warn(f"blend: {e}", throttle_duration_sec=5.0)

    def on_ir(self, m):
        if not self._want('ir'):
            return
        try:
            if m.encoding != 'mono8':
                return
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
            if m.width > 2 * STREAM_W:
                a = a[::2, ::2]
            self._encode(PImage.fromarray(a).convert('RGB'), 'ir')
        except Exception as e:
            self.get_logger().warn(f"ir: {e}", throttle_duration_sec=5.0)

    def on_scan(self, m):
        rng = np.asarray(m.ranges, dtype=np.float32)
        good = np.isfinite(rng) & (rng > m.range_min) & (rng < m.range_max)
        n_valid = int(good.sum())
        ang = np.degrees(m.angle_min + np.arange(len(rng)) * m.angle_increment)
        # wrap the forward window across +/-180
        d = np.abs(((ang - self.fwd_ang + 180.0) % 360.0) - 180.0)
        sel = good & (d <= self.fwd_arc / 2.0)
        nearest = float(rng[sel].min()) if sel.any() else float('inf')

        # Downsample for the radar, and rotate into the ROBOT frame: the lidar
        # is rear-mounted and yawed 180 deg, so 0 deg on the display means the
        # direction the rover actually drives.
        idx = np.where(good)[0]
        if len(idx) > 220:
            idx = idx[np.linspace(0, len(idx) - 1, 220).astype(int)]
        ra = ((ang[idx] - self.fwd_ang + 180.0) % 360.0) - 180.0
        pts = [[round(float(a), 1), round(float(r), 3)] for a, r in zip(ra, rng[idx])]

        with self.lock:
            self.scan_pts = pts
            self.scan_stats = ({
                'valid_pct': 100.0 * n_valid / max(len(rng), 1),
                'n_valid': n_valid,
                'nearest_fwd': nearest,
            }, time.time())
            self.scan_times.append(time.time())
            self.scan_times = self.scan_times[-30:]

    def on_odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                      1 - 2 * (q.y * q.y + q.z * q.z)))
        with self.lock:
            if self.start_xy is None:
                self.start_xy = (p.x, p.y)
            if self._last_xy is not None:
                self.path_len += math.dist((p.x, p.y), self._last_xy)
            self._last_xy = (p.x, p.y)
            if not self.path or math.dist((p.x, p.y), self.path[-1]) > 0.03:
                self.path.append((round(p.x, 3), round(p.y, 3)))
                self.path = self.path[-400:]
            self.max_v = max(self.max_v, abs(m.twist.twist.linear.x))
            self.odom = ({'x': p.x, 'y': p.y, 'yaw': yaw,
                          'v': m.twist.twist.linear.x,
                          'w': m.twist.twist.angular.z,
                          'from_start': math.dist((p.x, p.y), self.start_xy),
                          'path': self.path_len}, time.time())

    # ---------------- command loop ----------------
    def tick_cmd(self):
        now = time.time()
        with self.lock:
            lin, ang = self.cmd
            fresh = (now - self.cmd_time) < DEADMAN_S
            stopped = self.estop
            until = self.manual_until

        if stopped:
            # E-stop is a hard override: keep asserting zero so nothing else,
            # Nav2 included, can drive the rover.
            self._set_mode('estop')
            self.pub.publish(Twist())
            return

        driving = fresh and (abs(lin) > 1e-6 or abs(ang) > 1e-6)
        if driving:
            # Hold the channel for a moment after the last input so releasing a
            # key reliably produces a stop rather than leaving the last velocity
            # latched in the ESP32.
            with self.lock:
                self.manual_until = now + 0.8
            self._set_mode('manual')
            t = Twist()
            t.linear.x = float(lin)
            t.angular.z = float(ang)
            self.pub.publish(t)
            return

        if now < until:
            # Just released: assert zero briefly, then hand the topic back.
            self._set_mode('manual')
            self.pub.publish(Twist())
            return

        # Idle: publish NOTHING so Nav2 (or anything else) owns /cmd_vel.
        self._set_mode('idle')

    def _set_mode(self, m):
        if self.mode != m:
            self.mode = m
            self.get_logger().info(f"/cmd_vel: {m}")

    def set_cmd(self, lin, ang):
        lin = max(-self.max_lin, min(self.max_lin, float(lin)))
        ang = max(-self.max_ang, min(self.max_ang, float(ang)))
        with self.lock:
            if self.estop:
                lin = ang = 0.0
            self.cmd = (lin, ang)
            self.cmd_time = time.time()

    # ---------------- metrics ----------------
    def metrics(self):
        now = time.time()
        def hz(ts):
            return (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 2 and ts[-1] > ts[0] else 0.0
        def fresh(entry, limit):
            return entry is not None and (now - entry[1]) < limit
        with self.lock:
            o = self.odom; tk = self.ticks; dg = self.diag
            en = self.enc; ss = self.scan_stats
            cam_hz = hz(self.cam_times); scan_hz = hz(self.scan_times)
            cmd = self.cmd; cmd_age = now - self.cmd_time; estop = self.estop
            has_img = self.jpeg is not None
            # velocity history for the sparkline
            meas_v = o[0]['v'] if o else 0.0
            self.v_hist.append((now, cmd[0], meas_v))
            self.v_hist = [h for h in self.v_hist if now - h[0] < 20.0][-200:]
            m = {
                'uptime': now - self.t_start,
                'max_v': self.max_v,
                'scan': list(self.scan_pts),
                'map': {'seq': self.map_seq, 'meta': self.map_meta,
                        'ready': self.map_png is not None},
                'map_pose': self.map_pose,
                'nav': dict(self.nav),
                'build': BUILD,
                'stack': self.sup.status(),
                'cam_proc': self.sup.camera_status(),
                'maps': self.list_maps(),
                'save': dict(self.save_state, path=self.saved_map),
                'waypoints': {'map': self.wp['map'],
                              'current_map': self.map_name,
                              'points': {k: dict(v)
                                         for k, v in self.wp['points'].items()}},
                'plan': list(self.plan),
                'path': list(self.path),
                'vhist': [[round(now - t, 2), round(c, 3), round(v, 3)]
                          for t, c, v in self.v_hist[-120:]],
                'limits': {'lin': self.max_lin, 'ang': self.max_ang},
                'cmd': {'lin': cmd[0], 'ang': cmd[1],
                        'active': (not estop) and cmd_age < DEADMAN_S},
                'estop': estop,
                'mode': self.mode,
                'camera': {'hz': cam_hz, 'ok': has_img and cam_hz > 1.0,
                           'centre_m': self.depth_centre,
                           'views': {k: (v is not None)
                                     for k, v in self.frames.items()},
                           'depth_range': [self.d_min, self.d_max]},
                'lidar': {'hz': scan_hz, 'ok': fresh(ss, 2.0),
                          **(ss[0] if ss else {'valid_pct': 0, 'n_valid': 0,
                                               'nearest_fwd': float('inf')})},
                'odom': {'ok': fresh(o, 2.0), **(o[0] if o else {})},
                'ticks': {'ok': fresh(tk, 2.0),
                          'L': tk[0][0] if tk else 0, 'R': tk[0][1] if tk else 0,
                          'fault': tk[0][2] if tk else -1},
                'encoder': {'ok': fresh(en, 3.0),
                            'L': en[0][0] if en else 0, 'R': en[0][1] if en else 0,
                            'fault': en[0][2] if en else -1},
                'diag': {'ok': fresh(dg, 3.0),
                         'cmd_rx': dg[0][0] if dg else 0,
                         'wdog': dg[0][1] if dg else 0,
                         'rpm_L': dg[0][2] if dg else 0},
            }
        n = m['lidar'].get('nearest_fwd', float('inf'))
        if not math.isfinite(n):
            m['lidar']['nearest_fwd'] = None
        return m


def local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 1)); ip = s.getsockname()[0]; s.close()
        return ip
    except Exception:
        return '127.0.0.1'


NODE = None


PAGE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>Rover Control</title><style>
:root{
 --bg:#0b0e12; --panel:#12171d; --panel2:#161c23; --line:#232c36; --line2:#2e3a46;
 --fg:#e8eef5; --dim:#8998a8; --faint:#5c6b7a;
 --ok:#3fb950; --warn:#e3b341; --bad:#f85149; --acc:#4d9fff; --acc2:#1f6feb;
 --mono:ui-monospace,SFMono-Regular,"SF Mono",Menlo,Consolas,monospace;
 --hh:clamp(44px,6vh,54px);          /* header height */
 --g:clamp(8px,1vh,13px);            /* gap / padding rhythm */
 --fs:clamp(11px,1.15vh,13px);       /* body metric size */
}
*{box-sizing:border-box;margin:0;padding:0;min-width:0;min-height:0}
/* The browser's own [hidden]{display:none} is a bare-element rule, so ANY
   class rule here that sets display quietly beats it and el.hidden stops
   working -- silently, and only for that element. That shipped a button which
   was always visible and did nothing when pressed. One rule, once. */
[hidden]{display:none!important}
html,body{height:100%;overflow:hidden}
body{background:var(--bg);color:var(--fg);display:flex;flex-direction:column;
 font:14px/1.45 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Inter,sans-serif;
 -webkit-font-smoothing:antialiased}

/* ---------- header ---------- */
header{flex:none;display:flex;align-items:center;gap:10px;padding:0 var(--g);
 height:var(--hh);background:var(--panel);border-bottom:1px solid var(--line)}
.brand{display:flex;align-items:center;gap:8px;flex:none}
.dot{width:8px;height:8px;border-radius:99px;background:var(--ok);
 box-shadow:0 0 0 3px rgba(63,185,80,.15);flex:none}
.dot.off{background:var(--bad);box-shadow:0 0 0 3px rgba(248,81,73,.15)}
h1{font-size:clamp(12px,1.5vh,14px);font-weight:650;letter-spacing:-.01em;white-space:nowrap}
@media(max-width:760px){h1{display:none}}
.stats{display:flex;gap:5px;flex:1;overflow:hidden}
.chip{display:flex;align-items:center;gap:5px;padding:4px 9px;border-radius:99px;
 background:var(--panel2);border:1px solid var(--line);
 font-size:clamp(9.5px,1.05vh,11.5px);color:var(--dim);white-space:nowrap;flex:none}
.chip b{color:var(--fg);font-family:var(--mono);font-weight:600}
.chip.ok{border-color:rgba(63,185,80,.35)}.chip.ok b{color:var(--ok)}
.chip.warn{border-color:rgba(227,179,65,.35)}.chip.warn b{color:var(--warn)}
.chip.bad{border-color:rgba(248,81,73,.4);background:rgba(248,81,73,.08)}
.chip.bad b{color:var(--bad)}
@media(max-width:980px){.chip.opt{display:none}}
#hcancel{flex:none;padding:8px clamp(9px,1.2vw,15px);border-radius:8px;
 border:1px solid var(--warn);background:rgba(227,179,65,.14);color:var(--warn);
 font:700 clamp(10px,1.15vh,12px)/1 inherit;letter-spacing:.05em;cursor:pointer;
 animation:pulse 1.8s infinite}
#hcancel:hover{background:var(--warn);color:#0b0e12}
#estop{flex:none;padding:8px clamp(10px,1.4vw,18px);border-radius:8px;
 border:1px solid var(--bad);background:var(--bad);color:#fff;
 font:700 clamp(10px,1.15vh,12px)/1 inherit;letter-spacing:.05em;cursor:pointer}
#estop:hover{filter:brightness(1.1)}
#estop.armed{background:rgba(248,81,73,.12);color:var(--bad);animation:pulse 1.4s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.55}}

/* ---------- layout: fills the viewport, never scrolls ---------- */
.grid{flex:1;display:grid;gap:var(--g);padding:var(--g);overflow:hidden;
 /* minmax(0,...) on BOTH columns. A hard px minimum on the right column made
    the grid overflow when the viewport could not satisfy it, and since the page
    deliberately does not scroll, the metric values were clipped off-screen and
    unreachable. Letting both columns shrink guarantees the layout always fits
    whatever window it is given. */
 grid-template-columns:minmax(0,1.72fr) minmax(0,1fr)}
@media(max-width:1000px){.grid{grid-template-columns:minmax(0,1.35fr) minmax(0,1fr)}}
@media(max-width:720px){.grid{grid-template-columns:1fr;grid-template-rows:1.15fr 1fr}}
.col{display:flex;flex-direction:column;gap:var(--g);overflow:hidden}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;
 display:flex;flex-direction:column;overflow:hidden}
.card>h2{flex:none;display:flex;align-items:center;justify-content:space-between;gap:8px;
 font-size:clamp(9px,1.05vh,10.5px);text-transform:uppercase;letter-spacing:.1em;
 color:var(--faint);font-weight:700;padding:calc(var(--g)*.75) var(--g);
 border-bottom:1px solid var(--line)}
.card>h2 .tag{font:600 clamp(8.5px,1vh,10px)/1 var(--mono);color:var(--dim);
 text-transform:none;letter-spacing:0}
.body{flex:1;padding:var(--g);overflow:hidden;display:flex;flex-direction:column}
.canvbox{flex:1;position:relative;overflow:hidden}
canvas{position:absolute;inset:0;width:100%;height:100%;display:block}

/* ---------- view switch (drive / map) ---------- */
.vsw{flex:none;display:flex;background:var(--panel2);border:1px solid var(--line);
 border-radius:8px;padding:2px;gap:2px}
.vsw button{border:0;background:transparent;color:var(--dim);cursor:pointer;
 padding:5px clamp(9px,1.2vw,15px);border-radius:6px;
 font:700 clamp(9px,1.05vh,11px)/1 inherit;letter-spacing:.07em}
.vsw button.on{background:var(--acc2);color:#fff}

/* ---------- map ---------- */
#mapcard{flex:1}
.mapwrap{flex:1;position:relative;overflow:hidden;background:#0d1116;
 cursor:grab;touch-action:none}
.mapwrap.drag{cursor:grabbing}
#mapcv{position:absolute;inset:0;width:100%;height:100%;display:block}
.maptools{position:absolute;top:8px;right:8px;display:flex;flex-direction:column;gap:5px}
.maptools button{width:30px;height:30px;border-radius:7px;border:1px solid var(--line2);
 background:rgba(10,14,18,.82);color:var(--fg);cursor:pointer;
 font:600 13px/1 var(--mono);backdrop-filter:blur(4px)}
.maptools button:hover{border-color:var(--acc);color:var(--acc)}
.maptools button.on{border-color:var(--acc);color:var(--acc);background:rgba(77,159,255,.14)}
.mapinfo{position:absolute;left:10px;bottom:9px;display:flex;gap:5px;flex-wrap:wrap}
.mapinfo span{padding:3px 8px;border-radius:5px;background:rgba(7,10,13,.8);
 border:1px solid var(--line);font:600 clamp(9px,1.05vh,10.5px)/1 var(--mono);color:var(--dim)}
.mapinfo span b{color:var(--fg);font-weight:600}
.mapkey{position:absolute;left:10px;top:9px;display:flex;gap:9px;
 padding:4px 9px;border-radius:6px;background:rgba(7,10,13,.8);
 border:1px solid var(--line);font-size:clamp(8.5px,1vh,10px);color:var(--dim)}
.mapkey i{display:inline-block;width:8px;height:8px;border-radius:2px;
 margin-right:4px;vertical-align:-1px}
.mapempty{position:absolute;inset:0;display:flex;align-items:center;
 justify-content:center;flex-direction:column;gap:6px;text-align:center;
 color:var(--faint);font-size:clamp(10px,1.2vh,12px);padding:20px}
.mapempty b{color:var(--dim);font-size:clamp(11px,1.35vh,13.5px)}

/* ---------- map tools ---------- */
.mtools{position:absolute;top:8px;left:50%;transform:translateX(-50%);
 display:flex;gap:4px;padding:3px;border-radius:9px;
 background:rgba(7,10,13,.86);border:1px solid var(--line);backdrop-filter:blur(5px)}
.mtools button{border:1px solid transparent;background:transparent;color:var(--dim);
 cursor:pointer;padding:5px 11px;border-radius:6px;
 font:700 clamp(8.5px,1vh,10.5px)/1 inherit;letter-spacing:.06em;white-space:nowrap}
.mtools button:hover{color:var(--fg)}
.mtools button.arm{background:var(--acc2);color:#fff;border-color:var(--acc)}
.mtools button.arm.warnarm{background:#8a6d1b;border-color:var(--warn)}
.mtools button.stop{color:var(--bad);border-color:rgba(248,81,73,.4)}
.mtools button.stop:hover{background:rgba(248,81,73,.14)}
.mhint{position:absolute;top:46px;left:50%;transform:translateX(-50%);
 padding:4px 11px;border-radius:6px;background:var(--acc2);color:#fff;
 font:600 clamp(9px,1.05vh,11px)/1 inherit;white-space:nowrap;pointer-events:none}
.mhint.warn{background:#8a6d1b}
.navbar{flex:none;display:flex;align-items:center;gap:7px;padding:6px var(--g);
 border-top:1px solid var(--line);background:var(--panel2)}
.navbar .st{font:700 clamp(9px,1.05vh,11px)/1 var(--mono);letter-spacing:.06em}
.navbar .de{color:var(--dim);font:600 clamp(9px,1.05vh,11px)/1 var(--mono);
 overflow:hidden;text-overflow:ellipsis;white-space:nowrap;flex:1}

/* ---------- mode / stack control ---------- */
.modebar{display:flex;gap:5px;margin-bottom:calc(var(--g)*.7)}
.modebar button{flex:1;border:1px solid var(--line2);background:var(--panel2);
 color:var(--dim);border-radius:7px;padding:8px 4px;cursor:pointer;
 font:700 clamp(9px,1.05vh,11px)/1.35 inherit;letter-spacing:.05em}
.modebar button small{display:block;font:500 clamp(8px,.95vh,9.5px)/1.3 inherit;
 letter-spacing:0;color:var(--faint);margin-top:3px;text-transform:none}
.modebar button:hover:not(:disabled){border-color:var(--acc)}
.modebar button.on{background:var(--acc2);border-color:var(--acc);color:#fff}
.modebar button.on small{color:rgba(255,255,255,.8)}
.modebar button.on.map{background:#6b4ea8;border-color:#a371f7}
.modebar button.stop.on{background:var(--bad);border-color:var(--bad)}
.modebar button:disabled{opacity:.4;cursor:not-allowed}
.mrow{display:flex;gap:5px;align-items:center;margin-bottom:calc(var(--g)*.6)}
.mrow select,.mrow input[type=text]{flex:1;min-width:0;background:var(--bg);
 border:1px solid var(--line);border-radius:6px;color:var(--fg);padding:6px 7px;
 font:inherit;font-size:clamp(10px,1.15vh,11.5px)}
.mrow select:focus,.mrow input:focus{outline:0;border-color:var(--acc)}
.mrow label{flex:none;font-size:clamp(9px,1.05vh,10.5px);color:var(--faint);
 display:flex;align-items:center;gap:4px;cursor:pointer}
.mrow button{flex:none;border:1px solid var(--line2);background:var(--panel2);
 color:var(--fg);border-radius:6px;padding:6px 10px;cursor:pointer;
 font:700 clamp(8.5px,1vh,10.5px)/1 inherit;white-space:nowrap}
.mrow button:hover:not(:disabled){border-color:var(--acc);color:var(--acc)}
.mrow button:disabled{opacity:.4;cursor:not-allowed}
.mrow button.on{background:var(--acc2);border-color:var(--acc);color:#fff}
.mrow button.warn{border-color:var(--warn);color:var(--warn)}
.stnote{font-size:clamp(8.5px,1vh,10.5px);color:var(--faint);line-height:1.5}
.stnote b{color:var(--dim);font-family:var(--mono);font-weight:600}
.stnote.bad{color:var(--bad)}
.stnote.ok{color:var(--ok)}

/* ---------- waypoints ---------- */
.wplist{flex:1;overflow-y:auto;display:flex;flex-direction:column;gap:4px;
 margin:0 calc(var(--g)*-.4);padding:0 calc(var(--g)*.4)}
.wplist::-webkit-scrollbar{width:6px}
.wplist::-webkit-scrollbar-thumb{background:var(--line2);border-radius:9px}
.wp{display:flex;align-items:center;gap:7px;padding:5px 7px;border-radius:7px;
 background:var(--panel2);border:1px solid var(--line)}
.wp:hover{border-color:var(--line2)}
.wp.base{border-color:rgba(63,185,80,.4)}
.wp .nm{flex:1;font-size:clamp(10px,1.2vh,12px);font-weight:600;
 overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.wp .co{font:600 clamp(8.5px,1vh,10px)/1 var(--mono);color:var(--faint);flex:none}
.wp button{flex:none;border:1px solid var(--line2);background:transparent;
 color:var(--dim);cursor:pointer;border-radius:5px;padding:4px 8px;
 font:700 clamp(8.5px,1vh,10px)/1 inherit}
.wp button.at{border-color:var(--warn);color:var(--warn)}
.wp button.at:hover{background:var(--warn);color:#0b0e12}
.wp button.go{border-color:var(--acc2);color:var(--acc)}
.wp button.go:hover{background:var(--acc2);color:#fff}
.wp button.rm:hover{border-color:var(--bad);color:var(--bad)}
.wpadd{flex:none;display:flex;gap:5px;margin-top:calc(var(--g)*.7)}
.wpadd input{flex:1;min-width:0;background:var(--bg);border:1px solid var(--line);
 border-radius:6px;color:var(--fg);padding:6px 8px;
 font:inherit;font-size:clamp(10px,1.2vh,12px)}
.wpadd input:focus{outline:0;border-color:var(--acc)}
.wpadd button{flex:none;border:1px solid var(--acc2);background:var(--acc2);
 color:#fff;border-radius:6px;padding:6px 10px;cursor:pointer;
 font:700 clamp(8.5px,1vh,10.5px)/1 inherit;white-space:nowrap}
.wpadd button:disabled{opacity:.45;cursor:not-allowed}
.wpnote{flex:none;margin-top:5px;font-size:clamp(8.5px,1vh,10.5px);color:var(--faint)}
.wpnote.bad{color:var(--bad)}
.wpnote.ok{color:var(--ok)}
.wpempty{color:var(--faint);font-size:clamp(10px,1.15vh,11.5px);
 text-align:center;padding:14px 8px}

/* ---------- camera ---------- */
#camcard{flex:1}
.vidwrap{flex:1;position:relative;background:#000;overflow:hidden}
#hud-idle{display:none;flex-direction:column;align-items:center;gap:10px;
 text-align:center;padding:0 24px;pointer-events:auto}
#hud-idle.show{display:flex}
#hud-idle b{font-size:clamp(14px,2.2vh,20px);color:var(--fg)}
#hud-idle span{font-size:clamp(11px,1.4vh,13px);color:var(--dim);max-width:34em;
 line-height:1.6}
#hud-idle button{margin-top:4px;border:1px solid var(--acc);background:var(--acc2);
 color:#fff;border-radius:8px;padding:10px 18px;cursor:pointer;
 font:700 clamp(10px,1.2vh,12px)/1 inherit;letter-spacing:.05em}
#hud-idle button:hover{filter:brightness(1.12)}
#feed{position:absolute;inset:0;width:100%;height:100%;object-fit:contain}
.hud{position:absolute;inset:0;pointer-events:none}
.tl{position:absolute;top:8px;left:10px;display:flex;gap:5px}
.badge{padding:3px 8px;border-radius:5px;background:rgba(0,0,0,.62);
 font:600 clamp(9.5px,1.1vh,11px)/1 var(--mono);color:#fff}
.ctr{position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);
 font-size:12px;color:var(--dim)}
.alert{position:absolute;left:0;right:0;bottom:0;padding:7px 12px;
 font:700 clamp(10px,1.2vh,12.5px)/1 inherit;letter-spacing:.04em;display:none}
.alert.show{display:block}
.alert.amber{background:rgba(227,179,65,.92);color:#1a1300}
.alert.red{background:rgba(248,81,73,.94);color:#fff}
.tabs{display:flex;gap:2px;background:var(--panel2);border:1px solid var(--line);
 border-radius:7px;padding:2px}
.tabs button{padding:3px clamp(6px,.9vw,11px);border:0;border-radius:5px;
 background:transparent;color:var(--dim);cursor:pointer;
 font:600 clamp(8.5px,1vh,10.5px)/1 inherit;letter-spacing:.03em;
 text-transform:uppercase;transition:.12s;white-space:nowrap}
.tabs button:hover{color:var(--fg)}
.tabs button.on{background:var(--acc2);color:#fff}
.tabs button:disabled{opacity:.35;cursor:not-allowed}
.legend{flex:none;display:none;align-items:center;gap:8px;
 padding:5px var(--g);border-top:1px solid var(--line);
 font:600 clamp(8.5px,1vh,10px)/1 var(--mono);color:var(--faint)}
.legend.show{display:flex}
.ramp{flex:1;height:7px;border-radius:99px;background:linear-gradient(90deg,
 #30123b,#4169e1,#21a8e6,#1bd6b5,#6ef060,#c8f02f,#fdbe2f,#f06a1a,#a3160c)}

/* ---------- drive ---------- */
#drivecard{flex:none}
.drive{display:flex;gap:clamp(12px,2vw,22px);align-items:flex-start}
.dpad{display:grid;grid-template-columns:repeat(3,clamp(40px,5.2vh,56px));
 grid-auto-rows:clamp(40px,5.2vh,56px);gap:6px;flex:none}
.key{display:flex;flex-direction:column;align-items:center;justify-content:center;gap:1px;
 background:var(--panel2);border:1px solid var(--line2);border-radius:8px;color:var(--fg);
 cursor:pointer;user-select:none;-webkit-user-select:none;touch-action:none;
 transition:transform .06s,background .12s,border-color .12s}
.key span{font-size:clamp(12px,1.7vh,16px);line-height:1}
.key i{font:600 clamp(7px,.9vh,9px)/1 var(--mono);color:var(--faint);font-style:normal}
.key:hover{background:#1c242d;border-color:#3a4753}
.key.held{background:var(--acc2);border-color:var(--acc);color:#fff;transform:scale(.94)}
.key.held i{color:rgba(255,255,255,.75)}
.sliders{flex:1;display:flex;flex-direction:column;gap:clamp(7px,1.1vh,13px)}
.sl label{display:flex;justify-content:space-between;align-items:baseline;
 font-size:clamp(9.5px,1.05vh,11.5px);color:var(--dim);margin-bottom:5px}
.sl label b{font:600 clamp(10.5px,1.3vh,13px)/1 var(--mono);color:var(--fg)}
input[type=range]{width:100%;height:4px;-webkit-appearance:none;appearance:none;
 background:var(--line2);border-radius:99px;outline:none}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:15px;height:15px;
 border-radius:99px;background:var(--acc);cursor:pointer;border:2px solid var(--panel)}
input[type=range]::-moz-range-thumb{width:15px;height:15px;border-radius:99px;
 background:var(--acc);cursor:pointer;border:2px solid var(--panel)}
.presets{display:flex;gap:5px;margin-top:6px}
.presets button{flex:1;padding:5px 0;font:600 clamp(9px,1.05vh,11px)/1 inherit;
 border-radius:6px;background:var(--panel2);border:1px solid var(--line);
 color:var(--dim);cursor:pointer}
.presets button:hover{color:var(--fg);border-color:var(--line2)}
.presets button.on{background:rgba(77,159,255,.14);border-color:var(--acc);color:var(--acc)}
.bar{height:3px;background:var(--line);border-radius:99px;overflow:hidden;margin-top:6px}
.bar i{display:block;height:100%;background:var(--acc);width:0;transition:width .12s}
.hint{flex:none;padding:0 var(--g) calc(var(--g)*.85);font-size:clamp(9px,1.02vh,11px);
 color:var(--faint);line-height:1.55}
@media(max-height:620px){.hint{display:none}}
kbd{display:inline-block;min-width:16px;text-align:center;padding:1px 4px;
 background:var(--panel2);border:1px solid var(--line2);border-bottom-width:2px;
 border-radius:3px;font:600 9px/1.3 var(--mono);color:var(--dim)}

/* ---------- metrics ---------- */
.mets{flex:none;overflow:hidden}
.m{display:flex;align-items:baseline;justify-content:space-between;gap:8px;
 padding:calc(var(--g)*.42) 0;border-bottom:1px solid rgba(35,44,54,.55);
 font-size:var(--fs);overflow:hidden}
.m:last-child{border-bottom:0;padding-bottom:0}
.m:first-child{padding-top:0}
.m .k{color:var(--dim);overflow:hidden;text-overflow:ellipsis;white-space:nowrap;
 flex:1 1 auto;min-width:0}
/* flex-shrink 0 keeps the NUMBER intact; the label ellipsises instead. The
   value is the point of the row -- losing it to make room for its own caption
   is the wrong trade. */
.m .v{font:600 var(--fs)/1 var(--mono);white-space:nowrap;flex:0 0 auto}
.v.ok{color:var(--ok)}.v.warn{color:var(--warn)}.v.bad{color:var(--bad)}.v.dim{color:var(--faint)}
#radcard{flex:1.5}#spkcard{flex:.85}#odcard{flex:1.5}#dtcard{flex:1.15}
@media(max-height:700px){#spkcard{display:none}}
</style></head><body>

<header>
  <div class="brand"><span class="dot" id="live"></span><h1>Rover Control</h1></div>
  <div class="stats">
    <div class="chip" id="c-stack">stack <b>—</b></div>
    <div class="chip" id="c-mode">drive <b>—</b></div>
    <div class="chip" id="c-lat">lat <b>—</b></div>
    <div class="chip" id="c-esp">ESP32 <b>—</b></div>
    <div class="chip" id="c-lidar">lidar <b>—</b></div>
    <div class="chip" id="c-cam">cam <b>—</b></div>
    <div class="chip opt" id="c-wdog">wdog <b>—</b></div>
    <div class="chip opt" id="c-up">up <b>—</b></div>
  </div>
  <button id="hcancel" hidden>CANCEL GOAL</button>
  <div class="vsw" id="vsw">
    <button data-v="drive" class="on">DRIVE</button>
    <button data-v="map">MAP</button>
  </div>
  <button id="estop">EMERGENCY STOP</button>
</header>

<div class="grid" id="view-map" hidden>
  <div class="col">
    <div class="card" id="mapcard">
      <h2>Map <span class="tag" id="maptag">—</span></h2>
      <div class="mapwrap" id="mapwrap">
        <canvas id="mapcv"></canvas>
        <div class="mapkey">
          <span><i style="background:#e8eef5"></i>free</span>
          <span><i style="background:#05070a;outline:1px solid #46525f"></i>wall</span>
          <span><i style="background:#2c3744"></i>unknown</span>
          <span><i style="background:#f85149"></i>scan</span>
          <span><i style="background:#4d9fff"></i>plan</span>
          <span><i style="background:#a371f7"></i>demo point</span>
          <span><i style="background:#3fb950"></i>base</span>
        </div>
        <div class="maptools">
          <button id="mzin"  title="Zoom in">+</button>
          <button id="mzout" title="Zoom out">&minus;</button>
          <button id="mfit"  title="Fit map to window">⤢</button>
          <button id="mlock" title="Keep the rover centred" class="on">◎</button>
        </div>
        <div class="mapinfo">
          <span>pose <b id="mi-pose">—</b></span>
          <span>cursor <b id="mi-cur">—</b></span>
          <span>scale <b id="mi-scl">—</b></span>
        </div>
        <div class="mtools">
          <button id="t-pan" class="arm">PAN</button>
          <button id="t-goal">SET GOAL</button>
          <button id="t-pose">SET POSE</button>
          <button id="t-cancel" class="stop">CANCEL NAV</button>
        </div>
        <div class="mhint" id="mhint" hidden></div>
        <div class="mapempty" id="mapempty">
          <b>No map on /map</b>
          <span>Start a navigation or SLAM stack, then this fills in.</span>
        </div>
      </div>
      <div class="navbar">
        <span class="st" id="nav-st">IDLE</span>
        <span class="de" id="nav-de">arm a tool, then drag on the map to aim</span>
      </div>
    </div>
  </div>
  <div class="col">
    <div class="card">
      <h2>Rover mode <span class="tag" id="sttag">—</span></h2>
      <div class="body">
        <div class="modebar">
          <button id="m-nav">NAVIGATE<small>use a saved map</small></button>
          <button id="m-map" class="map">MAP ROOM<small>build a new map</small></button>
          <button id="m-idle" class="stop">STOP<small>shut the stack down</small></button>
        </div>
        <div class="mrow" id="row-map">
          <select id="mapsel"></select>
          <button id="camtog" title="The camera runs on its own, so it keeps
running across a mode change">CAMERA</button>
        </div>
        <div class="mrow" id="row-save" hidden>
          <input type="text" id="savename" placeholder="new map name"
                 maxlength="40" autocomplete="off" spellcheck="false">
          <button id="savemap">SAVE MAP</button>
        </div>
        <div class="mrow" id="row-use" hidden>
          <button id="usemap" style="flex:1;border-color:var(--acc);color:var(--acc)">
            USE THIS MAP &amp; NAVIGATE</button>
        </div>
        <div class="stnote" id="stnote">—</div>
      </div>
    </div>
    <div class="card">
      <h2>Localisation <span class="tag" id="loctag">—</span></h2>
      <div class="body"><div class="mets" id="locm"></div></div>
    </div>
    <div class="card" style="flex:1.35">
      <h2>Demo points <span class="tag" id="wptag">—</span></h2>
      <div class="body">
        <div class="wplist" id="wplist"></div>
        <div class="wpadd">
          <input id="wpname" placeholder="name this spot" maxlength="40"
                 autocomplete="off" spellcheck="false">
          <button id="wphere" title="Save the pose the rover is standing at now">
            SAVE HERE</button>
          <button id="wpplace" title="Drag the spot on the map instead">PLACE</button>
        </div>
        <div class="wpnote" id="wpnote">Drive to the spot, type a name, press SAVE HERE.</div>
      </div>
    </div>
  </div>
</div>

<div class="grid" id="view-drive">
  <div class="col">
    <div class="card" id="camcard">
      <h2>RealSense D455
        <span style="display:flex;align-items:center;gap:8px">
          <span class="tag" id="camtag">—</span>
          <span class="tabs" id="views">
            <button data-v="color" class="on">Colour</button>
            <button data-v="depth">Depth</button>
            <button data-v="blend">Overlay</button>
            <button data-v="ir">IR</button>
          </span>
        </span>
      </h2>
      <div class="vidwrap">
        <img id="feed" alt="">
        <div class="hud">
          <div class="tl">
            <div class="badge" id="hud-v">0.00 m/s</div>
            <div class="badge" id="hud-w">0.00 rad/s</div>
            <div class="badge" id="hud-d" style="display:none">— m</div>
          </div>
          <div class="ctr" id="hud-msg">waiting for camera…</div>
          <div class="ctr" id="hud-idle" hidden>
            <b>Nothing is running</b>
            <span>The rover's ROS stack has not been started, so the lidar,
                  ESP32 and camera are all off.</span>
            <button id="gomap">GO TO THE MAP TAB AND START IT</button>
          </div>
        </div>
        <div class="alert" id="alert"></div>
      </div>
      <div class="legend" id="legend">
        <span id="lg-min">0.3 m</span><span class="ramp"></span><span id="lg-max">6.0 m</span>
        <input type="range" id="dmax" min="15" max="100" value="40" step="5"
               title="far limit of the colour ramp"
               style="width:clamp(60px,9vw,120px);flex:none">
        <span style="color:var(--dim)" id="lg-note">near → far</span>
      </div>
    </div>

    <div class="card" id="drivecard">
      <h2>Drive <span class="tag" id="drivetag">hold to take manual control</span></h2>
      <div class="body">
        <div class="drive">
          <div class="dpad">
            <div></div>
            <div class="key" data-k="w"><span>▲</span><i>W</i></div>
            <div></div>
            <div class="key" data-k="a"><span>◀</span><i>A</i></div>
            <div class="key" data-k="s"><span>▼</span><i>S</i></div>
            <div class="key" data-k="d"><span>▶</span><i>D</i></div>
          </div>
          <div class="sliders">
            <div class="sl">
              <label>Linear speed <b id="spdv">0.20 m/s</b></label>
              <input type="range" id="spd" min="5" max="100" value="20">
              <div class="presets" id="pre">
                <button data-v="10">Slow</button>
                <button data-v="20" class="on">Normal</button>
                <button data-v="35">Fast</button>
              </div>
            </div>
            <div class="sl">
              <label>Turn rate <b id="trnv">0.60 rad/s</b></label>
              <input type="range" id="trn" min="10" max="100" value="60">
            </div>
            <div class="sl">
              <label>Throttle <b id="thr">idle</b></label>
              <div class="bar"><i id="thrbar"></i></div>
            </div>
          </div>
        </div>
      </div>
      <div class="hint">
        <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> drive ·
        <kbd>Shift</kbd> boost · <kbd>Space</kbd> e-stop ·
        <kbd>Esc</kbd> cancel goal · <kbd>M</kbd> map ·
        <kbd>1</kbd><kbd>2</kbd><kbd>3</kbd> presets —
        page must keep sending (0.6 s) and the ESP32 halts after 1.5 s of silence.
      </div>
    </div>
  </div>

  <div class="col">
    <div class="card" id="radcard">
      <h2>Obstacles <span class="tag" id="radtag">—</span></h2>
      <div class="body"><div class="canvbox"><canvas id="radar"></canvas></div></div>
    </div>
    <div class="card" id="spkcard">
      <h2>Velocity <span class="tag">cmd vs measured · 20 s</span></h2>
      <div class="body"><div class="canvbox"><canvas id="spark"></canvas></div></div>
    </div>
    <div class="card" id="odcard">
      <h2>Odometry <span class="tag" id="odtag">—</span></h2>
      <div class="body">
        <div class="canvbox"><canvas id="trace"></canvas></div>
        <div class="mets" id="od" style="margin-top:calc(var(--g)*.8)"></div>
      </div>
    </div>
    <div class="card" id="dtcard">
      <h2>Drivetrain &amp; link</h2>
      <div class="body"><div class="mets" id="dt"></div></div>
    </div>
  </div>
</div>

<script>
const $=s=>document.querySelector(s), TAU=Math.PI*2;
const BUILD='__BUILD__';   // substituted when the page is served
let keys={}, speed=.20, turn=.60, estop=false, boost=false, M=null;
const f=(x,n=2)=>(x==null||!isFinite(x))?'—':(+x).toFixed(n);
const row=(k,v,c='')=>`<div class="m"><span class="k">${k}</span><span class="v ${c}">${v}</span></div>`;

/* ---------- commands ---------- */
function cmdVec(){const m=boost?1.6:1;
  return {lin:((keys.w?1:0)-(keys.s?1:0))*speed*m,
          ang:((keys.a?1:0)-(keys.d?1:0))*turn*m};}
// Never let requests stack up. setInterval fires whether or not the previous
// request finished, so one slow response starts a backlog that only grows:
// the browser allows ~6 connections per host, the rest queue, and the measured
// latency climbs without limit. That is what "it works for a while then stops"
// looks like -- 5 s round trips to a server answering in 5 ms.
let sendBusy=false, lastSent=null;
async function send(force){
  if(sendBusy)return;
  const c=cmdVec();
  const zero=(c.lin===0&&c.ang===0);
  const same=lastSent&&lastSent.lin===c.lin&&lastSent.ang===c.ang;
  // Idle and already told it zero: say nothing. The node publishes NOTHING in
  // idle so Nav2 keeps the wheel, and its own deadman zeroed long ago. This is
  // 10 requests a second that bought nothing. A non-zero command is always
  // resent, because the deadman does need to keep hearing it.
  if(!force&&zero&&same)return;
  sendBusy=true;
  try{
    await fetch('/api/cmd',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify(c)});
    lastSent=c;
  }catch(e){}
  finally{sendBusy=false;}
}
setInterval(()=>send(false),100);
function setKey(k,on){
  if(keys[k]===on)return; keys[k]=on;
  document.querySelectorAll('[data-k="'+k+'"]').forEach(b=>b.classList.toggle('held',on));
  const c=cmdVec(), L=(M&&M.limits)||{lin:.35,ang:1};
  const mag=Math.min(1,Math.hypot(c.lin/L.lin,c.ang/L.ang));
  $('#thrbar').style.width=(mag*100)+'%';
  $('#thr').textContent=(c.lin||c.ang)?(f(c.lin)+' · '+f(c.ang)):'idle';
  send(true);}      // a key changed: send at once, even if it is the zero
const release=()=>['w','a','s','d'].forEach(k=>setKey(k,false));
// Typing must never drive. The name field also stops propagation, but the
// guard belongs here so any future input is safe without remembering to.
const typing=e=>{const t=e.target&&e.target.tagName;
  return t==='INPUT'||t==='TEXTAREA'||(e.target&&e.target.isContentEditable);};
addEventListener('keydown',e=>{
  if(e.repeat||typing(e))return;
  if(e.code==='Space'){e.preventDefault();toggleStop();return;}
  if(e.key==='Shift'){boost=true;return;}
  if('123'.includes(e.key)){setPreset([10,20,35][+e.key-1]);return;}
  const k=e.key.toLowerCase(); if('wasd'.includes(k)){e.preventDefault();setKey(k,true);}});
addEventListener('keyup',e=>{if(typing(e))return;
  if(e.key==='Shift'){boost=false;return;}
  const k=e.key.toLowerCase(); if('wasd'.includes(k))setKey(k,false);});
addEventListener('blur',release);
document.addEventListener('visibilitychange',()=>{if(document.hidden)release();});
document.querySelectorAll('[data-k]').forEach(b=>{const k=b.dataset.k;
  const on=e=>{e.preventDefault();setKey(k,true)}, off=e=>{e.preventDefault();setKey(k,false)};
  b.addEventListener('pointerdown',on);b.addEventListener('pointerup',off);
  b.addEventListener('pointerleave',off);b.addEventListener('pointercancel',off);});
function toggleStop(){estop=!estop;release();
  fetch('/api/estop',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({on:estop})}).catch(()=>{});
  const b=$('#estop');b.classList.toggle('armed',estop);
  b.textContent=estop?'STOPPED — RE-ARM':'EMERGENCY STOP';}
$('#estop').onclick=toggleStop;
function setPreset(v){$('#spd').value=v;speed=v/100;$('#spdv').textContent=speed.toFixed(2)+' m/s';
  document.querySelectorAll('#pre button').forEach(b=>b.classList.toggle('on',+b.dataset.v===v));}
$('#spd').oninput=e=>{speed=e.target.value/100;$('#spdv').textContent=speed.toFixed(2)+' m/s';
  document.querySelectorAll('#pre button').forEach(b=>b.classList.remove('on'));};
$('#trn').oninput=e=>{turn=e.target.value/100;$('#trnv').textContent=turn.toFixed(2)+' rad/s';};
document.querySelectorAll('#pre button').forEach(b=>b.onclick=()=>setPreset(+b.dataset.v));

/* ---------- canvas sizing: always match the container box ---------- */
function fit(c){
  const box=c.parentElement.getBoundingClientRect(), d=devicePixelRatio||1;
  const w=Math.max(30,box.width), h=Math.max(30,box.height);
  if(c.width!==Math.round(w*d)||c.height!==Math.round(h*d)){
    c.width=Math.round(w*d);c.height=Math.round(h*d);}
  const g=c.getContext('2d');g.setTransform(d,0,0,d,0,0);g.clearRect(0,0,w,h);
  return [g,w,h];}

function radar(pts,near,sel='#radar',tag='#radtag'){
  const [g,W,H]=fit($(sel));
  const cx=W/2, cy=H*0.93, R=Math.min(W/2-14,H*0.87), MAX=4.0;
  g.lineWidth=1;g.strokeStyle='#232c36';
  g.font='9px ui-monospace,monospace';
  for(let m=1;m<=4;m++){const r=R*m/MAX;
    g.beginPath();g.arc(cx,cy,r,Math.PI,TAU);g.stroke();
    g.fillStyle='#5c6b7a';g.fillText(m+'m',cx+3,cy-r+10);}
  g.beginPath();g.moveTo(cx,cy);g.lineTo(cx,cy-R);g.stroke();
  for(const d of [-60,-30,30,60]){const a=-Math.PI/2+d*Math.PI/180;
    g.beginPath();g.moveTo(cx,cy);g.lineTo(cx+Math.cos(a)*R,cy+Math.sin(a)*R);g.stroke();}
  g.fillStyle='rgba(77,159,255,.06)';
  g.beginPath();g.moveTo(cx,cy);g.arc(cx,cy,R,-Math.PI/2-.52,-Math.PI/2+.52);g.fill();
  const dotR=Math.max(1.3,Math.min(2.2,R/90));
  for(const p of (pts||[])){const rng=p[1]; if(rng>MAX)continue;
    const a=-Math.PI/2+p[0]*Math.PI/180, r=R*rng/MAX;
    g.fillStyle=rng<.5?'#f85149':rng<1?'#e3b341':'#3fb950';
    g.globalAlpha=rng<1?1:.72;
    g.beginPath();g.arc(cx+Math.cos(a)*r,cy+Math.sin(a)*r,dotR,0,TAU);g.fill();}
  g.globalAlpha=1;g.fillStyle='#4d9fff';
  g.beginPath();g.moveTo(cx,cy-7);g.lineTo(cx-5,cy+4);g.lineTo(cx+5,cy+4);g.closePath();g.fill();
  $(tag).textContent=near==null?'clear':f(near)+' m';}

function spark(h){
  const [g,W,H]=fit($('#spark')); if(!h||h.length<2)return;
  const lim=Math.max(.2,...h.map(p=>Math.max(Math.abs(p[1]),Math.abs(p[2]))))*1.15;
  const X=a=>W-(a/20)*W, Y=v=>H/2-(v/lim)*(H/2-5);
  g.strokeStyle='#232c36';g.lineWidth=1;
  g.beginPath();g.moveTo(0,H/2);g.lineTo(W,H/2);g.stroke();
  const line=(i,col,dash)=>{g.strokeStyle=col;g.lineWidth=1.6;g.setLineDash(dash);g.beginPath();
    h.forEach((p,n)=>{const x=X(p[0]),y=Y(p[i]);n?g.lineTo(x,y):g.moveTo(x,y)});
    g.stroke();g.setLineDash([]);};
  line(1,'#4d9fff',[3,3]); line(2,'#3fb950',[]);
  g.font='9px ui-monospace,monospace';
  g.fillStyle='#4d9fff';g.fillText('cmd',5,10);
  g.fillStyle='#3fb950';g.fillText('meas',5,20);
  g.fillStyle='#5c6b7a';g.fillText('±'+lim.toFixed(2),W-42,10);}

function trace(path,od){
  const [g,W,H]=fit($('#trace'));
  if(!path||path.length<2){g.fillStyle='#5c6b7a';g.font='11px sans-serif';
    g.fillText('no motion yet',8,H/2);return;}
  const xs=path.map(p=>p[0]),ys=path.map(p=>p[1]);
  const sp=Math.max(Math.max(...xs)-Math.min(...xs),Math.max(...ys)-Math.min(...ys),.5)*1.2;
  const mx=(Math.min(...xs)+Math.max(...xs))/2, my=(Math.min(...ys)+Math.max(...ys))/2;
  const S=Math.min(W,H)*.84;
  const TX=x=>W/2+((x-mx)/sp)*S, TY=y=>H/2-((y-my)/sp)*S;
  g.strokeStyle='#2e3a46';g.lineWidth=1;g.beginPath();
  g.moveTo(0,H/2);g.lineTo(W,H/2);g.moveTo(W/2,0);g.lineTo(W/2,H);g.stroke();
  g.strokeStyle='#4d9fff';g.lineWidth=1.8;g.beginPath();
  path.forEach((p,i)=>{const x=TX(p[0]),y=TY(p[1]);i?g.lineTo(x,y):g.moveTo(x,y)});g.stroke();
  g.fillStyle='#8998a8';g.beginPath();g.arc(TX(path[0][0]),TY(path[0][1]),3,0,TAU);g.fill();
  const last=path[path.length-1], yaw=((od&&od.yaw)||0)*Math.PI/180;
  g.save();g.translate(TX(last[0]),TY(last[1]));g.rotate(-yaw+Math.PI/2);
  g.fillStyle='#3fb950';g.beginPath();g.moveTo(0,-8);g.lineTo(-5.5,5);g.lineTo(5.5,5);
  g.closePath();g.fill();g.restore();
  g.fillStyle='#5c6b7a';g.font='9px ui-monospace,monospace';
  g.fillText(sp.toFixed(1)+' m',5,H-5);}

/* ---------- poll ---------- */
function chip(id,cls,txt){const e=$(id);e.className='chip '+cls+(id.includes('wdog')||id.includes('up')?' opt':'');
  e.querySelector('b').textContent=txt;}
function draw(){if(!M)return;
  if($('#radar').offsetParent){radar(M.scan,M.lidar.nearest_fwd);
    if($('#spkcard').offsetParent)spark(M.vhist);
    trace(M.path,M.odom);}
}

async function poll(){
  const t0=performance.now();
  try{M=await (await fetch('/api/metrics')).json();}
  catch(e){$('#live').className='dot off';chip('#c-lat','bad','off');return;}
  // A tab left open across a rebuild would keep running the old JavaScript,
  // which looks exactly like a feature that does not work. Reload instead.
  if(M.build&&BUILD!=='__BUILD__'&&M.build!==BUILD){location.reload();return;}
  const lat=Math.round(performance.now()-t0);
  $('#live').className='dot';
  const m=M;
  const md=m.mode||'idle';
  // The single most important thing on the page: is anything running at all?
  // Without this, "ESP32 off / lidar off / cam off" looks like broken hardware
  // when it only means no stack has been started -- and the DRIVE tab gave no
  // way to tell, because the chip beside it is about who holds the wheel.
  const ss=(m.stack&&m.stack.state)||'idle';
  chip('#c-stack',
       ss==='navigation'||ss==='mapping' ? 'ok'
       : ss==='starting'||ss==='stopping' ? 'warn'
       : ss==='external' ? 'warn' : 'bad',
       ss==='navigation' ? 'NAV' : ss==='mapping' ? 'SLAM'
       : ss==='idle' ? 'NOT RUNNING' : ss.toUpperCase());
  chip('#c-mode', md==='manual'?'warn':md==='estop'?'bad':'ok',
       md==='manual'?'MANUAL':md==='estop'?'E-STOP':'AUTO');
  const nothing=(ss==='idle');
  $('#hud-idle').classList.toggle('show', nothing);
  $('#hud-msg').style.display = nothing ? 'none' : '';
  chip('#c-lat',lat<150?'ok':lat<400?'warn':'bad',lat+'ms');
  chip('#c-esp',m.diag.ok?'ok':'bad',m.diag.ok?'on':'off');
  chip('#c-lidar',m.lidar.ok?'ok':'bad',m.lidar.ok?f(m.lidar.hz,0)+'Hz':'off');
  chip('#c-cam',m.camera.ok?'ok':'bad',m.camera.ok?f(m.camera.hz,0)+'fps':'off');
  chip('#c-wdog',m.diag.wdog>0?'warn':'ok',f(m.diag.wdog,0));
  const u=m.uptime|0;chip('#c-up','',(u/60|0)+'m'+(u%60)+'s');

  $('#hud-v').textContent=f(m.odom.v)+' m/s';
  $('#hud-w').textContent=f(m.odom.w)+' rad/s';
  const cm=m.camera.centre_m;
  const hd=$('#hud-d');
  if(cm!=null){hd.style.display='';hd.textContent='centre '+f(cm)+' m';}
  else hd.style.display='none';
  if(m.camera.depth_range && document.activeElement!==$('#dmax')){
    $('#lg-min').textContent=m.camera.depth_range[0].toFixed(1)+' m';
    $('#lg-max').textContent=m.camera.depth_range[1].toFixed(1)+' m';
    $('#dmax').value=Math.round(m.camera.depth_range[1]*10);}
  const av=m.camera.views||{};
  document.querySelectorAll('#views button').forEach(b=>{
    const v=b.dataset.v;
    b.disabled = (v!=='color') && (v!==view) && (av[v]===false) && !m.camera.ok;});
  $('#hud-msg').style.display=(m.camera.ok&&av[view]!==false)?'none':'block';
  $('#camtag').textContent=m.camera.ok?f(m.camera.hz,0)+' fps':'no signal';

  const n=m.lidar.nearest_fwd, A=$('#alert');
  if(n!=null&&n<.5){A.className='alert show red';A.textContent='⚠  OBSTACLE '+f(n)+' m AHEAD';}
  else if(n!=null&&n<1){A.className='alert show amber';A.textContent='obstacle '+f(n)+' m ahead';}
  else A.className='alert';

  draw();
  $('#drivetag').textContent = md==='manual'
      ? 'MANUAL — you have the wheel'
      : md==='estop' ? 'E-STOP asserted'
      : 'AUTO — Nav2 has the wheel · hold a key to take over';
  $('#odtag').textContent=f(m.odom.path,1)+' m';
  $('#od').innerHTML=
    row('Position','x '+f(m.odom.x)+'  y '+f(m.odom.y))+
    row('Heading',f(m.odom.yaw,1)+'°')+
    row('Path length',f(m.odom.path)+' m')+
    row('Peak speed',f(m.max_v)+' m/s');
  const fa=m.ticks.fault;
  $('#dt').innerHTML=
    row('cmd_vel received',f(m.diag.cmd_rx,0),m.diag.ok?'ok':'bad')+
    row('Watchdog stops',f(m.diag.wdog,0),m.diag.wdog>0?'warn':'ok')+
    row('Commanded RPM (L)',f(m.diag.rpm_L,0))+
    row('Ticks L / R',f(m.ticks.L,0)+' / '+f(m.ticks.R,0),'dim')+
    row('Encoder faults',fa==0?'none':'flag '+f(fa,0),fa==0?'ok':'bad')+
    row('Lidar valid',f(m.lidar.valid_pct,0)+'%',m.lidar.valid_pct>50?'ok':'warn')+
    row('E-stop',m.estop?'ENGAGED':'clear',m.estop?'bad':'ok');
  mapSync();
}
// Self-scheduling rather than setInterval, for the same reason as send():
// a slow response must delay the next poll, not queue behind it.
let pollBusy=false;
async function pollLoop(){
  if(!pollBusy){
    pollBusy=true;
    try{await poll();}finally{pollBusy=false;}
  }
  setTimeout(pollLoop,300);
}
pollLoop();
new ResizeObserver(draw).observe(document.body);

/* ================= MAP VIEW =================
   World metres -> screen pixels. `cam` is the world point under the centre of
   the viewport and `scale` is pixels per metre, so panning and zooming are
   just two numbers and every overlay (robot, scan, plan) uses the same pair.
   The map frame's +y is UP, the canvas's +y is DOWN, hence the flip in sy().  */
const MAPCV=$('#mapcv'), MAPWRAP=$('#mapwrap');
let mapImg=null, mapSeq=-1, mapMeta=null, mapScale=28, mapCam={x:0,y:0},
    mapFollow=true, mapFitted=false, mapCur=null, ROBOT_R=0.33;

// Viewport size in CSS pixels, refreshed once per frame and once per pointer
// event -- never read per point, because getBoundingClientRect forces a layout.
// It must come from the WRAPPER, which is also what pointer coordinates are
// measured against. Reading it off the canvas instead was the bug that put
// every click near the middle of the map: a canvas sized only by inset:0
// reports its pixel-buffer width, which fit() sets to CSS width x DPR.
let MW=1, MH=1;
function msize(){
  const r=MAPWRAP.getBoundingClientRect();
  MW=r.width||1; MH=r.height||1; return r;}

const sx=x=>(x-mapCam.x)*mapScale+MW/2;
const sy=y=>MH/2-(y-mapCam.y)*mapScale;
const wx=p=>(p-MW/2)/mapScale+mapCam.x;
const wy=p=>mapCam.y-(p-MH/2)/mapScale;

function mapFit(){
  // Refuse to fit while the panel is hidden: it measures 0 there and the
  // resulting scale would be nonsense that never gets recomputed.
  msize();
  if(!mapMeta||MW<2||MH<2)return;
  const W=MW, H=MH;
  const ew=mapMeta.w*mapMeta.res, eh=mapMeta.h*mapMeta.res;
  mapScale=Math.max(2,Math.min(W/ew,H/eh)*0.94);
  mapCam={x:mapMeta.ox+ew/2, y:mapMeta.oy+eh/2};
  mapFollow=false;$('#mlock').classList.remove('on');mapFitted=true;}

function mapZoom(k,px,py){
  // Zoom about a fixed point so the map does not slide out from under the
  // cursor. Without this, zooming in on a doorway loses the doorway.
  msize();
  const W=MW, H=MH;
  px=(px==null)?W/2:px; py=(py==null)?H/2:py;
  const bx=wx(px), by=wy(py);
  mapScale=Math.max(2,Math.min(600,mapScale*k));
  mapCam.x=bx-(px-W/2)/mapScale; mapCam.y=by+(py-H/2)/mapScale;
  mapFollow=false;$('#mlock').classList.remove('on');}

function drawMap(){
  const [g,W,H]=fit(MAPCV);
  MW=W; MH=H;
  if(!mapMeta||!mapImg){return;}
  if(mapFollow&&M&&M.map_pose)mapCam={x:M.map_pose.x,y:M.map_pose.y};

  const ew=mapMeta.w*mapMeta.res, eh=mapMeta.h*mapMeta.res;
  // Nearest-neighbour once a cell is bigger than a pixel: a blurred occupancy
  // grid hides exactly the thin walls and gaps you are zooming in to judge.
  g.imageSmoothingEnabled = mapScale*mapMeta.res < 1.2;
  g.drawImage(mapImg, sx(mapMeta.ox), sy(mapMeta.oy+eh), ew*mapScale, eh*mapScale);

  if(mapScale>=14){                       // 1 m grid, only when it is readable
    g.strokeStyle='rgba(93,107,122,.20)';g.lineWidth=1;g.beginPath();
    const x0=Math.floor(wx(0)), x1=Math.ceil(wx(W)), y0=Math.floor(wy(H)), y1=Math.ceil(wy(0));
    if(x1-x0<400){for(let x=x0;x<=x1;x++){g.moveTo(sx(x),0);g.lineTo(sx(x),H);}
                  for(let y=y0;y<=y1;y++){g.moveTo(0,sy(y));g.lineTo(W,sy(y));}}
    g.stroke();}

  for(const n in WP.points){                // saved demo points
    const p=WP.points[n], X=sx(p.x), Y=sy(p.y), base=isBase(n);
    if(X<-40||Y<-40||X>W+40||Y>H+40)continue;
    const c=base?'#3fb950':'#a371f7';
    g.strokeStyle=c;g.fillStyle=c;g.lineWidth=1.6;
    g.beginPath();g.arc(X,Y,6,0,TAU);g.stroke();
    g.beginPath();g.arc(X,Y,2.4,0,TAU);g.fill();
    const a=-p.yaw*Math.PI/180;
    g.beginPath();g.moveTo(X+Math.cos(a)*6,Y+Math.sin(a)*6);
    g.lineTo(X+Math.cos(a)*14,Y+Math.sin(a)*14);g.stroke();
    g.font='600 11px -apple-system,system-ui,sans-serif';
    const tw=g.measureText(n).width;
    g.fillStyle='rgba(7,10,13,.78)';
    g.fillRect(X+9,Y-16,tw+8,14);
    g.fillStyle=c;g.fillText(n,X+13,Y-5);}

  const P=M&&M.map_pose;
  if(P){
    const th=P.yaw*Math.PI/180;
    if(M.plan&&M.plan.length>1){         // Nav2 path
      g.strokeStyle='#4d9fff';g.lineWidth=2.5;g.lineJoin='round';g.beginPath();
      M.plan.forEach((p,i)=>i?g.lineTo(sx(p[0]),sy(p[1])):g.moveTo(sx(p[0]),sy(p[1])));
      g.stroke();
      const e=M.plan[M.plan.length-1];
      g.fillStyle='#4d9fff';g.beginPath();g.arc(sx(e[0]),sy(e[1]),5,0,TAU);g.fill();}
    if(M.scan){                          // live returns, robot frame -> world
      g.fillStyle='#f85149';
      const r=Math.max(1,Math.min(2.6,mapScale/22));
      for(const p of M.scan){const a=th+p[0]*Math.PI/180;
        g.beginPath();g.arc(sx(P.x+Math.cos(a)*p[1]),sy(P.y+Math.sin(a)*p[1]),r,0,TAU);g.fill();}}
    const R=ROBOT_R*mapScale;            // footprint, to scale
    g.strokeStyle='rgba(77,159,255,.75)';g.fillStyle='rgba(77,159,255,.16)';g.lineWidth=1.5;
    g.beginPath();g.arc(sx(P.x),sy(P.y),R,0,TAU);g.fill();g.stroke();
    g.strokeStyle='#4d9fff';g.lineWidth=2;g.beginPath();
    g.moveTo(sx(P.x),sy(P.y));
    g.lineTo(sx(P.x+Math.cos(th)*ROBOT_R*1.5),sy(P.y+Math.sin(th)*ROBOT_R*1.5));g.stroke();
    g.fillStyle='#4d9fff';g.beginPath();g.arc(sx(P.x),sy(P.y),3,0,TAU);g.fill();}

  if(aim){                                 // live gesture arrow
    const c=(tool==='pose')?'#e3b341':(tool==='place')?'#a371f7':'#3fb950';
    const dx=aim.x1-aim.x0, dy=aim.y1-aim.y0, L=Math.hypot(dx,dy)*mapScale;
    g.strokeStyle=c;g.fillStyle=c;g.lineWidth=2;
    g.beginPath();g.arc(sx(aim.x0),sy(aim.y0),ROBOT_R*mapScale,0,TAU);g.stroke();
    if(L>6){const a=Math.atan2(dy,dx);
      const ex=sx(aim.x0)+Math.cos(a)*L, ey=sy(aim.y0)-Math.sin(a)*L;
      g.beginPath();g.moveTo(sx(aim.x0),sy(aim.y0));g.lineTo(ex,ey);g.stroke();
      g.beginPath();g.moveTo(ex,ey);
      g.lineTo(ex-Math.cos(a-.4)*11,ey+Math.sin(a-.4)*11);
      g.lineTo(ex-Math.cos(a+.4)*11,ey+Math.sin(a+.4)*11);g.closePath();g.fill();}}

  $('#mi-pose').textContent=P?`${f(P.x)}, ${f(P.y)} · ${Math.round(P.yaw)}°`:'not localised';
  $('#mi-cur').textContent=mapCur?`${f(mapCur.x)}, ${f(mapCur.y)}`:'—';
  $('#mi-scl').textContent=`${Math.round(mapScale)} px/m`;
}

function mapSync(){                       // called from poll()
  const mi=M&&M.map; if(!mi)return;
  $('#mapempty').style.display=mi.ready?'none':'flex';
  if(mi.ready&&mi.seq!==mapSeq){
    mapSeq=mi.seq; mapMeta=mi.meta;
    const im=new Image();
    im.onload=()=>{mapImg=im; if(!mapFitted)mapFit();};
    im.src='/api/map.png?s='+mi.seq;      // seq busts the cache on a new map
    $('#maptag').textContent=
      `${(mi.meta.w*mi.meta.res).toFixed(1)} × ${(mi.meta.h*mi.meta.res).toFixed(1)} m `+
      `· ${mi.meta.res.toFixed(2)} m/cell`;}
  if(M.stack){ST=M.stack; stRender(M.stack,M.maps||[],M.save,M.cam_proc);}
  if(M.waypoints&&JSON.stringify(M.waypoints)!==JSON.stringify(WP)){
    WP=M.waypoints; wpRender();}
  const nv=M.nav||{state:'idle'};
  $('#hcancel').hidden=!['active','sending','cancelling'].includes(nv.state);
  $('#hcancel').textContent=nv.state==='cancelling'?'CANCELLING…':'CANCEL GOAL';
  const cls={active:'var(--acc)',sending:'var(--acc)',arrived:'var(--ok)',
             aborted:'var(--bad)',rejected:'var(--bad)',failed:'var(--bad)',
             'no server':'var(--bad)',cancelling:'var(--warn)'}[nv.state]||'var(--dim)';
  $('#nav-st').textContent=nv.state.toUpperCase();
  $('#nav-st').style.color=cls;
  $('#nav-de').textContent = nv.state==='active'
      ? `to ${f(nv.goal&&nv.goal[0])}, ${f(nv.goal&&nv.goal[1])}`+
        (nv.remaining!=null?` · ${f(nv.remaining)} m remaining`:'')
    : nv.result ? nv.result
    : 'arm a tool, then drag on the map to aim';
  if(!M.map_pose)$('#loctag').textContent='no map→base TF';
  else $('#loctag').textContent='localised';
  $('#locm').innerHTML=
    row('Frame','map → base_footprint',M.map_pose?'ok':'bad')+
    (M.map_pose?row('x',f(M.map_pose.x)+' m')+row('y',f(M.map_pose.y)+' m')+
                row('heading',Math.round(M.map_pose.yaw)+'°'):
                row('Status','waiting for AMCL / SLAM','warn'))+
    row('Plan',M.plan&&M.plan.length?M.plan.length+' poses':'none',
        M.plan&&M.plan.length?'ok':'')+
    row('Footprint',ROBOT_R.toFixed(2)+' m radius');
}

/* ---- tools -------------------------------------------------------
   A goal makes the rover DRIVE, so it can never be a stray click while
   panning. Arm a tool first; the next press-drag-release on the map defines
   the position and the heading, then the tool disarms itself. Same gesture as
   RViz, but it cannot fire by accident. */
let tool='pan', aim=null, WP={points:{},map:'',current_map:''}, wpPending=null;
function setTool(t){
  tool=t;
  $('#t-pan').classList.toggle('arm',t==='pan');
  $('#t-goal').classList.toggle('arm',t==='goal');
  $('#t-pose').classList.toggle('arm',t==='pose');
  $('#t-pose').classList.toggle('warnarm',t==='pose');
  if(t!=='place')wpPending=null;
  const h=$('#mhint');
  h.hidden=(t==='pan');
  h.className='mhint'+(t==='pose'?' warn':'');
  h.textContent = t==='goal' ? 'drag on the map: where to go, and which way to face'
                : t==='pose' ? 'drag on the map: where the rover actually is, and its heading'
                : t==='place'? `drag on the map: where "${wpPending}" is, and which way to face`
                : '';
  MAPWRAP.style.cursor = t==='pan' ? 'grab' : 'crosshair';}
$('#t-pan').onclick =()=>setTool('pan');
$('#t-goal').onclick=()=>setTool(tool==='goal'?'pan':'goal');
$('#t-pose').onclick=()=>setTool(tool==='pose'?'pan':'pose');
const navLive=()=>!!(M&&M.nav&&['active','sending','cancelling'].includes(M.nav.state));
async function cancelNav(){
  // Always say something. A button that silently does nothing when there is no
  // goal to cancel is indistinguishable from a broken button.
  const act=navLive();
  await fetch('/api/nav_cancel',{method:'POST',body:'{}'});
  $('#nav-de').textContent=act?'cancelling…':'nothing to cancel — no goal is running';
}
$('#t-cancel').onclick=cancelNav;
$('#hcancel').onclick=cancelNav;
// Escape is the reflex when a robot is heading somewhere you did not intend.
addEventListener('keydown',e=>{
  if(typing(e))return;
  if(e.key==='Escape'){e.preventDefault();cancelNav();}});

/* ---- mode / stack control ----
   One stack at a time is enforced on the server; the UI mirrors that by
   disabling both start buttons whenever anything is up, including a stack the
   operator launched from a terminal. */
let ST={state:'idle',nodes:[]}, MAPS=[], lastSaved='';

function stRender(st,maps,save,cam){
  const s=st.state, busy=(s==='starting'||s==='stopping');
  const idle=(s==='idle');
  const c=(cam&&cam.state)||'off';
  const cb=$('#camtog');
  cb.classList.toggle('on',c==='on');
  const usbBad = cam && cam.usb_mbps && !cam.usb_ok;
  cb.classList.toggle('warn', c==='starting'||c==='external'||!!usbBad);
  cb.textContent = c==='on'?(usbBad?'CAMERA · USB2':'CAMERA ON')
                 : c==='starting'?'CAMERA…'
                 : c==='external'?'CAMERA (EXT)':'CAMERA OFF';
  cb.disabled=(c==='external');
  cb.title = usbBad
    ? `The camera negotiated a USB 2 link (${cam.usb_mbps} Mbit/s) instead of `+
      `USB 3. It runs at about a quarter rate. Reseat the plug firmly or use a `+
      `USB 3 cable.`
    : 'The camera runs on its own, so it keeps running across a mode change';
  $('#sttag').textContent = s.toUpperCase()+(st.ready?' · '+st.ready:'');
  $('#sttag').style.color = {navigation:'var(--acc)',mapping:'#a371f7',
    starting:'var(--warn)',stopping:'var(--warn)',external:'var(--warn)'
    }[s]||'var(--dim)';
  $('#m-nav').classList.toggle('on',s==='navigation');
  $('#m-map').classList.toggle('on',s==='mapping');
  $('#m-idle').classList.toggle('on',false);
  // Switching modes is allowed while a stack is up: it stops the old one and
  // starts the new one in a single action, so the rover is never left dark.
  const sameMap = s==='navigation' &&
                  (st.opts&&st.opts.map)===$('#mapsel').value;
  $('#m-nav').disabled=busy||s==='external'||sameMap;
  $('#m-map').disabled=busy||s==='external'||s==='mapping';
  $('#m-nav').textContent = s==='mapping' ? 'SWITCH TO NAVIGATE'
                          : s==='navigation' ? 'LOAD THIS MAP' : 'NAVIGATE';
  $('#m-idle').disabled=idle||s==='external'||busy;
  $('#mapsel').disabled=busy;
  $('#row-save').hidden=(s!=='mapping');
  $('#row-use').hidden=!(s==='mapping'&&save&&save.path);
  $('#savemap').disabled=!!(save&&save.busy);

  if(JSON.stringify(maps.map(m=>m.name))!==JSON.stringify(MAPS.map(m=>m.name))){
    MAPS=maps;
    const cur=$('#mapsel').value;
    $('#mapsel').innerHTML=maps.length
      ? maps.map(m=>`<option value="${esc(m.path)}">${esc(m.name)}</option>`).join('')
      : '<option value="">no maps found</option>';
    if(cur&&maps.some(m=>m.path===cur))$('#mapsel').value=cur;
  }
  // Point the picker at a map that was just saved, so NAVIGATE means the map
  // you have this second rather than whatever was selected beforehand.
  if(save&&save.path&&save.path!==lastSaved){
    lastSaved=save.path;
    if(maps.some(m=>m.path===save.path))$('#mapsel').value=save.path;
  }
  const n=$('#stnote');
  let cls='',txt;
  if(s==='external'){cls='bad';
    txt='A stack was started outside this console: <b>'+
        esc(st.nodes.slice(0,5).join(', '))+'</b>. Stop it in its own terminal '+
        'before switching modes here.';}
  else if(save&&save.msg&&(Date.now()/1000-(save.at||0))<20){
    cls=/could not|must|no map/.test(save.msg)?'bad':'ok';
    txt=esc(save.msg);}
  else if(usbBad)
    txt='Camera is on a <b>USB 2</b> link ('+cam.usb_mbps+' Mbit/s), so it runs '+
        'at about a quarter rate and the depth obstacle layer with it. Reseat '+
        'the plug firmly, or use a USB 3 cable.';
  else if(s==='mapping')txt='Drive the room with <b>WASD</b> on the Drive tab, '+
    'then name and save the map.';
  else if(s==='navigation')txt='Map <b>'+esc((st.opts&&st.opts.map||'').split("/").pop()
    .replace(/\.yaml$/,''))+'</b> loaded. Set the pose, then send goals.';
  else if(s==='starting'||s==='stopping')txt='Working… <b>'+esc(st.ready||'')+'</b>';
  else txt=st.detail?esc(st.detail)+'. Pick a map and press NAVIGATE, or MAP ROOM '+
    'to survey a new one.':'Pick a map and press NAVIGATE, or MAP ROOM to survey a new one.';
  n.className='stnote '+cls; n.innerHTML=txt;
}

async function setMode(mode,mapPath){
  const body={mode:mode};
  if(mode==='navigation')body.map=mapPath||$('#mapsel').value;
  const live=ST.state==='mapping'||ST.state==='navigation';
  if(mode==='idle'){
    if(!confirm('Stop everything?\n\nThe lidar and ESP32 go dark. To change '+
                'mode instead, press the other mode button — that keeps '+
                'the rover running.'))return;
  }else if(live){
    // One action: stop the old stack, start the new one. "Leave mapping"
    // should never land on a dark rover.
    body.switch=true;
    if(ST.state==='navigation'&&mode==='navigation'&&
       !confirm('Reload navigation with this map?\n\nThe stack restarts and '+
                'the pose estimate is lost, so set the pose again after.'))return;
    if(ST.state==='mapping'&&!confirm('Leave mapping?\n\nAnything not saved '+
       'with SAVE MAP is lost. The rover stays powered and comes straight up '+
       'in navigation.'))return;
  }
  $('#stnote').className='stnote'; $('#stnote').textContent='working…';
  const r=await fetch('/api/mode',{method:'POST',body:JSON.stringify(body)});
  const j=await r.json();
  if(!j.ok){$('#stnote').className='stnote bad';$('#stnote').textContent=j.err||'failed';}
}
$('#m-nav').onclick =()=>setMode('navigation');
$('#m-map').onclick =()=>setMode('mapping');
$('#m-idle').onclick=()=>setMode('idle');
$('#usemap').onclick=()=>{
  // save.path is in-memory, so a console restart between SAVE MAP and this
  // press would empty it. The dropdown still knows which map is selected.
  const p=(M&&M.save&&M.save.path)||$('#mapsel').value;
  if(p)setMode('navigation',p);
  else{$('#stnote').className='stnote bad';
       $('#stnote').textContent='Save the map first, or pick one from the list.';}};
$('#camtog').onclick=async()=>{
  const on=!(M&&M.cam_proc&&M.cam_proc.state==='on');
  $('#camtog').disabled=true;
  await fetch('/api/camera',{method:'POST',body:JSON.stringify({on:on})});};
$('#savemap').onclick=()=>{
  const n=$('#savename').value.trim();
  if(!n){$('#savename').focus();return;}
  post('/api/save_map',{name:n});};
$('#savename').addEventListener('keydown',e=>{
  e.stopPropagation(); if(e.key==='Enter')$('#savemap').click();});

/* ---- demo points ---- */
const esc=t=>String(t).replace(/[&<>"']/g,c=>
  ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const isBase=n=>/^(home|base)$/i.test(n);

function wpRender(){
  const names=Object.keys(WP.points).sort((a,b)=>
    (isBase(b)-isBase(a))||a.localeCompare(b));
  $('#wptag').textContent=names.length?names.length+' saved':'none yet';
  $('#wplist').innerHTML = names.length ? names.map(n=>{
    const p=WP.points[n];
    return `<div class="wp${isBase(n)?' base':''}">
      <span class="nm">${esc(n)}</span>
      <span class="co">${p.x.toFixed(2)}, ${p.y.toFixed(2)}</span>
      <button class="at" data-at="${esc(n)}"
        title="The rover is standing here right now — set its pose from this point"
        >I'M HERE</button>
      <button class="go" data-go="${esc(n)}">GO</button>
      <button class="rm" data-rm="${esc(n)}">✕</button></div>`;}).join('')
    : `<div class="wpempty">No demo points yet.<br>
       Name a spot below and save it.</div>`;
  $('#wplist').querySelectorAll('[data-go]').forEach(b=>b.onclick=()=>
    post('/api/wp/goto',{name:b.dataset.go}));
  $('#wplist').querySelectorAll('[data-at]').forEach(b=>b.onclick=async()=>{
    const j=await post('/api/wp/localise',{name:b.dataset.at});
    if(j.ok){$('#wpnote').className='wpnote ok';
      $('#wpnote').textContent='Pose set from "'+b.dataset.at+
        '". Check the scan points land on the walls.';}});
  $('#wplist').querySelectorAll('[data-rm]').forEach(b=>b.onclick=()=>{
    if(confirm('Delete demo point "'+b.dataset.rm+'"?'))
      post('/api/wp/delete',{name:b.dataset.rm});});
  // Points captured on a different map are in the wrong coordinate frame.
  const wrong = WP.map && WP.current_map && WP.map!==WP.current_map;
  const note=$('#wpnote');
  if(!wrong && /^(Saved|Type a name)/.test(note.textContent))return;  // keep feedback
  note.className='wpnote'+(wrong?' bad':'');
  note.textContent = wrong
    ? `Saved against "${WP.map}" but "${WP.current_map}" is loaded — these
       coordinates do not apply.`.replace(/\s+/g,' ')
    : 'Drive to the spot, type a name, press SAVE HERE.';
}

async function post(url,body){
  try{
    const r=await fetch(url,{method:'POST',body:JSON.stringify(body||{})});
    const j=await r.json();
    if(!j.ok){$('#wpnote').className='wpnote bad';
              $('#wpnote').textContent=j.err||'failed';}
    return j;
  }catch(e){return {ok:false};}
}
function wpNeedName(){
  $('#wpnote').className='wpnote bad';
  $('#wpnote').textContent='Type a name for the spot first.';
  $('#wpname').focus();}
$('#wphere').onclick=async()=>{
  const n=$('#wpname').value.trim();
  if(!n)return wpNeedName();
  const j=await post('/api/wp/save',{name:n,here:true});
  if(j.ok){$('#wpname').value='';
           $('#wpnote').className='wpnote ok';
           $('#wpnote').textContent='Saved "'+n+'" at the rover\u2019s position.';}};
$('#wpplace').onclick=()=>{
  const n=$('#wpname').value.trim();
  if(!n)return wpNeedName();
  wpPending=n; setPane('map'); setTool('place');};
$('#wpname').addEventListener('keydown',e=>{
  e.stopPropagation();                       // do not drive the rover while typing
  if(e.key==='Enter')$('#wphere').click();});

async function fire(t,a){
  const dx=a.x1-a.x0, dy=a.y1-a.y0;
  // A plain click has no drag, so there is no heading in the gesture. Keep the
  // rover's current heading rather than snapping it to an arbitrary zero.
  const drag=Math.hypot(dx,dy)*mapScale>14;
  const yaw = drag ? Math.atan2(dy,dx)*180/Math.PI
                   : (M&&M.map_pose?M.map_pose.yaw:0);
  const body=JSON.stringify({x:a.x0,y:a.y0,yaw:yaw});
  if(t==='place'){
    const n=a.name;
    if(n){const j=await post('/api/wp/save',{name:n,x:a.x0,y:a.y0,yaw:yaw});
          if(j.ok)$('#wpname').value='';}
    return;}
  const url = t==='goal' ? '/api/goal' : '/api/initialpose';
  try{
    const r=await fetch(url,{method:'POST',body:body});
    const j=await r.json();
    if(!j.ok)$('#nav-de').textContent=j.err||'rejected';
  }catch(e){$('#nav-de').textContent='request failed';}
}

/* pan / zoom / follow
   Offsets come from the WRAPPER's rect, not e.offsetX: the toolbar buttons and
   the info bar sit on top of the canvas, and offsetX would be measured from
   whichever of those the pointer happened to land on. */
let mdrag=null;
const mpos=e=>{const r=msize();
  return {x:e.clientX-r.left, y:e.clientY-r.top};};
const onTool=e=>!!(e.target.closest&&e.target.closest('.maptools'));
MAPWRAP.addEventListener('pointerdown',e=>{
  if(onTool(e)||e.target.closest('.mtools'))return;
  const p=mpos(e);
  MAPWRAP.setPointerCapture(e.pointerId);
  if(tool!=='pan'){aim={x0:wx(p.x),y0:wy(p.y),x1:wx(p.x),y1:wy(p.y),
                        name:wpPending};return;}
  MAPWRAP.classList.add('drag');
  mdrag={x:p.x,y:p.y,cx:mapCam.x,cy:mapCam.y};});
MAPWRAP.addEventListener('pointermove',e=>{
  const p=mpos(e);
  mapCur={x:wx(p.x),y:wy(p.y)};
  if(aim){aim.x1=mapCur.x;aim.y1=mapCur.y;return;}
  if(!mdrag)return;
  mapCam.x=mdrag.cx-(p.x-mdrag.x)/mapScale;
  mapCam.y=mdrag.cy+(p.y-mdrag.y)/mapScale;
  mapFollow=false;$('#mlock').classList.remove('on');});
const mend=()=>{
  if(aim){const a=aim,t=tool;aim=null;setTool('pan');fire(t,a);}
  mdrag=null;MAPWRAP.classList.remove('drag');};
MAPWRAP.addEventListener('pointerup',mend);
MAPWRAP.addEventListener('pointercancel',mend);
MAPWRAP.addEventListener('pointerleave',()=>{mend();mapCur=null;});
MAPWRAP.addEventListener('wheel',e=>{if(onTool(e))return;e.preventDefault();
  const p=mpos(e);mapZoom(e.deltaY<0?1.12:1/1.12,p.x,p.y);},{passive:false});
$('#mzin').onclick =()=>mapZoom(1.3);
$('#mzout').onclick=()=>mapZoom(1/1.3);
$('#mfit').onclick =()=>mapFit();
$('#mlock').onclick=()=>{mapFollow=!mapFollow;
  $('#mlock').classList.toggle('on',mapFollow);};

(function mapLoop(){requestAnimationFrame(mapLoop);
  if(MAPCV.offsetParent)drawMap();})();

/* top-level view switch */
function setPane(v){
  $('#view-drive').hidden=(v!=='drive');
  $('#view-map').hidden  =(v!=='map');
  document.querySelectorAll('#vsw button').forEach(b=>b.classList.toggle('on',b.dataset.v===v));
  if(v==='map'&&mapMeta&&!mapFitted)mapFit();
  draw();}
document.querySelectorAll('#vsw button').forEach(b=>b.onclick=()=>setPane(b.dataset.v));
$('#gomap').onclick=()=>setPane('map');
addEventListener('keydown',e=>{
  if(typing(e))return;
  if(e.key==='m'||e.key==='M')setPane($('#view-map').hidden?'map':'drive');});

let view='color';
function setView(v){
  view=v;
  document.querySelectorAll('#views button').forEach(b=>b.classList.toggle('on',b.dataset.v===v));
  // cache-bust so the browser opens a fresh multipart stream rather than
  // reusing the previous view's connection
  $('#feed').src='/stream.mjpg?view='+v+'&t='+Date.now();
  const depthy=(v==='depth'||v==='blend');
  $('#legend').className='legend'+(depthy?' show':'');
  $('#lg-note').textContent = v==='depth' ? 'near → far'
    : v==='blend' ? 'depth over colour' : '';
  $('#hud-msg').textContent = v==='ir'
    ? 'waiting for IR stream…' : 'waiting for camera…';
}
document.querySelectorAll('#views button').forEach(b=>b.onclick=()=>setView(b.dataset.v));
let dmaxT=null;
$('#dmax').oninput=e=>{
  const hi=e.target.value/10;
  $('#lg-max').textContent=hi.toFixed(1)+' m';
  clearTimeout(dmaxT);
  dmaxT=setTimeout(()=>fetch('/api/depth_range',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({min:0.3,max:hi})}).catch(()=>{}),120);
};
setView('color');
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, *a):
        pass  # don't spam the ROS console with one line per poll

    def _send(self, code, ctype, body, extra=None):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self._send(200, 'text/html; charset=utf-8',
                       PAGE.replace('__BUILD__', BUILD).encode())
        elif self.path == '/api/metrics':
            self._send(200, 'application/json',
                       json.dumps(NODE.metrics()).encode())
        elif self.path == '/api/places':
            # Deliberately small and deliberately stable. An assistant polling
            # /api/metrics would be coupled to every internal the dashboard
            # happens to expose, and would be shipping the laser scan and the
            # whole plan several times a second to read one word of state.
            with NODE.lock:
                names = sorted(NODE.wp['points'])
                wp_map, cur_map = NODE.wp['map'], NODE.map_name
                nav = dict(NODE.nav)
                localised = NODE.map_pose is not None
                pose = dict(NODE.map_pose) if NODE.map_pose else None
                estop = NODE.estop
            stack = NODE.sup.status()
            # One flag worth trusting: is it safe to offer someone a walk to a
            # demo station right now? Anything less and the assistant has to
            # re-derive this, and will get it wrong.
            ready = (stack['state'] == 'navigation' and localised
                     and not estop and bool(names)
                     and not (wp_map and cur_map and wp_map != cur_map))
            self._send(200, 'application/json', json.dumps({
                'ok': True,
                'ready': ready,
                'places': names,
                'map': cur_map,
                'places_map': wp_map,
                'mode': stack['state'],
                'localised': localised,
                'pose': pose,
                'estop': estop,
                'nav': {'state': nav['state'], 'place': nav.get('place'),
                        'remaining': nav['remaining'],
                        'result': nav['result']},
            }).encode())
        elif self.path == '/api/stacklog':
            self._send(200, 'application/json',
                       json.dumps({'lines': NODE.sup.tail(80)}).encode())
        elif self.path.startswith('/api/map.png'):
            with NODE.lock:
                png = NODE.map_png
            if png is None:
                self._send(503, 'text/plain', b'no map')
            else:
                self._send(200, 'image/png', png)
        elif self.path.startswith('/stream.mjpg'):
            view = 'color'
            if '?' in self.path:
                q = urllib.parse.parse_qs(self.path.split('?', 1)[1])
                view = (q.get('view', ['color'])[0] or 'color').lower()
            if view not in ('color', 'depth', 'ir', 'blend'):
                view = 'color'
            self.stream(view)
        else:
            self._send(404, 'text/plain', b'not found')

    def stream(self, view='color'):
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        last = None
        try:
            while True:
                # Renew interest every frame: the node only encodes views that
                # someone has asked for in the last 5 s, so an unwatched depth
                # or IR view costs nothing.
                with NODE.lock:
                    NODE.wanted[view] = time.time()
                    j = NODE.frames.get(view)
                if j is not None and j is not last:
                    last = j
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                     b'Content-Length: ' + str(len(j)).encode() +
                                     b'\r\n\r\n' + j + b'\r\n')
                time.sleep(1 / 15.0)   # cap the stream; the camera may run at 30
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        n = int(self.headers.get('Content-Length', 0))
        try:
            body = json.loads(self.rfile.read(n) or b'{}')
        except Exception:
            body = {}
        if self.path == '/api/cmd':
            NODE.set_cmd(body.get('lin', 0.0), body.get('ang', 0.0))
            self._send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/depth_range':
            try:
                lo = float(body.get('min', DEPTH_MIN_M))
                hi = float(body.get('max', DEPTH_MAX_M))
                if hi - lo >= 0.3:
                    with NODE.lock:
                        NODE.d_min, NODE.d_max = max(0.1, lo), min(12.0, hi)
            except Exception:
                pass
            self._send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/goal':
            if NODE.estop:
                self._send(409, 'application/json',
                           b'{"ok":false,"err":"e-stop engaged"}')
            else:
                with NODE.lock:
                    NODE.nav_req = {'x': float(body['x']), 'y': float(body['y']),
                                    'yaw': float(body.get('yaw', 0.0))}
                self._send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/nav_cancel':
            with NODE.lock:
                NODE.nav_cancel_req = True
            self._send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/initialpose':
            NODE.set_initial_pose(float(body['x']), float(body['y']),
                                  float(body.get('yaw', 0.0)))
            self._send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/wp/save':
            # 'here' captures the pose the rover is actually standing at: drive
            # to the booth with the keyboard, then name the spot. That is the
            # flow that gets used, far more than clicking a point on a map.
            if body.get('here'):
                with NODE.lock:
                    P = NODE.map_pose
                if P is None:
                    self._send(409, 'application/json',
                               b'{"ok":false,"err":"rover is not localised"}')
                    return
                x, y, yaw = P['x'], P['y'], P['yaw']
            else:
                x, y, yaw = (float(body['x']), float(body['y']),
                             float(body.get('yaw', 0.0)))
            ok, msg = NODE.set_waypoint(body.get('name', ''), x, y, yaw)
            self._send(200 if ok else 400, 'application/json',
                       json.dumps({'ok': ok, 'err': None if ok else msg,
                                   'name': msg if ok else None}).encode())
        elif self.path == '/api/wp/delete':
            ok = NODE.del_waypoint(body.get('name', ''))
            self._send(200, 'application/json',
                       json.dumps({'ok': ok,
                                   'err': None if ok else 'no such waypoint'}).encode())
        elif self.path == '/api/wp/localise':
            # Park the rover on a marked spot, press one button, done.
            #
            # AMCL forgets its pose on every stack restart, and at an expo the
            # stack restarts whenever the rover is power-cycled or the map is
            # changed. Re-setting it by dragging on the map is imprecise and
            # has to be redone perfectly every time -- and a sloppy initial
            # pose is the single thing everything downstream depends on. A
            # waypoint captured at a physical floor marker is exact and
            # repeatable, which is what a demo day needs.
            with NODE.lock:
                pt = NODE.wp['points'].get(NODE.clean_name(body.get('name', '')))
            if pt is None:
                self._send(404, 'application/json',
                           b'{"ok":false,"err":"no such waypoint"}')
            else:
                NODE.set_initial_pose(pt['x'], pt['y'], pt['yaw'])
                self._send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/wp/goto':
            with NODE.lock:
                pt = NODE.wp['points'].get(NODE.clean_name(body.get('name', '')))
            if pt is None:
                self._send(404, 'application/json',
                           b'{"ok":false,"err":"no such waypoint"}')
            elif NODE.estop:
                self._send(409, 'application/json',
                           b'{"ok":false,"err":"e-stop engaged"}')
            else:
                with NODE.lock:
                    NODE.nav_req = dict(pt, place=NODE.clean_name(body['name']))
                self._send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/mode':
            want = str(body.get('mode', 'idle'))
            if want == 'idle':
                ok, msg = NODE.sup.stop()
            else:
                opts = {}
                if body.get('map'):
                    opts['map'] = str(body['map'])
                # A switch stops the old stack and starts the new one as ONE
                # action, so "leave mapping" never lands on a dead rover.
                ok, msg = (NODE.sup.switch(want, opts) if body.get('switch')
                           else NODE.sup.start(want, opts))
            self._send(200 if ok else 409, 'application/json',
                       json.dumps({'ok': ok, 'err': None if ok else msg}).encode())
        elif self.path == '/api/camera':
            ok, msg = NODE.sup.camera(bool(body.get('on')))
            self._send(200 if ok else 409, 'application/json',
                       json.dumps({'ok': ok, 'err': None if ok else msg}).encode())
        elif self.path == '/api/save_map':
            with NODE.lock:
                busy = NODE.save_state.get('busy')
                if not busy:
                    NODE.save_req = str(body.get('name', ''))
            self._send(200 if not busy else 409, 'application/json',
                       json.dumps({'ok': not busy,
                                   'err': 'a save is already running'
                                          if busy else None}).encode())
        elif self.path == '/api/estop':
            with NODE.lock:
                NODE.estop = bool(body.get('on', True))
                if NODE.estop:
                    NODE.cmd = (0.0, 0.0)
                    # Zeroing /cmd_vel only fights Nav2 -- the goal stays active
                    # and keeps commanding. An emergency stop has to end the
                    # goal, not out-shout it.
                    NODE.nav_cancel_req = True
            self._send(200, 'application/json', b'{"ok":true}')
        else:
            self._send(404, 'text/plain', b'not found')


def _port_holder(port):
    """PID currently listening on `port`, if we can work it out."""
    try:
        out = subprocess.run(['ss', '-lptnH', f'sport = :{port}'],
                             capture_output=True, text=True, timeout=3).stdout
        m = re.search(r'pid=(\d+)', out)
        return int(m.group(1)) if m else None
    except Exception:                               # noqa: BLE001
        return None


def main(args=None):
    global NODE
    rclpy.init(args=args)
    NODE = Dash()
    port = NODE.port
    try:
        srv = ThreadingHTTPServer(('0.0.0.0', port), Handler)
    except OSError as e:
        if e.errno != errno.EADDRINUSE:
            raise
        # Starting a second console is the single easiest mistake to make, and
        # a socket traceback says nothing useful about it. The first one is
        # still serving; say so, and say how to take the port if that is really
        # what was wanted.
        who = _port_holder(port)
        NODE.get_logger().error(
            f'a rover console is ALREADY running on port {port}'
            + (f' (pid {who})' if who else '') + '.\n'
            f'    Open http://{local_ip()}:{port} -- that one is still serving, '
            f'and it owns the running stack.\n'
            '    Only if you really want to replace it:  kill '
            + (str(who) if who else f'$(fuser -n tcp {port} 2>/dev/null)') +
            '\n    The stack and camera keep running either way; a new console '
            'adopts them.')
        NODE.destroy_node()
        rclpy.shutdown()
        return 1
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    # Every step of teardown is guarded. On Ctrl+C the rclpy context is already
    # tearing down, so publishing or destroying can raise -- and an unhandled
    # exception here makes the node exit(1), which shows up in the launch log as
    # "process has died" and looks like a crash when it is only a messy exit.
    try:
        rclpy.spin(NODE)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:                      # noqa: BLE001
        NODE.get_logger().error(f"dashboard stopped: {e}")
    finally:
        for step in (
            lambda: NODE.pub.publish(Twist()),  # leave the rover stopped
            srv.shutdown,
            NODE.destroy_node,
        ):
            try:
                step()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
