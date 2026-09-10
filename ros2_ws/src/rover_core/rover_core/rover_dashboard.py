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
import json, math, threading, time, io, socket, urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from PIL import Image as PImage

from geometry_msgs.msg import Twist, Point32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan

CMD_HZ     = 10.0
DEADMAN_S  = 0.6
STREAM_W   = 640
JPEG_Q     = 70
PORT       = 8080

SENSOR_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
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
        self.create_subscription(Point32, '/wheel_ticks',
                                 lambda m: self._set('ticks', (m.x, m.y, m.z)), 10)
        self.create_subscription(Point32, '/rover_diag',
                                 lambda m: self._set('diag', (m.x, m.y, m.z)), 10)
        self.create_subscription(Point32, '/encoder_ticks',
                                 lambda m: self._set('enc', (m.x, m.y, m.z)), 10)
        self.create_timer(1.0 / CMD_HZ, self.tick_cmd)
        self.get_logger().info(f"dashboard on http://{local_ip()}:{PORT}")

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
        with self.lock:
            lin, ang = self.cmd
            age = time.time() - self.cmd_time
            stopped = self.estop
        if stopped or age > DEADMAN_S:
            lin = ang = 0.0
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.pub.publish(t)

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
                'path': list(self.path),
                'vhist': [[round(now - t, 2), round(c, 3), round(v, 3)]
                          for t, c, v in self.v_hist[-120:]],
                'limits': {'lin': self.max_lin, 'ang': self.max_ang},
                'cmd': {'lin': cmd[0], 'ang': cmd[1],
                        'active': (not estop) and cmd_age < DEADMAN_S},
                'estop': estop,
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
#estop{flex:none;padding:8px clamp(10px,1.4vw,18px);border-radius:8px;
 border:1px solid var(--bad);background:var(--bad);color:#fff;
 font:700 clamp(10px,1.15vh,12px)/1 inherit;letter-spacing:.05em;cursor:pointer}
#estop:hover{filter:brightness(1.1)}
#estop.armed{background:rgba(248,81,73,.12);color:var(--bad);animation:pulse 1.4s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.55}}

/* ---------- layout: fills the viewport, never scrolls ---------- */
.grid{flex:1;display:grid;gap:var(--g);padding:var(--g);overflow:hidden;
 grid-template-columns:minmax(0,1.72fr) minmax(300px,1fr)}
@media(max-width:1000px){.grid{grid-template-columns:minmax(0,1.35fr) minmax(240px,1fr)}}
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
canvas{position:absolute;inset:0;display:block}

/* ---------- camera ---------- */
#camcard{flex:1}
.vidwrap{flex:1;position:relative;background:#000;overflow:hidden}
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
 font-size:var(--fs)}
.m:last-child{border-bottom:0;padding-bottom:0}
.m:first-child{padding-top:0}
.m .k{color:var(--dim);overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.m .v{font:600 var(--fs)/1 var(--mono);white-space:nowrap;flex:none}
.v.ok{color:var(--ok)}.v.warn{color:var(--warn)}.v.bad{color:var(--bad)}.v.dim{color:var(--faint)}
#radcard{flex:1.5}#spkcard{flex:.85}#odcard{flex:1.5}#dtcard{flex:1.15}
@media(max-height:700px){#spkcard{display:none}}
</style></head><body>

<header>
  <div class="brand"><span class="dot" id="live"></span><h1>Rover Control</h1></div>
  <div class="stats">
    <div class="chip" id="c-lat">lat <b>—</b></div>
    <div class="chip" id="c-esp">ESP32 <b>—</b></div>
    <div class="chip" id="c-lidar">lidar <b>—</b></div>
    <div class="chip" id="c-cam">cam <b>—</b></div>
    <div class="chip opt" id="c-wdog">wdog <b>—</b></div>
    <div class="chip opt" id="c-up">up <b>—</b></div>
  </div>
  <button id="estop">EMERGENCY STOP</button>
</header>

<div class="grid">
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
      <h2>Drive <span class="tag">hold to move · release to stop</span></h2>
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
let keys={}, speed=.20, turn=.60, estop=false, boost=false, M=null;
const f=(x,n=2)=>(x==null||!isFinite(x))?'—':(+x).toFixed(n);
const row=(k,v,c='')=>`<div class="m"><span class="k">${k}</span><span class="v ${c}">${v}</span></div>`;

/* ---------- commands ---------- */
function cmdVec(){const m=boost?1.6:1;
  return {lin:((keys.w?1:0)-(keys.s?1:0))*speed*m,
          ang:((keys.a?1:0)-(keys.d?1:0))*turn*m};}
async function send(){const c=cmdVec();
  try{await fetch('/api/cmd',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(c)});}catch(e){}}
setInterval(send,100);
function setKey(k,on){
  if(keys[k]===on)return; keys[k]=on;
  document.querySelectorAll('[data-k="'+k+'"]').forEach(b=>b.classList.toggle('held',on));
  const c=cmdVec(), L=(M&&M.limits)||{lin:.35,ang:1};
  const mag=Math.min(1,Math.hypot(c.lin/L.lin,c.ang/L.ang));
  $('#thrbar').style.width=(mag*100)+'%';
  $('#thr').textContent=(c.lin||c.ang)?(f(c.lin)+' · '+f(c.ang)):'idle';
  send();}
const release=()=>['w','a','s','d'].forEach(k=>setKey(k,false));
addEventListener('keydown',e=>{
  if(e.repeat)return;
  if(e.code==='Space'){e.preventDefault();toggleStop();return;}
  if(e.key==='Shift'){boost=true;return;}
  if('123'.includes(e.key)){setPreset([10,20,35][+e.key-1]);return;}
  const k=e.key.toLowerCase(); if('wasd'.includes(k)){e.preventDefault();setKey(k,true);}});
addEventListener('keyup',e=>{if(e.key==='Shift'){boost=false;return;}
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

function radar(pts,near){
  const [g,W,H]=fit($('#radar'));
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
  $('#radtag').textContent=near==null?'clear':f(near)+' m';}

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
function draw(){if(!M)return;radar(M.scan,M.lidar.nearest_fwd);
  if($('#spkcard').offsetParent)spark(M.vhist);trace(M.path,M.odom);}

async function poll(){
  const t0=performance.now();
  try{M=await (await fetch('/api/metrics')).json();}
  catch(e){$('#live').className='dot off';chip('#c-lat','bad','off');return;}
  const lat=Math.round(performance.now()-t0);
  $('#live').className='dot';
  const m=M;
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
}
setInterval(poll,300); poll();
new ResizeObserver(draw).observe(document.querySelector('.grid'));
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
            self._send(200, 'text/html; charset=utf-8', PAGE.encode())
        elif self.path == '/api/metrics':
            self._send(200, 'application/json',
                       json.dumps(NODE.metrics()).encode())
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
        elif self.path == '/api/estop':
            with NODE.lock:
                NODE.estop = bool(body.get('on', True))
                if NODE.estop:
                    NODE.cmd = (0.0, 0.0)
            self._send(200, 'application/json', b'{"ok":true}')
        else:
            self._send(404, 'text/plain', b'not found')


def main(args=None):
    global NODE
    rclpy.init(args=args)
    NODE = Dash()
    srv = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        rclpy.spin(NODE)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            NODE.pub.publish(Twist())   # stop on the way out
        except Exception:
            pass
        srv.shutdown()
        NODE.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
