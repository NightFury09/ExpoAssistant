"""Floor-boundary vision: one forward camera -> a virtual laser scan.

No ROS and no GPU in here, so it can be tested on synthetic images and run on a
bench. front_cam_scan.py is the thin ROS wrapper.

The idea is the classic "visual sonar" / floor-boundary one (the same shape as
ROS's depthimage_to_laserscan, but from colour instead of depth): the strip of
floor right in front of the rover is, by definition, floor. Learn what it looks
like, find every pixel that continues that look upward from the bottom of the
image, and in each image column the first thing that is NOT floor is where an
obstacle meets the ground. A calibrated camera at a known height and tilt turns
that pixel into a distance on the floor. A chair leg is exactly this case: a
thin vertical thing standing on the floor that a lidar plane at 27.5 cm can miss.

What it cannot do, and the caller must treat as normal:
  * an obstacle the same colour as the floor (a white leg on a white floor);
  * a floor that changes colour (a rug edge reads as a wall);
  * glare, darkness, and an obstacle already under the camera's seed strip.
So it reports a confidence, publishes nothing when it is low, and is meant to
sit BESIDE the lidar and the depth camera, never replace them.

Frames: base frame x forward, y left, z up, origin on the floor (base_link).
Optical frame: x right, y down, z forward.
"""
import math
from dataclasses import dataclass, field

import cv2
import numpy as np


# --------------------------------------------------------------------------- #
# Camera model and mounting
# --------------------------------------------------------------------------- #
@dataclass
class Camera:
    """Pinhole camera with Brown-Conrady distortion (OpenCV convention)."""
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    dist: tuple = (0.0, 0.0, 0.0, 0.0, 0.0)

    @classmethod
    def nominal(cls, width, height, hfov_deg):
        """No calibration: a pinhole with no distortion from a field of view.
        Good enough to test with; calibrate before trusting a distance."""
        f = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        return cls(width, height, f, f, width / 2.0, height / 2.0)

    def scaled(self, width, height):
        sx, sy = width / self.width, height / self.height
        return Camera(width, height, self.fx * sx, self.fy * sy,
                      self.cx * sx, self.cy * sy, self.dist)

    @property
    def matrix(self):
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]],
                        np.float64)

    @property
    def hfov_deg(self):
        return math.degrees(2 * math.atan(self.width / 2.0 / self.fx))

    @property
    def vfov_deg(self):
        return math.degrees(2 * math.atan(self.height / 2.0 / self.fy))

    def normalised(self, px):
        """(N,2) pixels -> (N,2) undistorted normalised image coordinates."""
        pts = np.asarray(px, np.float64).reshape(-1, 1, 2)
        out = cv2.undistortPoints(pts, self.matrix, np.asarray(self.dist, np.float64))
        return out.reshape(-1, 2)


@dataclass
class Mount:
    """Where the camera sits on the rover, in the base frame."""
    x: float = 0.0          # m forward of base_link
    y: float = 0.0          # m left
    z: float = 0.25         # m above the floor
    pitch_deg: float = 0.0  # nose-down tilt, positive looks at the floor
    yaw_deg: float = 0.0    # left-positive; 0 = straight ahead


def _rays(norm, mount):
    """Normalised optical coordinates (N,2) -> unit-ish direction in base frame."""
    xn, yn = norm[:, 0], norm[:, 1]
    fwd, left, up = np.ones_like(xn), -xn, -yn          # level camera body frame
    p, w = math.radians(mount.pitch_deg), math.radians(mount.yaw_deg)
    fwd_b = fwd * math.cos(p) + up * math.sin(p)
    up_b = -fwd * math.sin(p) + up * math.cos(p)
    dx = fwd_b * math.cos(w) - left * math.sin(w)
    dy = fwd_b * math.sin(w) + left * math.cos(w)
    return np.stack([dx, dy, up_b], axis=1)


def pixel_to_ground(cam, mount, px):
    """(N,2) pixels -> (N,2) floor points (x, y) in the base frame; NaN for a
    pixel whose ray never reaches the floor (at or above the horizon)."""
    d = _rays(cam.normalised(px), mount)
    out = np.full((len(d), 2), np.nan)
    hit = d[:, 2] < -1e-6
    t = mount.z / -d[hit, 2]
    out[hit, 0] = mount.x + t * d[hit, 0]
    out[hit, 1] = mount.y + t * d[hit, 1]
    return out


def point_to_pixel(cam, mount, pts):
    """(N,3) base-frame points -> (N,2) pixels; NaN behind the camera."""
    pts = np.asarray(pts, np.float64).reshape(-1, 3)
    v = pts - np.array([mount.x, mount.y, mount.z])
    p, w = math.radians(mount.pitch_deg), math.radians(mount.yaw_deg)
    # undo yaw, then pitch
    fwd_y = v[:, 0] * math.cos(w) + v[:, 1] * math.sin(w)
    left = -v[:, 0] * math.sin(w) + v[:, 1] * math.cos(w)
    up = v[:, 2]
    fwd = fwd_y * math.cos(p) - up * math.sin(p)
    up_c = fwd_y * math.sin(p) + up * math.cos(p)
    out = np.full((len(v), 2), np.nan)
    ok = fwd > 1e-6
    xn, yn = -left[ok] / fwd[ok], -up_c[ok] / fwd[ok]
    r2 = xn * xn + yn * yn
    k1, k2, p1, p2, k3 = (list(cam.dist) + [0.0] * 5)[:5]
    radial = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
    xd = xn * radial + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn)
    yd = yn * radial + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn
    out[ok, 0] = cam.fx * xd + cam.cx
    out[ok, 1] = cam.fy * yd + cam.cy
    return out


def horizon_row(cam, mount):
    """Image row of the horizon (where a level ray lands). Rows above it never
    reach the floor."""
    p = point_to_pixel(cam, mount, [[mount.x + 100.0, mount.y, mount.z]])
    return float(p[0, 1])


# --------------------------------------------------------------------------- #
# Can the camera see the floor at all from this spot?
# --------------------------------------------------------------------------- #
# Boxes the rover's own body puts in the camera's way, from rover.urdf:
# (x_min, x_max, z_top). Seen in the vertical plane straight ahead only.
# The URDF's green 6 x 14 cm "front indicator" is cosmetic (a visual-only marker
# for the viewer), so it is deliberately NOT here. The mast that carries the screen
# and the D455 is not modelled either; see FRONT_CAMERA.md.
DECK = (-0.215, 0.215, 0.23)
FRONT_EDGE = 0.215


def floor_visibility(cam, mount, bodies=(DECK,), front_edge=FRONT_EDGE):
    """Which of the camera's vertical rays reach the floor, and how close.

    Returns {'nearest_m': metres from the FRONT EDGE of the rover to the nearest
    visible floor (None if no ray reaches it), 'nearest_from_camera_m',
    'blocked_fraction': share of vertical rays that hit the rover's own body
    first, 'floor_fraction': share that reach the floor}.

    Straight-ahead plane only, using the deck as a box: a quick geometric check
    to run BEFORE drilling holes, not a render.
    """
    rows = np.linspace(0, cam.height - 1, 121)
    px = np.stack([np.full_like(rows, cam.cx), rows], axis=1)
    d = _rays(cam.normalised(px), mount)
    x0, z0 = mount.x, mount.z
    nearest, blocked, floor = None, 0, 0
    for dx, dz in zip(d[:, 0], d[:, 2]):
        if dz >= -1e-6 or dx <= 1e-6:
            continue                           # at/above horizon, or looking backwards
        hit_body = False
        for xa, xb, zt in bodies:
            if z0 <= zt and xa <= x0 <= xb:
                hit_body = True                # the camera is inside the box
                break
            if z0 > zt:
                t = (z0 - zt) / -dz            # where the ray crosses the box top
                xh = x0 + t * dx
                if xa <= xh <= xb:
                    hit_body = True
                    break
        if hit_body:
            blocked += 1
            continue
        floor += 1
        dist = x0 + (z0 / -dz) * dx
        if nearest is None or dist < nearest:
            nearest = dist
    n = max(1, len(d))
    return {'nearest_m': None if nearest is None else nearest - front_edge,
            'nearest_from_camera_m': None if nearest is None else nearest - x0,
            'blocked_fraction': blocked / n, 'floor_fraction': floor / n}


# --------------------------------------------------------------------------- #
# Floor segmentation
# --------------------------------------------------------------------------- #
@dataclass
class VisionParams:
    work_width: int = 320
    # The trusted strip: the bottom of the image, centred. It is the floor the
    # rover is about to drive over, so it defines "floor".
    seed_rows: float = 0.15          # bottom fraction of rows
    seed_cols: tuple = (0.25, 0.75)  # centred fraction of columns
    gate_sigmas: float = 4.0         # how far from the seed colour still counts
    min_sigma: tuple = (5.0, 3.0, 3.0)   # floors on the spread (Lab), stops a
    #                                      flat seed from rejecting its own noise
    light_slack: float = 14.0        # extra L tolerance: far floor is lit differently
    blur_px: int = 5
    close_rows: int = 7              # fills thin horizontal lines (tile grout)
    min_width_px: int = 3            # drops 1-2 px wide lines; a leg is wider
    min_run_px: int = 10             # an obstacle must stand this tall
    min_floor_fraction: float = 0.30  # below this the floor model is not trusted
    seed_spread_max: float = 25.0    # a seed this varied is not one floor
    min_brightness: float = 8.0      # mean L (0-100): a covered lens or a dark room
    max_brightness: float = 97.0


@dataclass
class FloorResult:
    mask: np.ndarray                 # True = floor, working resolution
    contact_rows: np.ndarray         # per column: row of obstacle contact, -1 none
    confidence: float
    reason: str = ''
    seed: tuple = field(default_factory=tuple)   # median Lab of the seed strip


def segment_floor(bgr, vp=VisionParams(), horizon=None):
    """BGR image -> FloorResult at the working resolution."""
    h0, w0 = bgr.shape[:2]
    scale = vp.work_width / float(w0)
    small = cv2.resize(bgr, (vp.work_width, max(8, int(round(h0 * scale)))),
                       interpolation=cv2.INTER_AREA)
    h, w = small.shape[:2]
    k = vp.blur_px | 1
    lab = cv2.cvtColor(cv2.GaussianBlur(small, (k, k), 0),
                       cv2.COLOR_BGR2LAB).astype(np.float32)

    r0 = int(h * (1.0 - vp.seed_rows))
    c0, c1 = int(w * vp.seed_cols[0]), int(w * vp.seed_cols[1])
    mean_l = float(lab[..., 0].mean()) * 100.0 / 255.0
    if mean_l < vp.min_brightness or mean_l > vp.max_brightness:
        return FloorResult(np.zeros((h, w), bool), -np.ones(w, int), 0.0,
                           'image too dark or too bright (lens covered, or no light)')
    seed = lab[r0:, c0:c1].reshape(-1, 3)
    med = np.median(seed, axis=0)
    mad = 1.4826 * np.median(np.abs(seed - med), axis=0)
    sigma = np.maximum(mad, np.asarray(vp.min_sigma, np.float32))
    if float(np.max(mad)) > vp.seed_spread_max:
        return FloorResult(np.zeros((h, w), bool), -np.ones(w, int), 0.0,
                           'the strip in front is not one colour', tuple(med))

    tol = vp.gate_sigmas * sigma
    tol[0] += vp.light_slack
    near = np.all(np.abs(lab - med) <= tol, axis=2)

    # Only floor that is joined to the seed strip counts: a floor-coloured poster
    # on a wall is not floor.
    near_u8 = near.astype(np.uint8)
    n, lbl = cv2.connectedComponents(near_u8, connectivity=4)
    keep = np.unique(lbl[r0:, c0:c1])
    keep = keep[keep > 0]
    floor = np.isin(lbl, keep)

    # Thin dark lines (grout, a cable) are not obstacles: close them vertically.
    floor_u8 = floor.astype(np.uint8)
    floor_u8 = cv2.morphologyEx(
        floor_u8, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (1, max(1, vp.close_rows))))
    # Thin vertical lines in the not-floor mask likewise.
    obst = (floor_u8 == 0).astype(np.uint8)
    obst = cv2.morphologyEx(
        obst, cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (max(1, vp.min_width_px), 1)))
    floor = obst == 0

    top = 0 if horizon is None else int(max(0, min(h - 1, horizon * scale)))
    contact = -np.ones(w, int)
    for c in range(w):
        col = obst[top:, c]
        r = len(col) - 1
        while r >= 0:                               # sweep from the bottom upward
            if not col[r]:
                r -= 1
                continue
            start = r                               # first obstacle pixel: its foot
            while r >= 0 and col[r]:
                r -= 1
            if start - r >= vp.min_run_px:          # tall enough to be real
                contact[c] = top + start
                break                               # nearest obstacle in this column

    below = floor[top:, :]
    frac = float(below.mean()) if below.size else 0.0
    conf = min(1.0, frac / max(vp.min_floor_fraction, 1e-6)) if frac else 0.0
    reason = '' if frac >= vp.min_floor_fraction else 'too little of the view looks like floor'
    return FloorResult(floor, contact, conf, reason, tuple(med))


def scan_from_floor(res, cam, mount, vp=VisionParams(), bins=90,
                    min_range=0.25, max_range=2.5):
    """FloorResult -> (angle_min, angle_increment, ranges) in the gravity-aligned
    scan frame at the camera (yaw only; no pitch). +inf means "floor all the way
    to max_range here" -- the costmap may clear through it. NaN-free."""
    w = res.mask.shape[1]
    scale = w / float(cam.width)
    cw = cam.scaled(w, int(round(cam.height * scale)))
    h_w = res.mask.shape[0]
    half = math.radians(cw.hfov_deg) / 2.0
    ranges = np.full(bins, math.inf)
    inc = 2 * half / bins
    angle_min = -half
    cols = np.arange(w)
    # Where does each column's ground-contact pixel land on the floor?
    rows = np.where(res.contact_rows >= 0, res.contact_rows, h_w - 1)
    pts = pixel_to_ground(cw, mount, np.stack([cols + 0.5, rows + 0.5], axis=1))
    for c in range(w):
        if res.contact_rows[c] < 0 or np.isnan(pts[c, 0]):
            continue
        dx, dy = pts[c, 0] - mount.x, pts[c, 1] - mount.y
        r = math.hypot(dx, dy)
        if r > max_range:
            continue                      # too far to trust: leave as clear
        ang = math.atan2(dy, dx) - math.radians(mount.yaw_deg)
        b = int((ang - angle_min) / inc)
        if 0 <= b < bins:
            ranges[b] = min(ranges[b], max(r, min_range))
    return angle_min, inc, ranges
