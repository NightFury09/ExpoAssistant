#!/usr/bin/env python3
"""
RealSense x NVIDIA -- Edge AI Perception Station
================================================
One camera, one app, several booth modes sharing a single depth pipeline.
Everything runs ON THIS JETSON, offline. The browser UI is the booth screen.

Modes (switch from the on-screen tabs):
  safety     Proximity / safety zone -- screen reacts as people approach.
  dimension  Live object dimensioning -- real-world W x H + distance from depth.
  face       Face greeting -- SFace recognition + depth liveness (anti-spoofing).
  cloud      Live 3D point cloud -- orbit a real-time 3D scan in the browser.
  detect     Open-vocabulary AI detection (NanoOWL) + distance -- "find anything".
  about      NVIDIA cloud-to-edge story panel (UI only).

Web UI:  http://<host>:8000/   (stdlib http.server -- no Flask, no X11)
"""

import json
import os
import re
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import pyrealsense2 as rs

# --------------------------------------------------------------------------- #
# Config
# --------------------------------------------------------------------------- #
def _f(n, d):
    try:
        return float(os.environ.get(n, d))
    except (TypeError, ValueError):
        return float(d)


def _s(n, d):
    return os.environ.get(n, d)


WIDTH        = int(_f("RS_WIDTH", 640))
HEIGHT       = int(_f("RS_HEIGHT", 480))
FPS          = int(_f("RS_FPS", 30))
PORT         = int(_f("RS_PORT", 8000))
JPEG_QUALITY = int(_f("RS_JPEG_QUALITY", 80))

WEB_DIR   = _s("RS_WEB_DIR", "web")
MODEL_DIR = _s("RS_MODEL_DIR", "models")
DATA_DIR  = _s("RS_DATA_DIR", "data")
OWL_ENGINE = _s("RS_OWL_ENGINE", "/opt/nanoowl/data/owl_image_encoder_patch32.engine")
START_MODE = _s("RS_START_MODE", "safety")


def _b(n, d=False):
    return _s(n, "1" if d else "0").strip().lower() in ("1", "true", "yes", "on")


# Nano fork: run only the RealSense depth demos (Safety/Inspect/Box Vol/Greet/3D Scan).
# AI Detect (NanoOWL/TensorRT) stays on the AGX Orin, so it is disabled here -- no torch/
# tensorrt/nanoowl needed at runtime, which keeps the Orin Nano image lean.
REALSENSE_ONLY = _b("RS_REALSENSE_ONLY", False)
# If set, the "Product AI" tab links here (the assistant lives on the AGX after the fork),
# e.g. https://192.168.3.224:5001 . Empty -> derive from the page host (single-box setup).
ASSISTANT_URL = _s("RS_ASSISTANT_URL", "")

# proximity
DANGER_M  = _f("RS_DANGER_M", 0.80)
CAUTION_M = _f("RS_CAUTION_M", 1.50)
# dimensioning
MEASURE_MIN_M = _f("RS_MEASURE_MIN_M", 0.25)
MEASURE_MAX_M = _f("RS_MEASURE_MAX_M", 1.30)
DEPTH_BAND_M  = _f("RS_DEPTH_BAND_M", 0.10)
MIN_OBJ_FRAC  = _f("RS_MIN_OBJ_FRAC", 0.015)
MEASURE_ZONE_W = _f("RS_MEASURE_ZONE_W", 0.60)   # reference box, fraction of frame
MEASURE_ZONE_H = _f("RS_MEASURE_ZONE_H", 0.72)
MEASURE_IGNORE_SKIN = int(_f("RS_MEASURE_IGNORE_SKIN", 1))  # drop hands from the object
# surface inspection (real depth-based QC)
INSPECT_ENABLE = int(_f("RS_INSPECT_ENABLE", 1))
DENT_MM = _f("RS_DENT_MM", 3.0)                  # depth deviation (mm) counted as a dent/bump
DENT_MIN_AREA = int(_f("RS_DENT_MIN_AREA", 50))  # px, ignore anomaly blobs smaller than this
INSPECT_TOL_MM = _f("RS_INSPECT_TOL_MM", 4.0)    # max deformation (mm) that still PASSES
# open-vocab labels the inspection expects (names what's held)
HANDHELD_VOCAB = ["a box", "a phone", "a mobile phone", "a cup", "a bottle",
                  "a can", "a book", "a carton", "a remote control", "a package"]
# things that are NOT a held product -- if a crop matches one of these better than
# any product class, the segment is the person / their clothing, not an object, so
# Inspect + Box Vol reject it (fixes the torso/t-shirt being measured as a box).
BODY_VOCAB = ["a person", "a chest", "a t-shirt", "a shirt", "a jacket",
              "clothing", "a torso", "a human body", "a face", "an arm"]
OBJ_MAX_FILL = _f("RS_OBJ_MAX_FILL", 0.72)          # segment filling >this frac of the zone = body/bg
BODY_REJECT_SCORE = _f("RS_BODY_REJECT_SCORE", 0.10)  # NanoOWL body-class score that rejects a segment
# box volume scan (rotate the box; combine faces -> L x W x H)
BOX_TILT_RATIO = _f("RS_BOX_TILT_RATIO", 0.22)   # depth-spread/diag to count a face square-on
BOX_RECT_MIN = _f("RS_BOX_RECT_MIN", 0.80)       # contour/minrect area ratio to accept a box face
BOX_CLUSTER_TOL = _f("RS_BOX_CLUSTER_TOL", 0.15) # relative tolerance grouping edge lengths
BOX_LOCK_SAMPLES = int(_f("RS_BOX_LOCK_SAMPLES", 6))  # samples per dimension before it locks
BOX_EMPTY_RESET = int(_f("RS_BOX_EMPTY_RESET", 10))   # empty frames before auto-reset for next box
BOX_RGB_ASSIST = int(_f("RS_BOX_RGB_ASSIST", 1))      # bigger close to bridge glossy depth holes
BOX_HOLD_SEC = _f("RS_BOX_HOLD_SEC", 5.0)             # show a locked volume this long, then reset
# face
RETENTION_H = _f("RS_RETENTION_HOURS", 8.0)
COSINE_MATCH = _f("RS_COSINE_MATCH", 0.363)
SAME_FACE_MIN = _f("RS_SAME_FACE_MIN", 0.28)   # cosine < this while enrolling => a different
                                               # person stepped in -> restart (never merge two)
MIN_FACE_PX = int(_f("RS_MIN_FACE_PX", 80))
ENROLL_STABLE = int(_f("RS_ENROLL_STABLE", 6))
LIVE_MIN_M = _f("RS_LIVE_MIN_M", 0.30)
LIVE_MAX_M = _f("RS_LIVE_MAX_M", 1.20)
LIVE_RELIEF_M = _f("RS_LIVE_RELIEF_M", 0.010)
LIVE_STD_M = _f("RS_LIVE_STD_M", 0.010)
ENROLL_VIEWS = int(_f("RS_ENROLL_VIEWS", 6))     # multi-angle embeddings per person
POSE_MIN_DELTA = _f("RS_POSE_MIN_DELTA", 0.06)   # head-pose change to bank a new view
GREET_REDIRECT_M = _f("RS_GREET_REDIRECT_M", 0.60)  # person within this range -> jump to Greet
# reception -> booth remote enrollment (Orin Nano posts SFace embeddings here)
ENROLL_TOKEN = _s("RS_ENROLL_TOKEN", "rover-reception")  # shared secret; set the SAME on the Nano
# point cloud
CLOUD_STRIDE = int(_f("RS_CLOUD_STRIDE", 2))
CLOUD_ZMIN = _f("RS_CLOUD_ZMIN", 0.20)
CLOUD_ZMAX = _f("RS_CLOUD_ZMAX", 4.5)               # include the room / background
# Room mapping + dynamic people: build a persistent per-pixel room map (the
# farthest stable depth per pixel), then render anything nearer than it as live
# foreground (people) slotted into that space.
CLOUD_ACCUM = int(_f("RS_CLOUD_ACCUM", 1))
CLOUD_DYN_MARGIN = _f("RS_CLOUD_DYN_MARGIN", 0.08)  # nearer than the map by this => a person
# detect
DETECT_THRESH = _f("RS_DETECT_THRESH", 0.15)
DETECT_EVERY = int(_f("RS_DETECT_EVERY", 2))
DETECT_QUERY = _s("RS_DETECT_QUERY", "person, laptop, bottle, cup, cell phone, chair")
# attract mode: booth presence via motion (robust to static walls/tables in view)
PRESENCE_MOTION = _f("RS_PRESENCE_MOTION", 3.5)   # mean abs grey diff (0-255) = someone moving
ANALYTICS_GAP = _f("RS_ANALYTICS_GAP", 12.0)      # s of no-presence that closes a visit
ANALYTICS_ENTER = _f("RS_ANALYTICS_ENTER", 1.5)   # s of presence before a visit is counted (ignore blips)
# Platform panel: live Jetson stats + static stack labels (overridable via env)
JETPACK = _s("RS_JETPACK", "Jetson AGX Orin · JetPack 6.2 (L4T R36.4.7)")
CUDA_VER = _s("RS_CUDA", "12.6.3")
TRT_VER = _s("RS_TRT", "10.4.0")
LLM_LABEL = _s("RS_LLM_LABEL", "gemma3 · Ollama")

GREEN=(80,200,80); AMBER=(0,170,255); RED=(60,60,235)
CYAN=(230,210,40); WHITE=(245,245,245); DARK=(25,25,25)
NV=(20,180,60)  # NVIDIA green (BGR)


# --------------------------------------------------------------------------- #
# Shared state
# --------------------------------------------------------------------------- #
class Shared:
    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg = None
        self.seq = 0
        self.mode = START_MODE
        self.cloud_blob = b""          # packed point cloud for /pointcloud
        self.pending_embs = []         # multi-angle face embeddings awaiting a name
        self.pending_poses = []        # head-pose signatures already banked
        self.pending_since = 0.0
        self.last_face_id = None
        self.detect_labels = [s.strip() for s in DETECT_QUERY.split(",") if s.strip()]
        self.detect_dirty = True
        self.owl_ready = False
        self.status = {"mode": self.mode, "message": "starting…"}
        # booth telemetry (live badge + attract mode + watchdog)
        self.cam_ok = False
        self.fps = 0.0
        self.latency_ms = 0.0
        self.present = False
        self.last_present = time.time()

    def publish(self, jpeg):
        with self.cond:
            self.jpeg = jpeg
            self.seq += 1
            self.cond.notify_all()

    def wait(self, last, timeout=5.0):
        with self.cond:
            if self.seq == last:
                self.cond.wait(timeout)
            return self.jpeg, self.seq


SH = Shared()


# --------------------------------------------------------------------------- #
# Encrypted face store (vectors only, local, auto-wipe) -- see face_station.py
# --------------------------------------------------------------------------- #
import base64
import sqlite3
from cryptography.fernet import Fernet


class FaceStore:
    def __init__(self, data_dir, retention_hours):
        os.makedirs(data_dir, exist_ok=True)
        self.retention = retention_hours * 3600.0
        self._fernet = self._cipher(data_dir)
        self._lock = threading.Lock()
        self._c = sqlite3.connect(os.path.join(data_dir, "faces.db"),
                                  check_same_thread=False)
        self._c.execute("CREATE TABLE IF NOT EXISTS people("
                        "id INTEGER PRIMARY KEY AUTOINCREMENT,name TEXT,"
                        "emb BLOB,enrolled_at REAL,seen_at REAL,"
                        "company TEXT,email TEXT,phone TEXT)")
        # lead-capture columns for booth DBs created before they existed
        for col in ("company", "email", "phone"):
            try:
                self._c.execute(f"ALTER TABLE people ADD COLUMN {col} TEXT")
            except sqlite3.OperationalError:
                pass                                 # already there
        # PERSISTENT leads: contact details for sales follow-up. Deliberately NOT
        # subject to the retention purge or /wipe (those only clear the biometric
        # embeddings in `people`). The visitor gave these details so we can contact
        # them, so they are kept until explicitly exported/cleared.
        self._c.execute("CREATE TABLE IF NOT EXISTS leads("
                        "name TEXT, company TEXT, email TEXT, phone TEXT,"
                        "enrolled_at REAL, seen_at REAL, source TEXT)")
        # one-time backfill so anyone already enrolled becomes a persistent lead
        try:
            if self._c.execute("SELECT COUNT(*) FROM leads").fetchone()[0] == 0:
                self._c.execute(
                    "INSERT INTO leads(name,company,email,phone,enrolled_at,seen_at,source)"
                    " SELECT name, MAX(company), MAX(email), MAX(phone),"
                    " MIN(enrolled_at), MAX(seen_at), 'backfill' FROM people GROUP BY name")
        except Exception:
            pass
        self._c.commit()
        self.purge()                                 # purges `people` only; leads kept

    def _cipher(self, d):
        pw = os.environ.get("RS_FACE_PASSPHRASE")
        if pw:
            from cryptography.hazmat.primitives import hashes
            from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC
            sp = os.path.join(d, "salt.bin")
            salt = open(sp, "rb").read() if os.path.exists(sp) else os.urandom(16)
            if not os.path.exists(sp):
                open(sp, "wb").write(salt); os.chmod(sp, 0o600)
            kdf = PBKDF2HMAC(algorithm=hashes.SHA256(), length=32, salt=salt,
                             iterations=390000)
            return Fernet(base64.urlsafe_b64encode(kdf.derive(pw.encode())))
        kp = os.path.join(d, "key.bin")
        if os.path.exists(kp):
            key = open(kp, "rb").read()
        else:
            key = Fernet.generate_key(); open(kp, "wb").write(key); os.chmod(kp, 0o600)
        return Fernet(key)

    def purge(self):
        with self._lock:
            self._c.execute("DELETE FROM people WHERE enrolled_at < ?",
                            (time.time() - self.retention,))
            self._c.commit()

    def add(self, name, emb, company="", email="", phone=""):
        now = time.time()
        blob = self._fernet.encrypt(emb.astype(np.float32).tobytes())
        with self._lock:
            self._c.execute("INSERT INTO people(name,emb,enrolled_at,seen_at,company,email,phone)"
                            " VALUES(?,?,?,?,?,?,?)", (name, blob, now, now, company, email, phone))
            self._c.commit()

    def add_lead(self, name, company="", email="", phone="", source="booth"):
        """Persist ONE contact record per enrolled visitor for sales follow-up.
        Survives retention purge, /wipe, and restarts. Deduped by name+email+phone;
        a repeat check-in just refreshes seen_at. No biometric is stored here."""
        if not name:
            return
        now = time.time()
        with self._lock:
            cur = self._c.execute(
                "UPDATE leads SET seen_at=?, company=?, source=? WHERE lower(name)=lower(?)"
                " AND COALESCE(email,'')=? AND COALESCE(phone,'')=?",
                (now, company, source, name, email, phone))
            if cur.rowcount == 0:
                self._c.execute(
                    "INSERT INTO leads(name,company,email,phone,enrolled_at,seen_at,source)"
                    " VALUES(?,?,?,?,?,?,?)", (name, company, email, phone, now, now, source))
            self._c.commit()

    def leads(self):
        """All captured leads (name+company+email+phone+first/last seen+source) for the
        sales-follow-up CSV. PERSISTENT — not affected by retention or /wipe. No embeddings."""
        with self._lock:
            return self._c.execute(
                "SELECT name, COALESCE(company,''), COALESCE(email,''), COALESCE(phone,''),"
                " enrolled_at, seen_at, COALESCE(source,'') FROM leads ORDER BY enrolled_at").fetchall()

    def lead_count(self):
        with self._lock:
            return self._c.execute("SELECT COUNT(*) FROM leads").fetchone()[0]

    def all(self):
        self.purge()
        out = []
        with self._lock:
            rows = self._c.execute("SELECT id,name,emb FROM people").fetchall()
        for rid, name, blob in rows:
            try:
                out.append((rid, name,
                            np.frombuffer(self._fernet.decrypt(blob), np.float32)))
            except Exception:
                pass
        return out

    def touch(self, rid):
        with self._lock:
            self._c.execute("UPDATE people SET seen_at=? WHERE id=?",
                            (time.time(), rid)); self._c.commit()

    def forget(self, rid):
        with self._lock:
            self._c.execute("DELETE FROM people WHERE id=?", (rid,)); self._c.commit()

    def wipe(self):
        with self._lock:
            n = self._c.execute("DELETE FROM people").rowcount; self._c.commit()
        return n

    def count(self):
        with self._lock:
            return self._c.execute("SELECT COUNT(*) FROM people").fetchone()[0]

    def count_people(self):
        """Distinct enrolled people (each may have several angle embeddings)."""
        with self._lock:
            return self._c.execute("SELECT COUNT(DISTINCT name) FROM people").fetchone()[0]


# --------------------------------------------------------------------------- #
# Drawing helpers
# --------------------------------------------------------------------------- #
def label(img, text, org, scale=0.6, color=WHITE, thick=2, bg=DARK, pad=6):
    (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
    x, y = org
    cv2.rectangle(img, (x - pad, y - th - pad), (x + tw + pad, y + base + pad), bg, -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def fmt_len(m):
    cm = m * 100.0
    return f"{cm:4.1f} cm" if cm < 100 else f"{m:4.2f} m"


# --------------------------------------------------------------------------- #
# Per-mode processing
# --------------------------------------------------------------------------- #
def nearest(depth_m, roi):
    x0, y0, x1, y1 = roi
    v = depth_m[y0:y1, x0:x1]
    v = v[v > 0]
    return None if v.size < 50 else float(np.percentile(v, 2))


def region_distance(depth_m, box, shrink=0.25, lo=0.2, hi=3.0):
    """Median valid distance inside (a shrunk) pixel box, or None."""
    x, y, w, h = box
    mx, my = int(w*shrink), int(h*shrink)
    x0, y0 = max(0, x+mx), max(0, y+my)
    x1, y1 = min(WIDTH, x+w-mx), min(HEIGHT, y+h-my)
    if x1-x0 < 4 or y1-y0 < 4:
        return None
    sub = depth_m[y0:y1, x0:x1]
    v = sub[(sub > lo) & (sub < hi)]
    return float(np.median(v)) if v.size > 30 else None


class SafetyEngine:
    """Person-aware safety zone: find the nearest face, measure its distance,
    colour the zone, and flag a redirect to Greet when a person is very close."""
    def __init__(self):
        self.last = None
        self.close = 0

    def update(self, img, depth_m, fe):
        fe.det.setInputSize((WIDTH, HEIGHT))
        _, faces = fe.det.detect(img)
        box, dist, person = None, None, False
        if faces is not None and len(faces):
            f = max(faces, key=lambda r: r[2]*r[3])
            if f[2] >= MIN_FACE_PX*0.6:                    # accept approaching faces
                box = tuple(int(v) for v in f[:4])
                dist = region_distance(depth_m, box)
                person = dist is not None
        if dist is None:                                    # fall back to nearest object
            dist = nearest(depth_m, (int(WIDTH*0.18), int(HEIGHT*0.12),
                                     int(WIDTH*0.82), int(HEIGHT*0.95)))
        if dist is None:
            state, col, msg = "CLEAR", GREEN, "ZONE CLEAR"
        elif dist <= DANGER_M:
            state, col, msg = "DANGER", RED, f"TOO CLOSE  {fmt_len(dist)}"
        elif dist <= CAUTION_M:
            state, col, msg = "CAUTION", AMBER, f"APPROACHING  {fmt_len(dist)}"
        else:
            state, col, msg = "CLEAR", GREEN, f"CLEAR  {fmt_len(dist)}"
        redirect = None
        if person and dist is not None and dist <= GREET_REDIRECT_M:
            self.close += 1
            if self.close >= 2:
                redirect = "greet"
        else:
            self.close = 0
        self.last = dict(state=state, col=col, msg=msg, box=box, dist=dist,
                         person=person, redirect=redirect)
        SH.status = {"mode": "safety", "state": state, "dist": dist,
                     "person": person, "redirect": redirect}

    def draw(self, img):
        d = self.last
        if not d:
            cv2.rectangle(img, (0, 0), (WIDTH-1, HEIGHT-1), GREEN, 8)
            return
        cv2.rectangle(img, (0, 0), (WIDTH-1, HEIGHT-1), d["col"],
                      14 if d["state"] == "DANGER" else 8)
        if d["box"]:
            x, y, w, h = d["box"]
            cv2.rectangle(img, (x, y), (x+w, y+h), d["col"], 2)
            tag = ("PERSON " if d["person"] else "") + (fmt_len(d["dist"]) if d["dist"] else "")
            label(img, tag, (x, max(24, y-10)), 0.55, WHITE, 2, d["col"])
        label(img, d["state"], (14, 64), 0.9, WHITE, 2, d["col"])
        label(img, d["msg"], (14, 98), 0.6, WHITE, 2, DARK)
        if d["redirect"]:
            label(img, "greeting you…", (14, 132), 0.6, DARK, 2, GREEN)


_dim_ema = {}


def _ema(key, val, a=0.4):
    if val is None:
        _dim_ema.pop(key, None); return None
    prev = _dim_ema.get(key)
    cur = val if prev is None else prev + a*(val-prev)
    _dim_ema[key] = cur
    return cur


def skin_mask(bgr):
    """Rough skin mask (YCrCb) so hands holding the object are excluded."""
    y = cv2.cvtColor(bgr, cv2.COLOR_BGR2YCrCb)
    cr, cb = y[:, :, 1], y[:, :, 2]
    m = ((cr >= 133) & (cr <= 176) & (cb >= 77) & (cb <= 128)).astype(np.uint8)
    return cv2.dilate(m, np.ones((7, 7), np.uint8), iterations=2)


def looks_like_body(contour, zone_w, zone_h):
    """Cheap geometry gate: a genuinely held object is bounded inside the reference
    frame; a person standing in it instead FILLS the frame, spans its full height,
    and runs off the bottom edge (the torso continues below view). Returns True when
    the segment looks like the person, so Inspect/Box Vol can skip it."""
    x, y, w, h = cv2.boundingRect(contour)
    area = cv2.contourArea(contour)
    fill = area / max(zone_w * zone_h, 1.0)
    touches_bottom = (y + h) >= (zone_h - 4)   # runs off the bottom = torso continues below
    touches_top = y <= 4
    spans_tall = h >= 0.85 * zone_h
    spans_wide = w >= 0.90 * zone_w
    # A held object is bounded inside the frame; a person runs off the bottom edge.
    # Require touches_bottom for the tall/wide rejections so a big box held in mid-air
    # (which leaves space below it) is NOT mistaken for a torso.
    return (fill > OBJ_MAX_FILL) or (touches_top and touches_bottom) or \
        (touches_bottom and (spans_tall or spans_wide or h > 0.6 * zone_h))


class InspectEngine:
    """Inspect tab = dimensioning + REAL depth-based surface QC. Segments the held
    object (hands removed), names it with the on-device open-vocab model, measures
    it, and analyses its depth surface: fits a plane (tilt-corrected) for flatness
    in mm and flags genuine deformations (dents/bumps) that exceed the depth noise
    floor. Fine scratches are below the D455's depth resolution, so they are not
    claimed -- this reports only what depth can truly measure."""
    def __init__(self):
        self.zone = (0, 0, WIDTH, HEIGHT)
        self.last = None
        self.owl = None            # set in main(); used to name the object
        self.tick = 0
        self.obj_label = None
        self.note = ""             # on-screen hint (e.g. "move it away from your body")
        self._body_hold = 0        # frames to keep rejecting after NanoOWL sees a body

    def _reset(self, note=""):
        _ema("w", None); _ema("h", None); _ema("d", None)
        self.last = None
        self.obj_label = None
        self.note = note
        SH.status = {"mode": "dimension", "measuring": False, "note": note}

    def _surface(self, sub, objmask):
        """Fit a plane to the object face (removes tilt) -> flatness (mm rms) and
        real local deformations: regions deviating > DENT_MM from that plane."""
        ys, xs = np.where(objmask > 0)
        z = sub[ys, xs]
        good = z > 0
        ys, xs, z = ys[good], xs[good], z[good]
        if xs.size < 200:
            return 0.0, [], 0.0
        A = np.column_stack([xs.astype(np.float64), ys.astype(np.float64), np.ones(xs.size)])
        coef, *_ = np.linalg.lstsq(A, z.astype(np.float64), rcond=None)
        resid_mm = (z - A @ coef) * 1000.0
        flat = float(np.sqrt(np.mean(resid_mm*resid_mm)))
        resimg = np.zeros(sub.shape, np.float32)
        resimg[ys, xs] = resid_mm.astype(np.float32)
        dm = ((np.abs(resimg) > DENT_MM) & (objmask > 0)).astype(np.uint8)
        dm = cv2.morphologyEx(dm, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, stats, _ = cv2.connectedComponentsWithStats(dm, 8)
        dents, maxdev = [], 0.0
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] < DENT_MIN_AREA:
                continue
            comp = lab == i
            pk = float(np.max(np.abs(resimg[comp])))
            maxdev = max(maxdev, pk)
            cc, _ = cv2.findContours(comp.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cc:
                sign = "dent" if float(np.mean(resimg[comp])) > 0 else "bump"
                dents.append((cc[0].reshape(-1, 2), sign, pk))
        return flat, dents, maxdev

    def process(self, img, depth_m, intr):
        zw, zh = int(WIDTH*MEASURE_ZONE_W), int(HEIGHT*MEASURE_ZONE_H)
        zx0, zy0 = (WIDTH-zw)//2, (HEIGHT-zh)//2
        zx1, zy1 = zx0+zw, zy0+zh
        self.zone = (zx0, zy0, zx1, zy1)
        sub = depth_m[zy0:zy1, zx0:zx1]
        subc = img[zy0:zy1, zx0:zx1]
        valid = sub > 0
        skin = skin_mask(subc) if MEASURE_IGNORE_SKIN else np.zeros(sub.shape, np.uint8)
        fg = valid & (sub >= MEASURE_MIN_M) & (sub <= MEASURE_MAX_M) & (skin == 0)
        if int(fg.sum()) < 300:
            return self._reset()
        near = float(np.percentile(sub[fg], 2))
        back = float(np.percentile(sub[valid], 85))
        band = min(DEPTH_BAND_M, 0.6*(back-near)) if back-near > 0.05 else DEPTH_BAND_M
        objmask = (valid & (skin == 0) & (sub >= near-0.02) & (sub <= near+band)).astype(np.uint8)
        k = np.ones((5, 5), np.uint8)
        objmask = cv2.morphologyEx(objmask, cv2.MORPH_OPEN, k)
        objmask = cv2.morphologyEx(objmask, cv2.MORPH_CLOSE, k)
        cnts, _ = cv2.findContours(objmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return self._reset()
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < MIN_OBJ_FRAC*sub.size:
            return self._reset()
        # reject the person: a torso fills / spans / runs off the reference frame
        if looks_like_body(c, zw, zh):
            return self._reset("hold the object away from your body")
        if self._body_hold > 0:                    # NanoOWL saw a body very recently
            self._body_hold -= 1
            return self._reset("hold the object away from your body")
        single = np.zeros_like(objmask); cv2.drawContours(single, [c], -1, 1, -1)
        objmask = objmask & single                       # restrict to the one object
        x, y, w, h = cv2.boundingRect(c)
        od = sub[y:y+h, x:x+w][objmask[y:y+h, x:x+w] > 0]
        if od.size < 50:
            return self._reset()
        d = float(np.median(od))
        bp = cv2.boxPoints(cv2.minAreaRect(c))

        def dp(px, py):
            return np.array(rs.rs2_deproject_pixel_to_point(intr, [float(px+zx0), float(py+zy0)], d))
        p = [dp(px, py) for px, py in bp]
        wm, hm = float(np.linalg.norm(p[0]-p[1])), float(np.linalg.norm(p[1]-p[2]))
        if hm > wm:
            wm, hm = hm, wm
        wm, hm, dd = _ema("w", wm), _ema("h", hm), _ema("d", d)

        # real depth surface QC (flatness + deformations, in mm)
        flat, dents, maxdev = 0.0, [], 0.0
        if INSPECT_ENABLE:
            try:
                flat, dents, maxdev = self._surface(sub, objmask)
            except Exception:
                pass
        # name what's held (and veto the person) via the on-device open-vocab model
        self.tick += 1
        if self.owl is not None and self.owl.predictor is not None and self.tick % 2 == 0:
            crop = img[zy0+y:zy0+y+h, zx0+x:zx0+x+w]
            if crop.size > 0:
                try:
                    r = self.owl.classify(crop)   # (label, score, is_body)
                    if r and r[2] and r[1] >= BODY_REJECT_SCORE:
                        self._body_hold = 4        # it's you / your clothing -> reject
                        return self._reset("hold the object away from your body")
                    if r and not r[2] and r[1] >= 0.08:
                        self.obj_label = r[0]; self._body_hold = 0
                except Exception:
                    pass
        verdict = "PASS" if (not dents and maxdev <= INSPECT_TOL_MM) else "FLAG"
        off = np.array([zx0, zy0], np.int32)
        self.last = dict(
            box=(bp.astype(np.int32) + off),
            outline=(c.reshape(-1, 2) + off),
            label=self.obj_label, w=wm, h=hm, d=dd, flat=flat, maxdev=maxdev,
            dents=[(cnt + off, sign, mm) for cnt, sign, mm in dents],
            verdict=verdict)
        SH.status = {"mode": "dimension", "measuring": True, "object": self.obj_label,
                     "w_cm": round(wm*100, 1), "h_cm": round(hm*100, 1),
                     "dist_cm": round(dd*100, 1), "flatness_mm": round(flat, 2),
                     "deformations": len(dents), "max_dev_mm": round(maxdev, 1),
                     "verdict": verdict}

    def draw(self, img):
        zx0, zy0, zx1, zy1 = self.zone
        cv2.rectangle(img, (zx0, zy0), (zx1, zy1), CYAN, 1)
        d = self.last
        if not d:
            msg = self.note or "hold an object in the box"
            col = AMBER if self.note else CYAN
            label(img, msg, (zx0+8, zy1-12), 0.5, col, 2, DARK)
            return
        cv2.polylines(img, [d["outline"]], True, (90, 220, 90), 2)
        cv2.drawContours(img, [d["box"]], 0, CYAN, 2)
        if d["dents"]:                                   # translucent fill + outline + mm
            ov = img.copy()
            for cnt, sign, _ in d["dents"]:
                cv2.drawContours(ov, [cnt], -1, RED if sign == "dent" else AMBER, -1)
            cv2.addWeighted(ov, 0.35, img, 0.65, 0, img)
            for cnt, sign, mm in d["dents"]:
                cv2.drawContours(img, [cnt], -1, RED if sign == "dent" else AMBER, 2)
                m = cnt.mean(axis=0).astype(int)
                label(img, f"{mm:.0f}mm", (int(m[0]), int(m[1])), 0.45, WHITE, 1,
                      RED if sign == "dent" else AMBER)
        obj = (d["label"] or "object").upper()
        lines = [f"OBJECT    {obj}",
                 f"SIZE      {fmt_len(d['w'])} x {fmt_len(d['h'])}",
                 f"DISTANCE  {fmt_len(d['d'])}",
                 f"FLATNESS  {d['flat']:.2f} mm rms",
                 f"DEFORM    {len(d['dents'])} region(s), max {d['maxdev']:.1f} mm"]
        for i, t in enumerate(lines):
            label(img, t, (14, 30+i*24), 0.5, WHITE, 1, DARK)
        vc = GREEN if d["verdict"] == "PASS" else RED
        label(img, f"VERDICT: {d['verdict']}", (14, 30+len(lines)*24+6), 0.65, WHITE, 2, vc)


def check_liveness(depth_m, box):
    x, y, w, h = [int(v) for v in box]
    mx, my = int(w*0.2), int(h*0.2)
    x0, y0 = max(0, x+mx), max(0, y+my)
    x1, y1 = min(WIDTH, x+w-mx), min(HEIGHT, y+h-my)
    if x1-x0 < 8 or y1-y0 < 8:
        return False, None
    sub = depth_m[y0:y1, x0:x1]
    valid = sub[(sub > LIVE_MIN_M) & (sub < LIVE_MAX_M+0.5)]
    if valid.size < 100:
        return False, None
    dist = float(np.median(valid))
    if not (LIVE_MIN_M <= dist <= LIVE_MAX_M):
        return False, dist
    ch, cw = sub.shape
    centre = sub[int(ch*0.35):int(ch*0.65), int(cw*0.35):int(cw*0.65)]
    centre = centre[(centre > LIVE_MIN_M) & (centre < LIVE_MAX_M+0.5)]
    ring = sub.copy(); ring[int(ch*0.35):int(ch*0.65), int(cw*0.35):int(cw*0.65)] = 0
    ring = ring[(ring > LIVE_MIN_M) & (ring < LIVE_MAX_M+0.5)]
    if centre.size < 20 or ring.size < 20:
        return False, dist
    relief = float(np.median(ring) - np.median(centre))
    spread = float(np.std(valid))
    return (relief >= LIVE_RELIEF_M or spread >= LIVE_STD_M), dist


def cosine(a, b):
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    return -1.0 if na == 0 or nb == 0 else float(np.dot(a, b)/(na*nb))


def face_pose(f):
    """Cheap head-pose signature from YuNet landmarks: nose offset from the
    eye-centre, normalised by eye distance -> (yaw-ish, pitch-ish)."""
    rex, rey, lex, ley, nx, ny = f[4], f[5], f[6], f[7], f[8], f[9]
    ecx, ecy = (rex+lex)/2.0, (rey+ley)/2.0
    ed = max(1.0, float(((lex-rex)**2 + (ley-rey)**2) ** 0.5))
    return ((nx-ecx)/ed, (ny-ecy)/ed)


def pose_dist(a, b):
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2) ** 0.5


class FaceEngine:
    def __init__(self):
        self.det = cv2.FaceDetectorYN_create(
            os.path.join(MODEL_DIR, "yunet.onnx"), "", (WIDTH, HEIGHT), 0.8, 0.3, 50)
        self.rec = cv2.FaceRecognizerSF_create(os.path.join(MODEL_DIR, "sface.onnx"), "")
        self.stable = 0
        self.last_faces = []   # cached, redrawn every frame to avoid flicker

    def process(self, img, depth_m, store):
        self.det.setInputSize((WIDTH, HEIGHT))
        _, faces = self.det.detect(img)
        known = store.all()
        primary_i, primary_a = -1, 0
        parsed = []
        if faces is not None:
            for i, f in enumerate(faces):
                x, y, w, h = f[0], f[1], f[2], f[3]
                if w < MIN_FACE_PX:
                    continue
                if w*h > primary_a:
                    primary_a, primary_i = w*h, i
                parsed.append((i, f, (x, y, w, h)))
        results = []
        # Pass 1: liveness + recognition for every face in view.
        info = []   # (i, f, box, live, dist, name, feat)
        for i, f, box in parsed:
            live, dist = check_liveness(depth_m, box)
            name, rid, feat = None, None, None
            if live:
                feat = self.rec.feature(self.rec.alignCrop(img, f)).flatten()
                best = COSINE_MATCH
                for kid, kname, kemb in known:
                    s = cosine(feat, kemb)
                    if s >= best:
                        best, name, rid = s, kname, kid
                if rid is not None:
                    store.touch(rid); SH.last_face_id = rid
            info.append((i, f, box, live, dist, name, feat))

        # MULTI-PERSON GUARD: only enrol a NEW person when EXACTLY ONE live, unknown
        # face is in frame. With two unknowns the largest one flips between them and
        # their embeddings would be merged into a single record. (Known faces are still
        # greeted no matter how many people are present.)
        unknown = [t for t in info if t[3] and t[5] is None]
        target_i = unknown[0][0] if len(unknown) == 1 else -1
        if target_i == -1:
            self.stable = 0

        for (i, f, box, live, dist, name, feat) in info:
            if i == target_i:
                self.stable += 1
                if self.stable >= ENROLL_STABLE:
                    # Bank multiple angles as the person naturally moves — a new view
                    # is kept only when the head pose changed enough.
                    pose = face_pose(f)
                    with SH.cond:
                        if SH.pending_since == 0.0:
                            SH.pending_since = time.time()
                            SH.pending_embs = []; SH.pending_poses = []
                        # CONSISTENCY GUARD: if the enrolling face stops matching the
                        # views banked so far, a different person replaced them -> start
                        # over with the current person rather than merging the two.
                        if SH.pending_embs and cosine(feat, np.mean(SH.pending_embs, axis=0)) < SAME_FACE_MIN:
                            SH.pending_embs = [feat.copy()]; SH.pending_poses = [pose]
                        elif len(SH.pending_embs) < ENROLL_VIEWS and (
                                not SH.pending_poses or
                                min(pose_dist(pose, q) for q in SH.pending_poses) >= POSE_MIN_DELTA):
                            SH.pending_embs.append(feat.copy())
                            SH.pending_poses.append(pose)
            x, y, w, h = [int(v) for v in box]
            if not live:
                col, tag, tc = AMBER, "PHOTO? (not live)", WHITE
            elif name:
                col, tag, tc = GREEN, f"Welcome back, {name}!", WHITE
            else:
                col, tag, tc = CYAN, "new visitor", DARK
            results.append({"box": (x, y, w, h), "col": col, "tag": tag,
                            "tc": tc, "dist": dist})
        with SH.cond:
            if SH.pending_since and time.time()-SH.pending_since > 30:
                SH.pending_embs = []; SH.pending_poses = []; SH.pending_since = 0.0
        self.last_faces = results
        SH.status = {"mode": "face", "known": store.count_people(),
                     "pending": len(SH.pending_embs) > 0, "multi": len(unknown) > 1,
                     "captured": len(SH.pending_embs), "views": ENROLL_VIEWS}

    def draw(self, img):
        for r in self.last_faces:
            x, y, w, h = r["box"]
            cv2.rectangle(img, (x, y), (x+w, y+h), r["col"], 2)
            label(img, r["tag"], (x, max(24, y-10)), 0.6, r["tc"], 2, r["col"])
            if r["dist"]:
                label(img, f"{r['dist']*100:.0f} cm", (x, y+h+22), 0.5, WHITE, 2, DARK)
        label(img, "FACE GREETING (depth-verified)", (14, 64), 0.55, DARK, 2, CYAN)
        n = len(SH.pending_embs)
        if n:
            label(img, f"scanning face - {n}/{ENROLL_VIEWS} angles (move gently)",
                  (14, 96), 0.55, DARK, 2, GREEN)


class BoxScanEngine:
    """Rotation-scan box / parcel dimensioner. As the box is slowly rotated, each
    time a face is square-on (fronto-parallel + rectangular) its two true edge
    lengths are sampled; the samples are clustered into the box's distinct
    dimensions -> L x W x H + volume. Rectangular cuboids only."""
    def __init__(self):
        self.owl = None        # set in main(); used to veto the person/clothing
        self.reset()

    def reset(self):
        self.samples = []      # edge lengths (m) from square-on faces
        self.faces = 0
        self.last = None
        self.dims = None       # (L,W,H) metres
        self.vol = None        # m^3
        self.empty = 0         # consecutive frames with nothing in the zone
        self.locked_at = None  # when a full L/W/H first locked (for the hold-then-reset)
        self.tick = 0
        self.note = ""         # on-screen hint (e.g. "hold the box away from your body")
        self._body_hold = 0    # frames to keep vetoing after NanoOWL sees a body

    def _clusters(self):
        cl = []
        for v in sorted(self.samples):
            for g in cl:
                if abs(v - np.median(g)) / max(np.median(g), 1e-6) <= BOX_CLUSTER_TOL:
                    g.append(v); break
            else:
                cl.append([v])
        return sorted(((float(np.median(g)), len(g)) for g in cl), key=lambda x: -x[1])

    def _segment(self, sub, subc, valid, skin):
        """Object mask + near distance. Depth slab when depth is dense (matte);
        colour grabCut (seeded by the sparse depth) when it is holey (glossy/bright)
        so the outline still comes through. Returns (objmask, near) or (None, None)."""
        fgd = valid & (skin == 0) & (sub >= MEASURE_MIN_M) & (sub <= MEASURE_MAX_M)
        if int(fgd.sum()) < 150:
            return None, None
        near = float(np.percentile(sub[fgd], 2))
        back = float(np.percentile(sub[valid], 85)) if int(valid.sum()) > 50 else near+0.3
        band = min(DEPTH_BAND_M, 0.6*(back-near)) if back-near > 0.05 else DEPTH_BAND_M
        slab = (fgd & (sub <= near+band)).astype(np.uint8)
        cpx = 15 if BOX_RGB_ASSIST else 5      # bigger close bridges glossy depth holes
        slab = cv2.morphologyEx(slab, cv2.MORPH_CLOSE, np.ones((cpx, cpx), np.uint8))
        slab = cv2.morphologyEx(slab, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        return slab, near

    def process(self, img, depth_m, intr):
        zw, zh = int(WIDTH*MEASURE_ZONE_W), int(HEIGHT*MEASURE_ZONE_H)
        zx0, zy0 = (WIDTH-zw)//2, (HEIGHT-zh)//2
        zx1, zy1 = zx0+zw, zy0+zh
        sub = depth_m[zy0:zy1, zx0:zx1]; subc = img[zy0:zy1, zx0:zx1]
        valid = sub > 0
        skin = skin_mask(subc) if MEASURE_IGNORE_SKIN else np.zeros(sub.shape, np.uint8)
        fg = valid & (sub >= MEASURE_MIN_M) & (sub <= MEASURE_MAX_M) & (skin == 0)
        if self.locked_at is not None and time.time() - self.locked_at >= BOX_HOLD_SEC:
            self.reset()                 # showed the volume long enough -> ready for next box
        present = int(fg.sum()) >= 300
        if not present:
            self.empty += 1
            if self.empty >= BOX_EMPTY_RESET and (self.samples or self.dims):
                self.reset()             # box removed -> clear values for the next box
        else:
            self.empty = 0
        info = dict(zone=(zx0, zy0, zx1, zy1), box=None, outline=None, face=None,
                    square=False, faces=self.faces, dims=self.dims, vol=self.vol,
                    cents=self._clusters(), note="")
        if present:
            objmask, near = self._segment(sub, subc, valid, skin)
            cnts = []
            if objmask is not None:
                cnts, _ = cv2.findContours(objmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts:
                c = max(cnts, key=cv2.contourArea)
                area = cv2.contourArea(c)
                rect = cv2.minAreaRect(c); (rw, rh) = rect[1]
                if area >= MIN_OBJ_FRAC*sub.size and rw > 2 and rh > 2:
                    x, y, w, h = cv2.boundingRect(c)
                    # veto the person: torso geometry, or NanoOWL (throttled) seeing a
                    # body/clothing crop, means this is the visitor -> don't measure it
                    self.tick += 1
                    body = looks_like_body(c, zw, zh)
                    if not body and self.owl is not None and self.owl.predictor is not None and self.tick % 3 == 0:
                        crop = img[zy0+y:zy0+y+h, zx0+x:zx0+x+w]
                        if crop.size > 0:
                            try:
                                rr = self.owl.classify(crop)   # (label, score, is_body)
                                if rr and rr[2] and rr[1] >= BODY_REJECT_SCORE:
                                    self._body_hold = 5
                                elif rr and not rr[2]:
                                    self._body_hold = 0
                            except Exception:
                                pass
                    if self._body_hold > 0:
                        self._body_hold -= 1; body = True
                    if body:
                        info["note"] = "hold the box away from your body"
                    else:
                        fd = sub[y:y+h, x:x+w][objmask[y:y+h, x:x+w] > 0]
                        fd = fd[(fd > 0.1) & (fd < 5.0)]
                        d = float(np.median(fd)) if fd.size >= 20 else near
                        bp = cv2.boxPoints(rect)

                        def dp(px, py):
                            return np.array(rs.rs2_deproject_pixel_to_point(
                                intr, [float(px+zx0), float(py+zy0)], d))
                        p = [dp(px, py) for px, py in bp]
                        e1 = float(np.linalg.norm(p[0]-p[1]))
                        e2 = float(np.linalg.norm(p[1]-p[2]))
                        diag = max((e1*e1 + e2*e2) ** 0.5, 1e-3)
                        rectangularity = area / max(rw*rh, 1.0)
                        # Square-on needs depth across the face; glossy faces with too
                        # little depth are still shown/outlined but not banked.
                        if fd.size >= 40:
                            spread = float(np.percentile(fd, 95) - np.percentile(fd, 5))
                            square = (rectangularity >= BOX_RECT_MIN) and (spread/diag < BOX_TILT_RATIO)
                        else:
                            square = False
                        info["box"] = bp.astype(np.int32) + [zx0, zy0]
                        info["outline"] = c.reshape(-1, 2) + [zx0, zy0]
                        info["face"] = (e1, e2)
                        info["square"] = square
                        if square:
                            self.samples += [e1, e2]
                            if len(self.samples) > 400:
                                self.samples = self.samples[-400:]
                            self.faces += 1
                            info["faces"] = self.faces
                            cents = self._clusters(); info["cents"] = cents
                            locked = sorted((c0 for c0, n in cents if n >= BOX_LOCK_SAMPLES),
                                            reverse=True)[:3]
                            if len(locked) >= 3:
                                self.dims = tuple(locked[:3])
                                self.vol = locked[0]*locked[1]*locked[2]
                                info["dims"] = self.dims; info["vol"] = self.vol
                                if self.locked_at is None:
                                    self.locked_at = time.time()
        self.last = info
        distinct = len([1 for _, n in info["cents"] if n >= BOX_LOCK_SAMPLES])
        st = {"mode": "boxscan", "faces": self.faces, "distinct": distinct, "locked": False}
        if self.dims:
            L, W, H = self.dims
            st.update({"locked": True, "L_cm": round(L*100, 1), "W_cm": round(W*100, 1),
                       "H_cm": round(H*100, 1), "volume_l": round(self.vol*1000, 2),
                       "volume_cm3": round(self.vol*1e6)})
            if self.locked_at is not None:
                st["hold_left"] = round(max(0, BOX_HOLD_SEC - (time.time() - self.locked_at)), 1)
        SH.status = st

    def draw(self, img):
        info = self.last
        zx0, zy0, zx1, zy1 = info["zone"] if info else (0, 0, WIDTH, HEIGHT)
        cv2.rectangle(img, (zx0, zy0), (zx1, zy1), CYAN, 1)
        if info and info["box"] is not None:
            col = GREEN if info["square"] else AMBER
            cv2.polylines(img, [info["outline"]], True, (90, 220, 90), 2)
            cv2.drawContours(img, [info["box"]], 0, col, 2)
            if info["face"]:
                e1, e2 = info["face"]
                tag = f"face {e1*100:.1f} x {e2*100:.1f} cm  " + ("[captured]" if info["square"] else "hold square-on")
                label(img, tag, (zx0+8, zy0+24), 0.5, DARK if info["square"] else WHITE, 2, col)
        label(img, "BOX VOLUME SCAN", (14, 30), 0.55, DARK, 2, CYAN)
        if info and info.get("note"):
            label(img, info["note"], (zx0+8, zy0+24), 0.5, WHITE, 2, AMBER)
        label(img, f"faces captured: {info['faces'] if info else 0}", (14, 58), 0.5, WHITE, 1, DARK)
        if info and info["dims"]:
            L, W, H = info["dims"]
            label(img, f"L {L*100:.1f}  W {W*100:.1f}  H {H*100:.1f} cm", (14, 86), 0.6, DARK, 2, GREEN)
            label(img, f"VOLUME  {self.vol*1000:.2f} L  ({self.vol*1e6:.0f} cm3)", (14, 116), 0.65, DARK, 2, GREEN)
            if self.locked_at is not None:
                left = max(0, BOX_HOLD_SEC - (time.time() - self.locked_at))
                label(img, f"resetting for next box in {left:.0f}s", (14, 144), 0.5, WHITE, 1, DARK)
        else:
            nd = len([1 for _, n in (info["cents"] if info else []) if n >= BOX_LOCK_SAMPLES])
            label(img, "slowly rotate the box to show each side", (14, 86), 0.5, WHITE, 1, DARK)
            if nd:
                label(img, f"dimensions found: {nd}/3", (14, 112), 0.55, DARK, 2, AMBER)
            else:
                label(img, "dimensions found: 0/3", (14, 112), 0.5, WHITE, 1, DARK)


# ---- point cloud ----
def depth_heatmap(depth_m, color):
    """Colourised depth (turbo, near=warm) blended over dim RGB -> a full-frame
    'RGB + depth' view for the 3D-scan tab's right panel (no ugly black voids)."""
    lo, hi = 0.3, 3.0
    valid = (depth_m > lo) & (depth_m < hi)
    d = np.clip((depth_m - lo) / max(hi - lo, 1e-3), 0.0, 1.0)
    norm = np.zeros(depth_m.shape, np.uint8)
    norm[valid] = (255.0 * (1.0 - d[valid])).astype(np.uint8)
    heat = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
    out = color.astype(np.float32) * 0.4
    out[valid] = 0.72 * heat[valid] + 0.28 * color[valid]
    return out.astype(np.uint8)


# Persistent room map (per-pixel farthest-stable depth) + live dynamic foreground.
_MAP = {"bg": None, "col": None, "mapped": 0, "dyn": 0}


def cloud_reset():
    _MAP["bg"] = None; _MAP["col"] = None; _MAP["mapped"] = 0; _MAP["dyn"] = 0


def build_cloud(depth_m, color, intr):
    S = CLOUD_STRIDE
    small = depth_m[::S, ::S]
    scol = color[::S, ::S][:, :, ::-1]                 # strided RGB
    h, w = small.shape
    valid = (small > CLOUD_ZMIN) & (small < CLOUD_ZMAX)
    if _MAP["bg"] is None or _MAP["bg"].shape != small.shape:
        _MAP["bg"] = np.zeros((h, w), np.float32)
        _MAP["col"] = np.zeros((h, w, 3), np.uint8)
    bg, bgc = _MAP["bg"], _MAP["col"]
    if CLOUD_ACCUM:
        # Room map = farthest stable return per pixel (a person is nearer, so the
        # room behind is recovered as they move); also fill any empty pixel now.
        upd = valid & ((small > bg) | (bg == 0))
        bg[upd] = small[upd]; bgc[upd] = scol[upd]
        dyn = valid & (bg > 0) & (small < bg - CLOUD_DYN_MARGIN)   # people in front
    else:
        bg[:] = 0; bg[valid] = small[valid]; bgc[valid] = scol[valid]
        dyn = np.zeros_like(valid)

    def deproj(ys, xs, z):
        gx = xs * S; gy = ys * S
        X = (gx - intr.ppx) / intr.fx * z
        Y = (gy - intr.ppy) / intr.fy * z
        return np.stack([X, -Y, -z], axis=-1).astype(np.float32)

    bys, bxs = np.where(bg > 0)                          # static room map
    sp = deproj(bys, bxs, bg[bys, bxs]) if bxs.size else np.empty((0, 3), np.float32)
    sc = bgc[bys, bxs] if bxs.size else np.empty((0, 3), np.uint8)
    dys, dxs = np.where(dyn)                             # live people, slotted in
    dp = deproj(dys, dxs, small[dys, dxs]) if dxs.size else np.empty((0, 3), np.float32)
    dc = scol[dys, dxs] if dxs.size else np.empty((0, 3), np.uint8)
    _MAP["mapped"] = int(bxs.size); _MAP["dyn"] = int(dxs.size)
    out_pts = np.concatenate([sp, dp]).astype(np.float32)
    out_cols = np.ascontiguousarray(np.concatenate([sc, dc]).astype(np.uint8))
    n = out_pts.shape[0]
    return struct.pack("<I", n) + out_pts.tobytes() + out_cols.tobytes()


# ---- open-vocab detection (NanoOWL) ----
class OwlEngine:
    def __init__(self):
        self.predictor = None
        self.encodings = None
        self.labels = []
        self.last_dets = []   # cached, redrawn every frame to avoid flicker
        self._cls_enc = None   # cached encodings for the object+body classify vocabulary
        self._cls_vocab = None

    def load(self):
        from nanoowl.owl_predictor import OwlPredictor
        self.predictor = OwlPredictor(image_encoder_engine=OWL_ENGINE)
        SH.owl_ready = True
        print("[station] NanoOWL ready", flush=True)

    def ensure_text(self):
        if SH.detect_dirty and self.predictor is not None:
            self.labels = list(SH.detect_labels)
            self.encodings = self.predictor.encode_text(self.labels) if self.labels else None
            SH.detect_dirty = False

    def classify(self, bgr):
        """Classify a crop over product + body vocab. Returns (label, score, is_body):
        is_body=True means the crop matched a person/clothing class better than any
        product -> the depth slab grabbed the visitor, not a held object."""
        if self.predictor is None:
            return None
        import PIL.Image
        if self._cls_enc is None:
            self._cls_vocab = HANDHELD_VOCAB + BODY_VOCAB
            self._cls_enc = self.predictor.encode_text(self._cls_vocab)
        pil = PIL.Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        out = self.predictor.predict(image=pil, text=self._cls_vocab,
                                     text_encodings=self._cls_enc, threshold=0.02)
        sc = out.scores.detach().cpu().numpy()
        lb = out.labels.detach().cpu().numpy()
        if sc.size == 0:
            return None
        i = int(np.argmax(sc))
        label = self._cls_vocab[int(lb[i])]
        is_body = label in BODY_VOCAB
        return (label.replace("a ", "").replace("an ", ""), float(sc[i]), is_body)

    def detect(self, img_bgr, depth_m, intr):
        import PIL.Image
        self.ensure_text()
        dets = []
        if self.predictor is None or not self.labels or self.encodings is None:
            return dets
        pil = PIL.Image.fromarray(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
        out = self.predictor.predict(image=pil, text=self.labels,
                                     text_encodings=self.encodings, threshold=DETECT_THRESH)
        boxes = out.boxes.detach().cpu().numpy()
        labs = out.labels.detach().cpu().numpy()
        scores = out.scores.detach().cpu().numpy()
        keep = []  # light per-class NMS
        for cls in set(labs.tolist()):
            idx = np.where(labs == cls)[0]
            b = boxes[idx]
            if len(b) == 0:
                continue
            rects = [[int(x0), int(y0), int(x1-x0), int(y1-y0)] for x0, y0, x1, y1 in b]
            sc = [float(scores[i]) for i in idx]
            ni = cv2.dnn.NMSBoxes(rects, sc, DETECT_THRESH, 0.4)
            for j in (np.array(ni).flatten().tolist() if len(ni) else []):
                keep.append(idx[j])
        for i in keep:
            x0, y0, x1, y1 = [int(v) for v in boxes[i]]
            x0, y0 = max(0, x0), max(0, y0); x1, y1 = min(WIDTH, x1), min(HEIGHT, y1)
            name = self.labels[int(labs[i])]
            sub = depth_m[y0:y1, x0:x1]
            vv = sub[(sub > 0.15) & (sub < 6.0)]
            dist = float(np.median(vv)) if vv.size > 20 else None
            txt = f"{name} {scores[i]:.2f}" + (f"  {dist:.2f}m" if dist else "")
            dets.append({"box": (x0, y0, x1, y1), "label": name,
                         "score": round(float(scores[i]), 2), "dist": dist, "txt": txt})
        self.last_dets = dets
        return dets

    def draw(self, img):
        for d in self.last_dets:
            x0, y0, x1, y1 = d["box"]
            cv2.rectangle(img, (x0, y0), (x1, y1), NV, 2)
            label(img, d["txt"], (x0, max(22, y0-8)), 0.55, WHITE, 2, (40, 40, 40))


# --------------------------------------------------------------------------- #
# Capture loop
# --------------------------------------------------------------------------- #
# --------------------------------------------------------------------------- #
# Booth analytics: visitor sessions + dwell time, from the motion-presence sig.
# A "visit" opens on presence and closes after ANALYTICS_GAP seconds of none.
# --------------------------------------------------------------------------- #
_ANL = {"visitors": 0, "in_session": False, "counted": False, "start": 0.0, "last_seen": 0.0,
        "dwell_total": 0.0, "dwell_max": 0.0, "sessions": 0, "since": time.time()}


def analytics_update(present, now):
    a = _ANL
    if present:
        if not a["in_session"]:
            a["in_session"] = True; a["counted"] = False; a["start"] = now
        a["last_seen"] = now
        # count a visitor only once presence has lasted long enough (blips don't count),
        # and only once per visit — a lingering person is one visitor, not many.
        if not a["counted"] and (now - a["start"]) >= ANALYTICS_ENTER:
            a["counted"] = True; a["visitors"] += 1
    elif a["in_session"] and (now - a["last_seen"]) > ANALYTICS_GAP:
        if a["counted"]:
            dur = max(0.0, a["last_seen"] - a["start"])
            a["dwell_total"] += dur; a["sessions"] += 1
            a["dwell_max"] = max(a["dwell_max"], dur)
        a["in_session"] = False


def analytics_status(now):
    a = _ANL
    live = (now - a["start"]) if a["in_session"] else 0.0
    avg = (a["dwell_total"] / a["sessions"]) if a["sessions"] else 0.0
    return {"visitors": a["visitors"], "dwell_avg_s": round(avg, 1),
            "dwell_max_s": round(a["dwell_max"], 1), "live_dwell_s": round(live, 1)}


def analytics_reset():
    _ANL.update({"visitors": 0, "in_session": False, "counted": False, "start": 0.0,
                 "last_seen": 0.0, "dwell_total": 0.0, "dwell_max": 0.0, "sessions": 0,
                 "since": time.time()})


def _start_pipeline():
    """(Re)open the RealSense pipeline and return (pipeline, scale). Raises on failure.
    Kept separate from capture_loop so the booth watchdog can call it again after a
    USB drop without rebuilding the filters or losing frame-rate/telemetry state."""
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    profile = pipeline.start(cfg)
    dsen = profile.get_device().first_depth_sensor()
    scale = dsen.get_depth_scale()
    # Maximise depth coverage on hard (glossy/bright/textureless) surfaces:
    # projected IR texture on + full laser power + high-density preset (fills more
    # pixels). Specular reflections still can't be recovered -- see README notes.
    try:
        if dsen.supports(rs.option.emitter_enabled):
            dsen.set_option(rs.option.emitter_enabled, 1)
        if dsen.supports(rs.option.laser_power):
            dsen.set_option(rs.option.laser_power, dsen.get_option_range(rs.option.laser_power).max)
        if dsen.supports(rs.option.visual_preset):
            dsen.set_option(rs.option.visual_preset, float(rs.rs400_visual_preset.high_density))
        print("[station] depth tuned: emitter on, laser max, high-density preset", flush=True)
    except Exception as e:
        print(f"[station] depth option tuning skipped: {e}", flush=True)
    for _ in range(15):
        pipeline.wait_for_frames()
    return pipeline, scale


def capture_loop(store, face_engine, safety_engine, inspect_engine, box_engine, owl):
    align = rs.align(rs.stream.color)
    spatial = rs.spatial_filter()          # edge-preserving smoothing + small-hole fill
    spatial.set_option(rs.option.filter_magnitude, 2)
    spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
    spatial.set_option(rs.option.filter_smooth_delta, 20)
    spatial.set_option(rs.option.holes_fill, 2)
    temporal = rs.temporal_filter()
    hole = rs.hole_filling_filter(1)

    enc = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]

    def show_msg(title, sub=""):
        """Publish a themed placeholder frame so the stream stays alive (and the
        booth screen shows a friendly message) while the camera is (re)connecting."""
        img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        img[:] = (14, 11, 10)                          # --bg #0a0b0e in BGR
        (tw, _), _ = cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
        cv2.putText(img, title, ((WIDTH - tw) // 2, HEIGHT // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, WHITE, 2, cv2.LINE_AA)
        if sub:
            (sw, _), _ = cv2.getTextSize(sub, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.putText(img, sub, ((WIDTH - sw) // 2, HEIGHT // 2 + 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 144, 138), 1, cv2.LINE_AA)
        ok, jpeg = cv2.imencode(".jpg", img, enc)
        if ok:
            SH.publish(jpeg.tobytes())

    fi = 0; ft = time.monotonic(); fn = 0; fps = 0.0; last_purge = time.time()
    lat_ms = 0.0; prev_gray = None
    pipeline = None; scale = 1.0; miss = 0; attempt = 0

    while True:
        # ---- booth watchdog: (re)open the camera whenever it is not streaming ----
        if pipeline is None:
            attempt += 1
            try:
                show_msg("Connecting to camera…" if attempt == 1 else "Reconnecting camera…",
                         "Intel RealSense D455 · NVIDIA Jetson")
                print(f"[station] starting pipeline {WIDTH}x{HEIGHT}@{FPS} "
                      f"(attempt {attempt}) …", flush=True)
                pipeline, scale = _start_pipeline()
                SH.cam_ok = True; miss = 0
                print(f"[station] streaming on :{PORT}", flush=True)
            except Exception as e:
                SH.cam_ok = False
                SH.status = {"mode": SH.mode, "cam_ok": False, "message": "camera offline"}
                print(f"[station] camera open failed: {e}; retrying …", flush=True)
                time.sleep(1.5)
                continue

        # ---- grab a frame; a drop/unplug is caught here and triggers a reconnect ----
        t0 = time.monotonic()
        try:
            ok_f, frames = pipeline.try_wait_for_frames(2000)
        except Exception as e:
            ok_f, frames = False, None
            print(f"[station] frame error: {e}", flush=True)
        if not ok_f:
            miss += 1
            if miss >= 3:                              # camera really gone -> reconnect
                SH.cam_ok = False
                try:
                    pipeline.stop()
                except Exception:
                    pass
                pipeline = None
                show_msg("Reconnecting camera…", "check the USB cable")
            continue
        miss = 0

        try:
            aligned = align.process(frames)
            dframe = aligned.get_depth_frame()
            cframe = aligned.get_color_frame()
            if not dframe or not cframe:
                continue
            # Spatial filter (denser depth) is only worth its cost in the depth-heavy
            # modes; skip it in safety/face/detect to keep them at full frame rate.
            if SH.mode in ("cloud", "dimension", "boxscan"):
                dframe = spatial.process(dframe)
            dframe = temporal.process(dframe)
            dframe = hole.process(dframe)
            intr = dframe.profile.as_video_stream_profile().intrinsics
            depth_m = np.asanyarray(dframe.get_data()).astype(np.float32) * scale
            img = np.asanyarray(cframe.get_data()).copy()

            # booth presence via MOTION in the central region -> someone is here.
            # Robust to static backgrounds (a wall/table in view never counts as a
            # person). Computed on the clean colour frame, before any overlays.
            gsm = cv2.resize(cv2.cvtColor(
                img[HEIGHT // 4:HEIGHT * 3 // 4, WIDTH // 4:WIDTH * 3 // 4],
                cv2.COLOR_BGR2GRAY), (80, 60))
            motion = float(np.mean(cv2.absdiff(gsm, prev_gray))) if prev_gray is not None else 0.0
            prev_gray = gsm
            present = motion > PRESENCE_MOTION
            now_w = time.time()
            if present:
                SH.last_present = now_w
            SH.present = present
            analytics_update(present, now_w)     # visitor count + dwell time

            fi += 1
            mode = SH.mode

            if mode == "safety":
                if fi % 3 == 0:
                    safety_engine.update(img, depth_m, face_engine)
                safety_engine.draw(img)
            elif mode == "dimension":
                if fi % 3 == 0:
                    inspect_engine.process(img, depth_m, intr)
                inspect_engine.draw(img)   # redraw cached inspection every frame
            elif mode == "face":
                if fi % 3 == 0:
                    face_engine.process(img, depth_m, store)
                face_engine.draw(img)  # redraw cached faces every frame (no flicker)
            elif mode == "cloud":
                if fi % 2 == 0:
                    SH.cloud_blob = build_cloud(depth_m, img, intr)
                img = depth_heatmap(depth_m, img)   # right panel: RGB + depth heatmap
                total = (HEIGHT // CLOUD_STRIDE) * (WIDTH // CLOUD_STRIDE)
                cov = int(100 * _MAP["mapped"] / max(total, 1)) if _MAP["bg"] is not None else 0
                people = _MAP["dyn"] > 150
                label(img, "DEPTH MAP  (near = warm)", (14, 30), 0.55, WHITE, 2, DARK)
                label(img, f"room map {cov}%   people in frame: {'yes' if people else 'no'}",
                      (14, 58), 0.5, WHITE, 1, DARK)
                SH.status = {"mode": "cloud",
                             "points": struct.unpack("<I", SH.cloud_blob[:4])[0] if SH.cloud_blob else 0,
                             "map_cov": cov, "people": 1 if people else 0}
            elif mode == "detect":
                if owl is None:
                    label(img, "AI Detect runs on the AGX Orin (Ask the Camera)", (14, 64), 0.7, WHITE, 2, DARK)
                elif not SH.owl_ready:
                    label(img, "loading AI model…", (14, 64), 0.7, WHITE, 2, DARK)
                else:
                    if fi % DETECT_EVERY == 0:
                        owl.detect(img, depth_m, intr)
                        SH.status = {"mode": "detect", "query": SH.detect_labels,
                                     "dets": [{"label": d["label"], "score": d["score"],
                                               "dist": d["dist"]} for d in owl.last_dets]}
                    owl.draw(img)  # redraw cached boxes every frame (no flicker)
                label(img, "OPEN-VOCAB AI (NanoOWL)", (14, 30), 0.55, WHITE, 2, NV)
            elif mode == "boxscan":
                if fi % 2 == 0:
                    box_engine.process(img, depth_m, intr)
                box_engine.draw(img)
            else:
                label(img, "RealSense x NVIDIA Edge AI", (14, 64), 0.6, WHITE, 2, DARK)

            fn += 1
            now = time.monotonic()
            if now-ft >= 1.0:
                fps = fn/(now-ft); ft, fn = now, 0
            lat_ms = (now - t0) * 1000.0
            SH.fps = fps; SH.latency_ms = lat_ms
            label(img, "NVIDIA Jetson - on-device / offline", (14, HEIGHT-14), 0.45, WHITE, 1, NV)
            label(img, f"{fps:4.1f} fps", (WIDTH-92, HEIGHT-14), 0.45, WHITE, 1, DARK)

            # merge shared booth telemetry onto whatever the active mode published
            # (drives the live 'on-device' badge + attract-mode presence in the UI)
            if isinstance(SH.status, dict):
                SH.status = {**SH.status, "cam_ok": True, "fps": round(fps, 1),
                             "latency_ms": round(lat_ms, 1), "present": bool(present),
                             "idle_s": round(time.time() - SH.last_present, 1),
                             **analytics_status(time.time())}

            ok, jpeg = cv2.imencode(".jpg", img, enc)
            if ok:
                SH.publish(jpeg.tobytes())

            if time.time()-last_purge > 300:
                store.purge(); last_purge = time.time()
        except Exception as e:
            # a per-frame processing hiccup must never take the booth down
            print(f"[station] frame processing error: {e}", flush=True)
            continue


# --------------------------------------------------------------------------- #
# Live Jetson stats for the Platform panel -- GPU load, RAM, temperature.
# All readable inside the container from /proc and /sys (no tegrastats needed).
# --------------------------------------------------------------------------- #
_GPU_LOAD_PATHS = ["/sys/devices/platform/gpu.0/load", "/sys/devices/gpu.0/load"]


def _my_ip():
    """Best-effort primary LAN IP (works offline; UDP connect just picks the route)."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("192.168.255.255", 9)); ip = s.getsockname()[0]; s.close()
        return ip
    except Exception:
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            return "127.0.0.1"


def read_sysstats():
    st = {"gpu_pct": None, "ram_used_gb": None, "ram_total_gb": None, "ram_pct": None,
          "cpu_c": None, "gpu_c": None, "temp_c": None,
          "platform": JETPACK, "cuda": CUDA_VER, "trt": TRT_VER,
          "vision": "NanoOWL (TensorRT)", "llm": LLM_LABEL}
    for p in _GPU_LOAD_PATHS:                          # GPU load, per-mille 0-1000
        try:
            with open(p) as f:
                st["gpu_pct"] = round(int(f.read().strip()) / 10.0); break
        except Exception:
            pass
    try:
        mi = {}
        with open("/proc/meminfo") as f:
            for line in f:
                k, _, v = line.partition(":"); mi[k.strip()] = v.strip()
        tot = int(mi["MemTotal"].split()[0]); avail = int(mi["MemAvailable"].split()[0])
        st["ram_total_gb"] = round(tot / 1048576.0, 1)
        st["ram_used_gb"] = round((tot - avail) / 1048576.0, 1)
        st["ram_pct"] = round(100.0 * (tot - avail) / max(tot, 1))
    except Exception:
        pass
    hottest = 0.0
    try:
        import glob
        for z in glob.glob("/sys/class/thermal/thermal_zone*"):
            try:
                typ = open(os.path.join(z, "type")).read().strip()
                raw = open(os.path.join(z, "temp")).read().strip()
                if not raw:
                    continue
                c = int(raw) / 1000.0
                hottest = max(hottest, c)
                if typ == "cpu-thermal":
                    st["cpu_c"] = round(c, 1)
                elif typ == "gpu-thermal":
                    st["gpu_c"] = round(c, 1)
            except Exception:
                pass
        if hottest:
            st["temp_c"] = round(hottest, 1)
    except Exception:
        pass
    return st


# --------------------------------------------------------------------------- #
# HTTP server
# --------------------------------------------------------------------------- #
CTYPES = {".html": "text/html; charset=utf-8", ".css": "text/css",
          ".js": "application/javascript", ".json": "application/json",
          ".png": "image/png", ".jpg": "image/jpeg", ".jpeg": "image/jpeg",
          ".svg": "image/svg+xml", ".webp": "image/webp", ".ico": "image/x-icon"}


def make_handler(store, box_engine):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, code, body=b"", ctype="text/plain"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            # booth kiosks must never show a stale UI after an on-site update
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.end_headers()
            if body:
                self.wfile.write(body)

        def _static(self, rel):
            rel = rel.lstrip("/")
            base = os.path.abspath(WEB_DIR)
            path = os.path.abspath(os.path.join(base, rel))
            if not path.startswith(base) or not os.path.isfile(path):
                return self._send(404)
            ext = os.path.splitext(path)[1]
            with open(path, "rb") as f:
                self._send(200, f.read(), CTYPES.get(ext, "application/octet-stream"))

        def _index(self):
            base = os.path.abspath(WEB_DIR)
            path = os.path.join(base, "index.html")
            if not os.path.isfile(path):
                return self._send(404)
            with open(path, "r", encoding="utf-8") as f:
                html = f.read()
            inject = ""
            if REALSENSE_ONLY:   # RealSense-only station: drop the NVIDIA-demo tabs.
                # AI Detect (NanoOWL) and Product AI (voice assistant) live on the AGX.
                html = re.sub(r'\s*<button class="tab ai" data-mode="detect">.*?</button>',
                              "", html, flags=re.S)
                html = re.sub(r'\s*<button class="tab" data-mode="assistant">.*?</button>',
                              "", html, flags=re.S)
                inject += "window.REALSENSE_ONLY=true;"
            if ASSISTANT_URL:    # point the Product AI tab at the AGX assistant
                inject += "window.ASSISTANT_URL=%s;" % json.dumps(ASSISTANT_URL)
            if inject:
                html = html.replace("</head>", "<script>%s</script></head>" % inject)
            return self._send(200, html.encode(), CTYPES[".html"])

        def do_GET(self):
            p = urlparse(self.path).path
            if p == "/":
                return self._index()
            if p == "/status":
                return self._send(200, json.dumps(SH.status).encode(), CTYPES[".json"])
            if p == "/sysstats":
                return self._send(200, json.dumps(read_sysstats()).encode(), CTYPES[".json"])
            if p == "/whoami":
                # lets the reception (Orin Nano) confirm it's talking to the booth and
                # discover the current IP/mDNS name instead of trusting a hardcoded one.
                host = socket.gethostname()
                info = {"service": "realsense-booth", "hostname": host,
                        "mdns": host + ".local", "ip": _my_ip(), "port": PORT,
                        "enrolled": store.count_people()}
                return self._send(200, json.dumps(info).encode(), CTYPES[".json"])
            if p == "/leads.csv":
                import csv, io
                buf = io.StringIO()
                w = csv.writer(buf)
                w.writerow(["name", "company", "email", "phone", "first_seen", "last_seen", "source"])
                for name, comp, mail, ph, en, sn, src in store.leads():
                    w.writerow([name, comp, mail, ph,
                                time.strftime("%Y-%m-%d %H:%M", time.localtime(en)),
                                time.strftime("%Y-%m-%d %H:%M", time.localtime(sn)), src])
                self.send_response(200)
                self.send_header("Content-Type", "text/csv")
                self.send_header("Content-Disposition", "attachment; filename=booth_leads.csv")
                body = buf.getvalue().encode()
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                self.end_headers()
                return self.wfile.write(body)
            if p == "/pointcloud":
                return self._send(200, SH.cloud_blob, "application/octet-stream")
            if p == "/stream":
                self.send_response(200)
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                seq = 0
                try:
                    while True:
                        jpeg, seq = SH.wait(seq)
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                        self.wfile.write(jpeg); self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    return
            return self._static(p)

        def do_POST(self):
            p = urlparse(self.path).path
            n = int(self.headers.get("Content-Length", 0) or 0)
            body = self.rfile.read(n) if n else b""
            params = parse_qs(body.decode("utf-8", "ignore"))
            if p == "/mode":
                m = (params.get("mode", [""])[0] or "").strip()
                if m:
                    SH.mode = m
                return self._send(200, b"ok")
            if p == "/query":
                q = (params.get("text", [""])[0] or "").strip()
                SH.detect_labels = [s.strip() for s in q.replace(".", ",").split(",") if s.strip()]
                SH.detect_dirty = True
                return self._send(200, b"ok")
            if p == "/enroll":
                name = (params.get("name", [""])[0] or "").strip()[:40]
                company = (params.get("company", [""])[0] or "").strip()[:60]
                email = (params.get("email", [""])[0] or "").strip()[:80]
                phone = (params.get("phone", [""])[0] or "").strip()[:30]
                with SH.cond:
                    embs = SH.pending_embs; SH.pending_embs = []
                    SH.pending_poses = []; SH.pending_since = 0.0
                if name and embs:
                    for e in embs:
                        store.add(name, e, company, email, phone)
                    store.add_lead(name, company, email, phone, "booth")   # persistent lead
                    print(f"[station] enrolled '{name}' with {len(embs)} view(s) "
                          f"({store.count_people()} people)", flush=True)
                    return self._send(200, b"ok")
                return self._send(400, b"no pending/name")
            if p == "/enroll_remote":
                # Reception (Orin Nano) posts JSON: SFace embeddings + visitor details.
                # Only math (128-d vectors) crosses the network, never photos.
                try:
                    payload = json.loads(body.decode("utf-8", "ignore") or "{}")
                except Exception:
                    return self._send(400, b"bad json")
                if ENROLL_TOKEN and payload.get("token") != ENROLL_TOKEN:
                    return self._send(403, b"bad token")
                name = (payload.get("name") or "").strip()[:40]
                company = (payload.get("company") or "").strip()[:60]
                email = (payload.get("email") or "").strip()[:80]
                phone = (payload.get("phone") or "").strip()[:30]
                stored = 0
                for e in (payload.get("embeddings") or []):
                    try:
                        arr = np.frombuffer(base64.b64decode(e), dtype=np.float32)
                        if arr.size >= 64:                      # SFace is 128-d
                            store.add(name, arr.copy(), company, email, phone); stored += 1
                    except Exception:
                        pass
                if name and stored:
                    store.add_lead(name, company, email, phone, "reception")   # persistent lead
                    print(f"[station] reception enrolled '{name}' with {stored} view(s) "
                          f"({store.count_people()} people)", flush=True)
                    return self._send(200, json.dumps({"ok": True, "stored": stored}).encode(),
                                      CTYPES[".json"])
                return self._send(400, json.dumps({"ok": False, "error": "no name/embeddings"}).encode(),
                                  CTYPES[".json"])
            if p == "/recognize_remote":
                # Reception (Orin Nano) asks: "does this embedding match anyone already
                # enrolled?" so a returning visitor is greeted at reception itself and
                # skips capturing 6 fresh angles + the check-in form for nothing.
                try:
                    payload = json.loads(body.decode("utf-8", "ignore") or "{}")
                except Exception:
                    return self._send(400, b"bad json")
                if ENROLL_TOKEN and payload.get("token") != ENROLL_TOKEN:
                    return self._send(403, b"bad token")
                name, score, rid = None, COSINE_MATCH, None
                try:
                    arr = np.frombuffer(base64.b64decode(payload.get("embedding") or ""), dtype=np.float32)
                except Exception:
                    arr = None
                if arr is not None and arr.size >= 64:
                    for kid, kname, kemb in store.all():
                        s = cosine(arr, kemb)
                        if s >= score:
                            score, name, rid = s, kname, kid
                if rid is not None:
                    store.touch(rid)
                return self._send(200, json.dumps({"ok": True, "match": name,
                                                    "score": round(score, 3) if name else None}).encode(),
                                  CTYPES[".json"])
            if p == "/skip":
                with SH.cond:
                    SH.pending_embs = []; SH.pending_poses = []; SH.pending_since = 0.0
                return self._send(200, b"ok")
            if p == "/forget":
                if SH.last_face_id is not None:
                    store.forget(SH.last_face_id)
                return self._send(200, b"ok")
            if p == "/wipe":
                store.wipe()
                return self._send(200, b"ok")
            if p == "/boxreset":
                box_engine.reset()
                return self._send(200, b"ok")
            if p == "/cloudreset":
                cloud_reset()
                return self._send(200, b"ok")
            if p == "/analyticsreset":
                analytics_reset()
                return self._send(200, b"ok")
            return self._send(404)

    return H


def main():
    store = FaceStore(DATA_DIR, RETENTION_H)
    face_engine = FaceEngine()
    safety_engine = SafetyEngine()
    inspect_engine = InspectEngine()
    box_engine = BoxScanEngine()
    if REALSENSE_ONLY:
        owl = None                                           # AI Detect / NanoOWL lives on the AGX
        print("[station] RealSense-only mode: AI Detect / NanoOWL disabled", flush=True)
    else:
        owl = OwlEngine()
        inspect_engine.owl = owl                             # lets Inspect name + veto the person
        box_engine.owl = owl                                 # lets Box Vol veto the person too
        threading.Thread(target=owl.load, daemon=True).start()   # load AI model in bg
    threading.Thread(target=capture_loop,
                     args=(store, face_engine, safety_engine, inspect_engine, box_engine, owl),
                     daemon=True).start()
    for _ in range(100):
        if SH.seq > 0:
            break
        time.sleep(0.1)
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), make_handler(store, box_engine))
    print(f"[station] open  http://<this-host>:{PORT}/", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
