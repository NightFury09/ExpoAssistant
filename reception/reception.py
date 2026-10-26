#!/usr/bin/env python3
"""
Reception station — Orin Nano + USB webcam.
================================================================
Greets visitors, captures their details (name / company / email / phone) plus
multi-angle SFace face embeddings, and POSTs them to the RealSense demo booth so
the booth automatically greets the same person by name when they walk up.

Why it works: this uses the SAME YuNet + SFace models as the booth, so the 128-d
face embeddings are directly comparable — a face enrolled here matches there.

Privacy: only face EMBEDDINGS (vectors) + text details cross the network, NEVER
photos. Runs offline on the booth LAN.

Setup on the Nano:
    pip3 install opencv-python numpy          # (or opencv-contrib-python)
    # put yunet.onnx + sface.onnx in ./models/ (copy from the booth's models/)
    RS_BOOTH_URL=http://192.168.3.224:8000 RS_ENROLL_TOKEN=rover-reception \
        python3 reception.py
    # then open http://<nano-ip>:8001/ on the reception touchscreen
"""
import base64
import json
import os
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import cv2
import numpy as np


def _s(n, d):
    return os.environ.get(n, d)


def _f(n, d):
    try:
        return float(os.environ.get(n, d))
    except (TypeError, ValueError):
        return float(d)


WEBCAM        = _s("RS_WEBCAM", "0").strip()      # preferred /dev/videoN index, or "auto"
V4L_SYS       = "/sys/class/video4linux"
REALSENSE_VID = "8086"                             # Intel RealSense: the rover's camera
WIDTH         = int(_f("RS_WIDTH", 1280))
HEIGHT        = int(_f("RS_HEIGHT", 720))
PORT          = int(_f("RS_PORT", 8001))
MODEL_DIR     = _s("RS_MODEL_DIR", "models")
BOOTH_URL     = _s("RS_BOOTH_URL", "http://booth.local:8000").rstrip("/")  # mDNS name > hardcoded IP
ENROLL_TOKEN  = _s("RS_ENROLL_TOKEN", "rover-reception")  # MUST match the booth
JPEG_QUALITY  = int(_f("RS_JPEG_QUALITY", 80))
MIN_FACE_PX   = int(_f("RS_MIN_FACE_PX", 90))
ENROLL_VIEWS  = int(_f("RS_ENROLL_VIEWS", 6))     # multi-angle embeddings per person
POSE_MIN_DELTA = _f("RS_POSE_MIN_DELTA", 0.06)    # head-pose change to bank a new view
ENROLL_STABLE = int(_f("RS_ENROLL_STABLE", 4))    # stable frames before banking starts
SAME_FACE_MIN = _f("RS_SAME_FACE_MIN", 0.28)      # cosine < this => the face in front doesn't
                                                  # match the scan in progress (a different person)
SWITCH_FRAMES = int(_f("RS_SWITCH_FRAMES", 6))    # a new front face must persist this many frames
                                                  # before the scan switches to them (ignores flips)

GREEN = (80, 200, 80); CYAN = (230, 210, 40); WHITE = (245, 245, 245); DARK = (25, 25, 25)


class Shared:
    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg = None
        self.seq = 0
        self.pending_embs = []       # multi-angle embeddings awaiting a name
        self.pending_poses = []
        self.recognized_name = None  # set once the booth confirms this visitor already exists
        self.recognized_at = 0.0
        self.status = {"faces": 0, "captured": 0, "views": ENROLL_VIEWS, "cam_ok": False,
                       "recognized": None}

    def publish(self, j):
        with self.cond:
            self.jpeg = j; self.seq += 1; self.cond.notify_all()

    def wait(self, last, timeout=5.0):
        with self.cond:
            if self.seq == last:
                self.cond.wait(timeout)
            return self.jpeg, self.seq


SH = Shared()


def label(img, text, org, scale=0.6, color=WHITE, thick=2, bg=DARK, pad=6):
    (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
    x, y = org
    cv2.rectangle(img, (x - pad, y - th - pad), (x + tw + pad, y + base + pad), bg, -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def face_pose(f):
    """Cheap head-pose signature from YuNet landmarks (nose offset / eye distance)."""
    rex, rey, lex, ley, nx, ny = f[4], f[5], f[6], f[7], f[8], f[9]
    ecx, ecy = (rex + lex) / 2.0, (rey + ley) / 2.0
    ed = max(1.0, float(((lex - rex) ** 2 + (ley - rey) ** 2) ** 0.5))
    return ((nx - ecx) / ed, (ny - ecy) / ed)


def pose_dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


class FaceEngine:
    """YuNet detect + SFace embed (same models/params as the booth). Banks up to
    ENROLL_VIEWS embeddings of the primary face as the visitor naturally moves."""
    def __init__(self):
        self.det = cv2.FaceDetectorYN_create(
            os.path.join(MODEL_DIR, "yunet.onnx"), "", (WIDTH, HEIGHT), 0.8, 0.3, 50)
        self.rec = cv2.FaceRecognizerSF_create(os.path.join(MODEL_DIR, "sface.onnx"), "")
        self.stable = 0
        self.last = []
        self.mismatch = 0          # consecutive frames the front face didn't match the scan
        self.recog_checked = False # already asked the booth about this scan session?
        self.recog_inflight = False
        self.empty_frames = 0      # consecutive frames with nobody in view

    def process(self, img):
        self.det.setInputSize((img.shape[1], img.shape[0]))
        _, faces = self.det.detect(img)
        # FRONT-PERSON PRIORITY: the largest (closest) face is always the one being
        # scanned, no matter how many other faces are in frame. A crowded desk never
        # blocks check-in for the person actually standing in front.
        prim_i, prim_a, parsed = -1, 0, []
        if faces is not None:
            for i, f in enumerate(faces):
                w, h = f[2], f[3]
                if w < MIN_FACE_PX:
                    continue
                if w * h > prim_a:
                    prim_a, prim_i = w * h, i
                parsed.append((i, f))

        if prim_i == -1:
            self.stable = 0
            self.empty_frames += 1
            # Nobody's been at the desk for a beat -> clear the session so the next
            # visitor starts fresh instead of inheriting a stale scan/recognition.
            if self.empty_frames == 10:
                with SH.cond:
                    SH.pending_embs = []; SH.pending_poses = []
                    SH.recognized_name = None
                self.recog_checked = False; self.mismatch = 0
        else:
            self.empty_frames = 0
            for i, f in parsed:
                if i != prim_i or SH.recognized_name:
                    continue   # only ever scan the front face; nothing left to do once known
                feat = self.rec.feature(self.rec.alignCrop(img, f)).flatten()
                pose = face_pose(f)
                self.stable += 1
                if self.stable < ENROLL_STABLE:
                    continue
                with SH.cond:
                    if SH.pending_embs:
                        ref = np.mean(SH.pending_embs, axis=0)
                        denom = float(np.linalg.norm(feat) * np.linalg.norm(ref)) + 1e-9
                        sim = float(np.dot(feat, ref)) / denom
                    else:
                        sim = 1.0
                    # --- NEVER-MIX GUARD -----------------------------------------
                    # If the front face stops matching the scan in progress, a
                    # DIFFERENT person is now closest to the camera. Require this to
                    # persist SWITCH_FRAMES in a row (ignores a single noisy/angled
                    # frame from the SAME person) before we restart the scan for the
                    # new person -- this is what stops two people's embeddings ever
                    # being averaged into one record.
                    if SH.pending_embs and sim < SAME_FACE_MIN:
                        self.mismatch += 1
                        if self.mismatch >= SWITCH_FRAMES:
                            SH.pending_embs = [feat.copy()]; SH.pending_poses = [pose]
                            self.recog_checked = False; self.mismatch = 0
                    else:
                        self.mismatch = 0
                        if len(SH.pending_embs) < ENROLL_VIEWS and (
                                not SH.pending_poses or
                                min(pose_dist(pose, q) for q in SH.pending_poses) >= POSE_MIN_DELTA):
                            SH.pending_embs.append(feat.copy())
                            SH.pending_poses.append(pose)
                # RECOGNIZE-AND-SKIP: once a couple of stable views are banked, ask
                # the booth once whether this face already exists there. If so, greet
                # them and skip capturing more angles / the check-in form entirely.
                if (not self.recog_checked and not self.recog_inflight
                        and len(SH.pending_embs) >= 2):
                    self.recog_checked = True
                    self.recog_inflight = True
                    mean_feat = np.mean(SH.pending_embs, axis=0).astype(np.float32).copy()
                    threading.Thread(target=self._recognize_bg, args=(mean_feat,),
                                      daemon=True).start()

        self.last = [(int(f[0]), int(f[1]), int(f[2]), int(f[3])) for _, f in parsed]
        SH.status = {"faces": len(parsed), "captured": len(SH.pending_embs),
                     "views": ENROLL_VIEWS, "cam_ok": True,
                     "recognized": SH.recognized_name}

    def _recognize_bg(self, embedding):
        """Ask the booth (background thread, so the camera loop never blocks on the
        network) whether this embedding matches an already-enrolled visitor."""
        try:
            res = recognize_at_booth(embedding)
            name = res.get("match")
            if name:
                with SH.cond:
                    SH.recognized_name = name
                    SH.recognized_at = time.time()
                    SH.pending_embs = []; SH.pending_poses = []  # already known -> nothing to enrol
                print(f"[reception] recognized returning visitor '{name}' "
                      f"(score {res.get('score')}) -> enrollment skipped", flush=True)
        except Exception as e:
            print(f"[reception] recognize check failed: {e}", flush=True)
        finally:
            self.recog_inflight = False

    def draw(self, img):
        for (x, y, w, h) in self.last:
            cv2.rectangle(img, (x, y), (x + w, y + h), GREEN, 2)
        label(img, "RECEPTION - face check-in", (14, 34), 0.6, DARK, 2, CYAN)
        if SH.recognized_name:
            label(img, f"Welcome back, {SH.recognized_name}! Already checked in.",
                  (14, 70), 0.55, DARK, 2, GREEN)
        elif self.last:
            n = len(SH.pending_embs)
            label(img, f"capturing {n}/{ENROLL_VIEWS} angles - look around gently",
                  (14, 70), 0.55, DARK, 2, GREEN)
        else:
            label(img, "step in front of the camera", (14, 70), 0.55, WHITE, 2, DARK)


def show_msg(title, sub, enc):
    img = np.zeros((HEIGHT, WIDTH, 3), np.uint8); img[:] = (14, 11, 10)
    (tw, _), _ = cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
    cv2.putText(img, title, ((WIDTH - tw) // 2, HEIGHT // 2), cv2.FONT_HERSHEY_SIMPLEX,
                0.9, WHITE, 2, cv2.LINE_AA)
    if sub:
        (sw, _), _ = cv2.getTextSize(sub, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.putText(img, sub, ((WIDTH - sw) // 2, HEIGHT // 2 + 34), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (160, 144, 138), 1, cv2.LINE_AA)
    ok, jpeg = cv2.imencode(".jpg", img, enc)
    if ok:
        SH.publish(jpeg.tobytes())


def _sysfs(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return ""


def find_webcam(sysfs=V4L_SYS):
    """The /dev/videoN to open, or None when no usable webcam is plugged in.

    Chosen by what the device IS, not by its number. Numbers follow plug-in
    order, and the rover's RealSense D455 alone creates six nodes, so with it
    plugged in first "video0" was the D455 -- reception then grabbed the rover's
    camera. Usable means: a USB device, not an Intel RealSense, and the UVC
    capture node (index 0) rather than its metadata node (index 1). RS_WEBCAM is
    preferred when it names a usable node; "auto" (or anything unusable) takes
    the lowest-numbered usable one.
    """
    try:
        nodes = sorted(int(d[5:]) for d in os.listdir(sysfs)
                       if d.startswith("video") and d[5:].isdigit())
    except OSError:
        return None
    usable = []
    for n in nodes:
        base = os.path.join(sysfs, f"video{n}")
        # device -> the USB interface; the USB device above it has idVendor.
        vid = _sysfs(os.path.join(os.path.realpath(os.path.join(base, "device")),
                                  "..", "idVendor"))
        if vid and vid != REALSENSE_VID and _sysfs(os.path.join(base, "index")) in ("", "0"):
            usable.append(n)
    if WEBCAM.isdigit() and int(WEBCAM) in usable:
        return int(WEBCAM)
    return usable[0] if usable else None


def capture_loop(fe):
    """Webcam watchdog: (re)open the camera and stream; survives a USB drop."""
    enc = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
    cap = None; fi = 0; miss = 0; waiting = False
    while True:
        if cap is None or not cap.isOpened():
            show_msg("Connecting to camera...", "USB webcam", enc)
            dev = find_webcam()
            if dev is None:
                # Nothing to open, so do not ask OpenCV: each failed open logged
                # four V4L2/GStreamer warnings, every 1.5 s -- ~100k syslog lines
                # a day while the webcam was unplugged. Say it once instead.
                if not waiting:
                    print("[reception] waiting for a USB webcam "
                          "(RealSense cameras are left to the rover)", flush=True)
                    waiting = True
                SH.status = {"cam_ok": False, "faces": 0, "captured": len(SH.pending_embs),
                             "views": ENROLL_VIEWS}
                time.sleep(1.5); continue
            # V4L2 directly: it is what opened the webcam before, and without it
            # OpenCV falls back to a GStreamer pipeline on every failure.
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
            if not cap.isOpened():
                SH.status = {"cam_ok": False, "faces": 0, "captured": len(SH.pending_embs),
                             "views": ENROLL_VIEWS}
                time.sleep(1.5); continue
            name = _sysfs(os.path.join(V4L_SYS, f"video{dev}", "name")) or "webcam"
            print(f"[reception] webcam: /dev/video{dev} ({name})", flush=True)
            waiting = False
        ok, img = cap.read()
        if not ok:
            miss += 1
            if miss >= 5:
                try: cap.release()
                except Exception: pass
                cap = None; miss = 0
                show_msg("Reconnecting camera...", "check the USB cable", enc)
            time.sleep(0.05); continue
        miss = 0
        try:
            fi += 1
            if fi % 2 == 0:
                fe.process(img)
            fe.draw(img)
            ok, jpeg = cv2.imencode(".jpg", img, enc)
            if ok:
                SH.publish(jpeg.tobytes())
        except Exception as e:
            print(f"[reception] frame error: {e}", flush=True)


def post_to_booth(name, company, email, phone, embs):
    payload = json.dumps({
        "token": ENROLL_TOKEN, "name": name, "company": company, "email": email, "phone": phone,
        "embeddings": [base64.b64encode(e.astype(np.float32).tobytes()).decode() for e in embs],
    }).encode()
    req = urllib.request.Request(BOOTH_URL + "/enroll_remote", data=payload,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=6) as r:
        return json.loads(r.read().decode())


def recognize_at_booth(embedding):
    """Ask the booth if this (mean) embedding matches an already-enrolled visitor.
    Returns {"ok": True, "match": name or None, "score": float or None}."""
    payload = json.dumps({
        "token": ENROLL_TOKEN,
        "embedding": base64.b64encode(embedding.astype(np.float32).tobytes()).decode(),
    }).encode()
    req = urllib.request.Request(BOOTH_URL + "/recognize_remote", data=payload,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read().decode())


PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<title>Reception - Check in</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#0a0b0e;--panel:#14161c;--line:#262a33;--txt:#e9ecf2;--dim:#8a90a0;--nv:#76b900;--accent:#39c2ff}
*{box-sizing:border-box}html,body{margin:0;height:100%;background:var(--bg);color:var(--txt);
 font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
.wrap{min-height:100%;display:flex;flex-direction:column;align-items:center;gap:18px;padding:22px}
h1{margin:6px 0 0;font-size:26px;font-weight:800}h1 b{color:var(--nv)}
.sub{color:var(--dim);font-size:14px;margin:0}
.stage{position:relative;width:min(92vw,900px);aspect-ratio:16/9;background:#000;border-radius:14px;
 overflow:hidden;box-shadow:0 12px 50px rgba(0,0,0,.5)}
#video{width:100%;height:100%;object-fit:cover}
.cap{position:absolute;left:14px;top:14px;background:rgba(10,11,14,.7);border:1px solid var(--line);
 border-radius:20px;padding:6px 12px;font-size:13px;font-weight:700;color:var(--nv)}
.form{width:min(92vw,900px);display:grid;grid-template-columns:1fr 1fr;gap:12px}
.form input{background:#0d0f14;border:1px solid var(--line);color:#fff;border-radius:10px;
 padding:14px 14px;font-size:17px}
.form input.full{grid-column:1/3}
.btn{grid-column:1/3;display:flex;gap:12px}
.btn button{flex:1;border:0;border-radius:12px;padding:16px;font-size:17px;font-weight:800;cursor:pointer}
.go{background:var(--nv);color:#08210b}.go:disabled{opacity:.5;cursor:not-allowed}
.clr{background:transparent;color:var(--dim);border:1px solid var(--line)!important;flex:0 0 auto;padding:16px 20px}
.note{min-height:26px;font-size:15px;color:var(--dim)}
.ok{color:var(--nv);font-weight:700}.err{color:#ff6b6b;font-weight:700}
.big{position:fixed;inset:0;display:none;align-items:center;justify-content:center;flex-direction:column;
 gap:10px;background:rgba(5,7,10,.94);z-index:9;text-align:center;padding:24px}
.big.show{display:flex}.big h2{font-size:clamp(30px,6vw,64px);margin:0;font-weight:800}
.big p{font-size:clamp(16px,2.4vw,24px);color:var(--dim);margin:0}
.big .tick{font-size:64px}
</style></head>
<body><div class="wrap">
  <h1>Welcome to <b>RP tech</b> - please check in</h1>
  <p class="sub">We'll greet you by name at the demo booth. A local, encrypted face signature (no photo) is stored for up to 3 days.</p>
  <div class="stage"><img id="video" src="/stream" alt="camera"><div class="cap" id="cap">step in front of the camera</div></div>
  <div class="form">
    <input id="name" class="full" placeholder="Your name" autocomplete="off">
    <input id="company" placeholder="Company" autocomplete="off">
    <input id="email" placeholder="Email (required)" autocomplete="off" inputmode="email">
    <input id="phone" class="full" placeholder="Phone number" autocomplete="off" inputmode="tel">
    <div class="btn">
      <button class="go" id="go" disabled onclick="checkin()">Check me in</button>
      <button class="clr" onclick="resetCap()">Start over</button>
    </div>
    <div class="note full" id="note"></div>
  </div>
</div>
<div class="big" id="big"><div class="tick">✅</div><h2 id="bigName">Welcome!</h2><p id="bigMsg">Head to the demo booth - it will greet you.</p></div>
<script>
const cap=document.getElementById('cap'), go=document.getElementById('go'), note=document.getElementById('note');
let captured=0, views=6, lastRecognized=null;
function say(t){try{const u=new SpeechSynthesisUtterance(t);u.rate=1.0;speechSynthesis.speak(u);}catch(e){}}
async function poll(){
  try{const s=await (await fetch('/status')).json();
    captured=s.captured||0; views=s.views||6;
    if(s.cam_ok===false){cap.textContent='camera offline';}
    else if(s.recognized){cap.textContent='Welcome back, '+s.recognized+'!';}
    else if(s.faces){cap.textContent='capturing '+captured+'/'+views+' angles - look around gently';}
    else{cap.textContent='step in front of the camera';}
    go.disabled = captured<1 || !!s.recognized;
    // A returning visitor is recognised automatically -- greet them and skip the
    // enrolment form entirely. Only fire once per recognition (not every poll tick).
    if(s.recognized && s.recognized!==lastRecognized){
      lastRecognized=s.recognized;
      document.getElementById('bigName').textContent='Welcome back, '+s.recognized+'!';
      document.getElementById('bigMsg').textContent="You're already checked in - head to the demo booth.";
      document.getElementById('big').classList.add('show');
      say('Welcome back, '+s.recognized+'! You are already checked in.');
      setTimeout(()=>document.getElementById('big').classList.remove('show'),4500);
    }else if(!s.recognized){
      lastRecognized=null;
    }
  }catch(e){}
}
setInterval(poll,400); poll();
async function checkin(){
  const name=document.getElementById('name').value.trim();
  const email=document.getElementById('email').value.trim();
  if(!name){note.innerHTML='<span class="err">Please enter your name.</span>';return;}
  if(!email){note.innerHTML='<span class="err">Please enter your email.</span>';return;}
  if(!/^[^@\s]+@[^@\s]+\.[^@\s]+$/.test(email)){note.innerHTML='<span class="err">Please enter a valid email address.</span>';return;}
  if(captured<1){note.innerHTML='<span class="err">Look at the camera for a moment first.</span>';return;}
  go.disabled=true; note.textContent='Saving...';
  const body=new URLSearchParams({name, email,
    company:document.getElementById('company').value.trim(),
    phone:document.getElementById('phone').value.trim()});
  try{
    const r=await (await fetch('/enroll',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body})).json();
    if(r.ok){
      document.getElementById('bigName').textContent='Welcome, '+name+'!';
      document.getElementById('bigMsg').textContent='Head to the demo booth - it will greet you.';
      document.getElementById('big').classList.add('show');
      say('Welcome '+name+'! Head to the demo booth, it will recognise you.');
      setTimeout(()=>{document.getElementById('big').classList.remove('show');
        ['name','company','email','phone'].forEach(id=>document.getElementById(id).value='');
        note.textContent='';},4500);
    }else{note.innerHTML='<span class="err">'+(r.error||'Could not check in')+'</span>';go.disabled=false;}
  }catch(e){note.innerHTML='<span class="err">Booth unreachable - check the network.</span>';go.disabled=false;}
}
async function resetCap(){await fetch('/reset',{method:'POST'});
  ['name','company','email','phone'].forEach(id=>document.getElementById(id).value='');note.textContent='';}
</script></body></html>"""


def make_handler():
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, code, body=b"", ctype="text/plain"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.end_headers()
            if body:
                self.wfile.write(body)

        def do_GET(self):
            p = urlparse(self.path).path
            if p == "/":
                return self._send(200, PAGE.encode(), "text/html; charset=utf-8")
            if p == "/status":
                return self._send(200, json.dumps(SH.status).encode(), "application/json")
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
            return self._send(404)

        def do_POST(self):
            p = urlparse(self.path).path
            n = int(self.headers.get("Content-Length", 0) or 0)
            body = self.rfile.read(n) if n else b""
            params = parse_qs(body.decode("utf-8", "ignore"))
            if p == "/enroll":
                name = (params.get("name", [""])[0] or "").strip()[:40]
                company = (params.get("company", [""])[0] or "").strip()[:60]
                email = (params.get("email", [""])[0] or "").strip()[:80]
                phone = (params.get("phone", [""])[0] or "").strip()[:30]
                if not name:
                    return self._send(400, json.dumps({"ok": False, "error": "name required"}).encode(),
                                      "application/json")
                if not email or "@" not in email or "." not in email.rsplit("@", 1)[-1]:
                    return self._send(400, json.dumps({"ok": False, "error": "a valid email is required"}).encode(),
                                      "application/json")
                with SH.cond:
                    embs = SH.pending_embs; SH.pending_embs = []; SH.pending_poses = []
                if not embs:
                    return self._send(400, json.dumps({"ok": False, "error": "no face captured yet"}).encode(),
                                      "application/json")
                try:
                    res = post_to_booth(name, company, email, phone, embs)
                    ok = bool(res.get("ok"))
                    print(f"[reception] checked in '{name}' -> booth stored {res.get('stored')}", flush=True)
                    return self._send(200 if ok else 502,
                                      json.dumps({"ok": ok, "name": name,
                                                  "stored": res.get("stored", len(embs))}).encode(),
                                      "application/json")
                except Exception as e:
                    print(f"[reception] booth POST failed: {e}", flush=True)
                    return self._send(502, json.dumps({"ok": False, "error": f"booth unreachable: {e}"}).encode(),
                                      "application/json")
            if p == "/reset":
                with SH.cond:
                    SH.pending_embs = []; SH.pending_poses = []
                    SH.recognized_name = None
                return self._send(200, b"ok")
            return self._send(404)

    return H


def check_booth():
    """Confirm the booth is reachable at startup via its /whoami and log its identity
    (hostname/IP/enrolled count) so a wrong RS_BOOTH_URL is obvious immediately."""
    try:
        with urllib.request.urlopen(BOOTH_URL + "/whoami", timeout=4) as r:
            info = json.loads(r.read().decode())
        print(f"[reception] booth OK: {info.get('hostname')} @ {info.get('ip')} "
              f"({info.get('enrolled')} enrolled) via {BOOTH_URL}", flush=True)
    except Exception as e:
        print(f"[reception] WARNING: booth at {BOOTH_URL} not reachable yet ({e}). "
              f"Check it's on + same LAN; set RS_BOOTH_URL to its name/IP if wrong.", flush=True)


def main():
    for m in ("yunet.onnx", "sface.onnx"):
        if not os.path.isfile(os.path.join(MODEL_DIR, m)):
            print(f"[reception] MISSING {MODEL_DIR}/{m} - copy it from the booth's models/ dir", flush=True)
    check_booth()
    fe = FaceEngine()
    threading.Thread(target=capture_loop, args=(fe,), daemon=True).start()
    for _ in range(100):
        if SH.seq > 0:
            break
        time.sleep(0.1)
    print(f"[reception] open http://<this-nano-ip>:{PORT}/   ->   booth {BOOTH_URL}", flush=True)
    try:
        ThreadingHTTPServer(("0.0.0.0", PORT), make_handler()).serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
