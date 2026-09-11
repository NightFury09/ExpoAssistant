# Rover Console

One web page at **http://192.168.3.224:8080** that replaces the terminal for
everything you do with the rover day to day: drive it, map a room, load a map,
mark the demo stations, and send it to one of them.

It runs **outside** every ROS stack and stays up while stacks come and go. That
is deliberate — it is what lets it start and stop them, and it is what makes
two stacks at once impossible.

---

## The two tabs

**DRIVE** — the camera, WASD driving, speed control, obstacle radar, velocity
trace, drivetrain and link health. Unchanged from before.

**MAP** — the occupancy grid with the rover on it, live scan, the current Nav2
plan, and the saved demo points. Press **M** to flip between tabs.

---

## Normal expo day

1. Power up. The console is already running (systemd starts it at boot).
2. Open the page, go to **MAP**.
3. Pick the map in the dropdown, press **NAVIGATE**. Wait for the mode chip to
   read `NAVIGATION · 4/4 nodes`.
4. Arm **SET POSE**, then drag on the map from where the rover actually is,
   in the direction it is actually facing. The scan points should snap onto the
   walls. If they do not, do it again — everything downstream depends on this.
5. Press **GO** next to a demo point, or arm **SET GOAL** and drag anywhere.
6. **CANCEL NAV** stops the current goal. **EMERGENCY STOP** kills all motion.

## Mapping a new room

1. Mode → **MAP ROOM**. This starts SLAM, the lidar, odometry and Foxglove.
2. Go to the **DRIVE** tab and drive the perimeter slowly with WASD. Watch the
   map fill in on the MAP tab as you go.
3. Back on MAP, type a name and press **SAVE MAP**.
4. Press **USE THIS MAP & NAVIGATE**.

Step 4 is one action: it stops SLAM and brings navigation up on the map you
just saved, about 20 s end to end. The rover stays powered the whole time.
**Do not press STOP to leave mapping** — STOP means "shut everything down",
and you would be left with the lidar and ESP32 dark.

The same applies in reverse: **MAP ROOM** while navigating switches straight
into mapping. Only STOP actually powers things down.

Saving also writes a `.posegraph`, so the map can be extended in a later
session instead of re-surveyed from scratch.

## The camera

**CAMERA** on the MAP tab turns the RealSense on and off, and it is deliberately
**not** part of either stack — it runs on its own, so switching between mapping
and navigation does not make the video drop out and the USB device
re-enumerate. Turn it on once and leave it on.

It also survives a console restart: on startup the console re-adopts a camera
(or a stack) it left running, rather than orphaning it.

If the button reads **CAMERA (EXT)**, something else started a camera node and
the console will not fight it.

## Marking demo points

The flow that actually gets used:

1. In **NAVIGATION** mode with the pose set, drive to the booth with WASD.
2. On the MAP tab, type the name and press **SAVE HERE**.

Or place one without driving there: type the name, press **PLACE**, then drag
on the map — the drag sets both the spot and the direction the rover should
face when it arrives (point it at the visitor, not at the wall).

Name one of them `home` or `base` and it is drawn in green — that is where
"return to base" goes.

Points are stored in `config/demo_waypoints.yaml` together with the name of the
map they were captured on. Load a different map and the console says so in red
rather than sending the rover to coordinates that mean nothing.

---

## What the map shows

| | |
|---|---|
| light grey | surveyed, drivable |
| black | wall |
| mid slate | never seen — the planner will not route through it |
| red dots | live lidar returns |
| blue line | the current Nav2 plan |
| blue circle | the rover, drawn at its true 0.33 m footprint |
| purple pin | a demo point |
| green pin | home / base |

Drag to pan, scroll to zoom, **⤢** fits the map, **◎** keeps the rover centred.

If the rover circle overlaps a wall on screen, it would overlap it in the room.
That is the check to make before trusting a goal.

---

## Mode states

| state | meaning |
|---|---|
| `IDLE` | nothing running, safe to start either mode |
| `STARTING` | launched, waiting for the nodes to appear |
| `NAVIGATION` / `MAPPING` | up and ready |
| `STOPPING` | shutting the stack down, up to ~25 s |
| `EXTERNAL` | **a stack was started from a terminal, not here** |

Switching mode while a stack is up is allowed and is the normal way to move
between mapping and navigation: the console stops one and starts the other as a
single action. Only `STOP` leaves the rover with nothing running.

`EXTERNAL` is not an error, it is the console refusing to fight you. Stop the
stack in its own terminal and the console goes back to `IDLE`.

Starting a *second* stack is refused: two stacks means two micro-ROS agents on
the same serial port and two nodes publishing `/tf`, which shows up as the
rover teleporting rather than as an obvious duplicate. A mode switch is not a
second stack — it stops the first one and waits for it to go.

Each launch writes `logs/<mode>-<timestamp>.log`. When a stack fails to come
up, that file has the reason.

---

## Autostart

```bash
~/AGX_Orin_Backup/rover_project/systemd/install.sh
```

Then:

```bash
sudo systemctl status rover-console
journalctl -u rover-console -f
```

Stopping the service also stops any stack it launched — they live in its
cgroup, so nothing is left holding the lidar or the ESP32 serial port.

Restarting the console on its own (not via systemd) leaves the stack and camera
running and re-adopts them, so a console restart is not a rover restart.

---

## Foxglove

Still there, still launched by both stacks, still the right tool for deep
debugging (TF trees, costmap layers, raw messages). The console covers the
day-to-day; Foxglove covers "why is this not working".

---

## Running it by hand

If the service is not installed, or you want it in a terminal:

```bash
source /opt/ros/humble/setup.bash && source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash && ros2 run rover_core rover_dashboard
```

Do **not** also pass `use_dashboard:=true` to `master_navigation.launch.py`.
The second copy cannot bind :8080 and the page silently stops updating.

---

## Endpoints

The page is a client of a plain HTTP API, so anything else can drive the rover
the same way. This is the hook for Product_RAG_:

| method | path | body | does |
|---|---|---|---|
| POST | `/api/wp/goto` | `{"name":"robotic_arm"}` | send the rover to a demo point |
| POST | `/api/nav_cancel` | `{}` | cancel the current goal |
| GET | `/api/metrics` | | everything: `nav.state`, `nav.remaining`, pose, waypoints, mode |
| POST | `/api/goal` | `{"x":,"y":,"yaw":}` | arbitrary goal, degrees |
| POST | `/api/wp/save` | `{"name":"x","here":true}` | capture the current pose |
| POST | `/api/mode` | `{"mode":"navigation","map":"/path.yaml","switch":true}` | change mode; `switch` stops the running stack first |
| POST | `/api/camera` | `{"on":true}` | RealSense on/off, independent of the stack |
| POST | `/api/estop` | `{"on":true}` | stop everything |

`nav.state` goes `sending → active → arrived` (or `aborted`), which is what an
assistant would poll to know when to start talking.

Goals are refused while the e-stop is engaged, at the API, not just in the UI.
