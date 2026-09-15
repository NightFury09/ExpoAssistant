# Rover — Architecture & Troubleshooting Reference

Companion to [COMMANDS.md](COMMANDS.md) (which has copy-paste commands). This file
is the mental model: how the system fits together, and how to diagnose it yourself
when something breaks.

---

## 1. Hardware inventory

| Device | Connects via | Notes |
|---|---|---|
| Jetson AGX Orin | — | The brain. Runs ROS 2 Humble, headless (SSH from Windows). |
| ESP32 | USB → `/dev/ttyESP32` | Runs micro-ROS firmware. Talks to both motor drivers over two internal UARTs (NOT the same USB link). |
| RMCS-2303 ×2 | ESP32 UART1 (pins 16/17) and UART2 (pins 14/13), Modbus ASCII @ 9600 baud | Closed-loop DC servo drives. LEFT wheel = slave ID **2** (UART2/pins 14,13). RIGHT wheel = slave ID **7** (UART1/pins 16,17). |
| DC motors ×2 | wired to the RMCS-2303 outputs | Each has a quadrature encoder (334 lines/rot × 4 = 1336 counts/rev) feeding its driver's closed loop — NOT read directly by the ESP32 except via Modbus register reads. |
| RPLIDAR A1M8 | USB → `/dev/ttyLIDAR` | 2D lidar, ~360°, publishes `/scan`. |
| RealSense D455 | USB (currently USB-2, should be USB-3) | Optional/time-shared with the booth demo. Depth + pointcloud feed Nav2's costmap. |

**udev identifies devices by their unique USB serial number**, not by port
(`/etc/udev/rules.d/99-rover.rules`). So `/dev/ttyESP32` and `/dev/ttyLIDAR` always
point to the right device no matter which physical USB port you plug into — you
never need to edit config after moving a cable.

### RMCS-2303 wiring (ESP32 ↔ each driver — cross TX/RX, 3.3V direct)

| Wheel | Slave ID | ESP32 TX → driver RXD (pin 2) | ESP32 RX ← driver TXD (pin 3) | GND |
|---|---|---|---|---|
| LEFT | 2 | GPIO 13 | GPIO 14 | common with driver pin 1 |
| RIGHT | 7 | GPIO 17 | GPIO 16 | common with driver pin 1 |

Full register map / control words are in [`rmcs_2303_manual.md`](rmcs_2303_manual.md).
Key ones the firmware uses: `REG_SPEED=14`, `REG_CONTROL=2`,
`CTRL_CW=257 (0x0101)`, `CTRL_CCW=265 (0x0109)`, `CTRL_STOP=0 (0x0000)` —
**0x0100 is NOT a stop command**, it only selects Mode 1.

---

## 2. Software / ROS graph — how a drive command flows

```
teleop / Nav2 controller
        │  /cmd_vel  (geometry_msgs/Twist)
        ▼
micro-ROS agent  (serial bridge, Jetson ⇄ ESP32, 115200 baud)
        ▼
ESP32 firmware (esp32_rover_firmware_v2/src/main.cpp)
   - twist_callback(): Twist → per-wheel target RPM
   - apply_command(): RPM → Modbus ASCII writes to each RMCS-2303
        │
        ├──► RMCS-2303 LEFT  (slave 2) ──► motor + encoder (closed loop)
        └──► RMCS-2303 RIGHT (slave 7) ──► motor + encoder (closed loop)
        │
        ├─► publishes /wheel_ticks   (dead-reckoned from COMMANDED rpm, 20Hz)
        └─► publishes /encoder_ticks (REAL Modbus register reads, 2Hz, has a
                                       health flag — see §5)
```

`/wheel_ticks` → **rover_odometry** node → `/odom` (+ TF, depending on mode, see §3)
`/scan` ← RPLIDAR
Depth/pointcloud ← RealSense (optional, separate launch)

Two ROS "modes" exist as two different launch files — pick the one matching
what you're trying to do:

| Launch file | Purpose | Localizer |
|---|---|---|
| `slam_teleop.launch.py` | **Build a new map** while driving manually | `slam_toolbox` (SLAM) |
| `master_navigation.launch.py` | **Autonomously navigate** a saved map | `AMCL` (Nav2) |

---

## 3. The TF tree — the part that silently breaks things

```
map ──► odom ──► base_footprint ──► base_link ──► laser_frame
                                              ├──► camera_link
                                              ├──► left_wheel
                                              └──► right_wheel
```

**Golden rule: every edge has exactly ONE publisher.** Two nodes publishing the
same edge fight each other and produce jittery/wrong poses.

| Edge | SLAM mode publishes it as... | Nav2 mode publishes it as... |
|---|---|---|
| `map → odom` | `slam_toolbox` | `AMCL` |
| `odom → base_footprint` | `rover_odometry` (its `publish_tf` param = **True**) | the EKF (`robot_localization`); `rover_odometry` runs with `publish_tf` = **False** |
| `base_footprint → ...` | `robot_state_publisher` (from `rover.urdf`), same in both modes | same |

If you ever see two nodes both trying to publish `odom→base_footprint`, that's
a bug — check the `publish_tf` param on `rover_odometry` matches which mode
you're running.

**`map` does not exist until the localizer runs and succeeds.** In Nav2 mode
that means AMCL needs an initial pose (§6) — until then, every costmap log
line about "map frame does not exist" is expected and harmless.

---

## 4. Calibration values (already tuned — don't reset these)

In `rover_core/rover_core/rover_odometry.py`:
- `wheel_radius = 0.0257` — calibrated from a measured 1 m drive (raw model of
  0.05 over-reported ~1.94×).
- `wheel_separation = 0.4621` — calibrated from a measured 720° spin (raw
  model of 0.44 over-reported rotation ~5%).
- `encoder_cpr = 4000.0` — an internal bookkeeping unit for the **dead-reckoned**
  `/wheel_ticks` only. It is NOT the real encoder resolution (that's 1336
  counts/rev on the driver) — don't confuse the two.

If you ever re-flash new motors/wheels, these two values need
re-calibrating (drive exactly 1 m, compare to `/odom`; spin exactly one or
more full turns, compare to `/heading_deg`).

---

## 5. Diagnostic toolkit — commands to actually see what's wrong

```bash
source /opt/ros/humble/setup.bash && source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash
```
then:

| Question | Command |
|---|---|
| What's running? | `ros2 node list` |
| What topics exist? | `ros2 topic list` |
| Is data flowing, how fast? | `ros2 topic hz <topic>` |
| Peek at one message | `ros2 topic echo <topic> --once` |
| **#1 motor-base health check** | `ros2 topic echo /encoder_ticks --field z` |
| Is this specific TF link alive? | `ros2 run tf2_ros tf2_echo <parent_frame> <child_frame>` |
| Are the USB devices present? | `ls -la /dev/ttyESP32 /dev/ttyLIDAR` |
| Both CP210x devices on the bus? | `lsusb \| grep -c 10c4:ea60` (should be 2) |
| Can a serial port actually be opened? | `python3 -c "import serial; s=serial.Serial('/dev/ttyESP32',115200,timeout=1); print('OK'); s.close()"` |

### `/encoder_ticks` field `z` — the single most useful number on this robot
It's a real Modbus register READ from each driver (not a guess), reported
every ~2Hz:
- `z = 0` → both drivers responding. Motor base is healthy.
- `z = 1` → LEFT (slave 2, GPIO 13/14) not answering.
- `z = 2` → RIGHT (slave 7, GPIO 16/17) not answering.
- `z = 3` → **neither** responding — almost always a shared problem: driver
  power off, or the **common GND wire at the ESP32** is loose (this has bitten
  us more than once — check it first).

---

## 6. Setting the initial pose (localizing AMCL under Nav2)

Nav2 can't do anything until AMCL knows where the robot is on the saved map
(`map→odom` must exist). Two ways to do it — pick whichever is easier in the
moment:

### Method A — let AMCL figure it out itself (recommended, no guessing)
1. In Foxglove: Fixed frame = `map`, enable `/map`, `/scan`, and if it exists
   `/particle_cloud` (check with `ros2 topic list | grep -i particle` — the
   exact topic name varies by Nav2 version). The particle cloud is AMCL's
   spread of pose guesses — watching it collapse to one tight cluster is the
   clearest sign of successful localization.
2. Scatter the guesses across the whole map:
   ```bash
   ros2 service call /reinitialize_global_localization std_srvs/srv/Empty {}
   ```
3. Run your normal teleop and **slowly** rotate in place for a couple of full
   turns, then also drive forward/back a meter or so if there's room.
   **Pure rotation alone sometimes isn't enough** to disambiguate (especially
   in a boxy/symmetric room) — a bit of translation through a distinctive part
   of the room (a corner, a doorway) resolves it much faster.
4. Watch Foxglove: the particle cloud tightens, and the red `/scan` points
   start snapping onto the map's black wall cells. That's convergence.
5. If it hasn't converged after ~1 minute, re-run step 2 and drive more —
   through a less symmetric part of the room if you can.

### Method B — tell it directly (precise, but you do the math)
```bash
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'}, pose: {pose: {position: {x: X, y: Y, z: 0.0}, \
orientation: {z: QZ, w: QW}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, \
0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}"
```
- `X, Y` = the robot's real position in the `map` frame (meters). Check the
  map's extent first: `ros2 topic echo /map --field info --once` gives you
  `origin.position` + `width/height*resolution`, so you know the coordinate
  range to aim for.
- `QZ, QW` come from the heading (yaw, radians, 0 = facing the map's +x axis,
  positive = counter-clockwise): `QZ = sin(yaw/2)`, `QW = cos(yaw/2)`.
  Quick table: yaw 0°→(0,1) · 90°→(0.707,0.707) · 180°→(1,0) · -90°→(-0.707,0.707).
- Publish, look at Foxglove: if the scan is rotated off the walls by roughly
  some angle, adjust yaw by that amount; if it's offset but parallel, nudge
  x/y. Republish. Repeat until the scan overlays the walls.

Once localized, send a goal the same way Nav2 always expects it — publish a
`geometry_msgs/PoseStamped` to `/goal_pose` (or use Foxglove's publish tool
configured for that topic/type).

---

## 7. Troubleshooting playbook

| Symptom | Likely cause | Check / fix |
|---|---|---|
| Rover doesn't move at all | ESP32 not connected to agent, or driver wiring | `ros2 node list \| grep esp32_rover`; then `/encoder_ticks` field `z` (§5) |
| `z: 3` on `/encoder_ticks` | Both drivers silent — usually shared GND, or driver power off | Re-seat the ground wire at the **ESP32 end** first |
| `z: 1` or `z: 2` | One driver's wiring only | Check that driver's 2 data pins per the table in §1 |
| Agent won't open the port / `errno: 5` (EIO) | Bad cable, flaky USB port, or unplug/replug happened mid-session | Reseat the cable; try `python3 -c "import serial; ..."` open test (§5); as a last resort reflash `esp32_rover_firmware_v2` — a successful flash proves the link and firmware are both healthy |
| ESP32 connects to USB fine but agent never sees a session | Firmware not actually running / corrupted from earlier flashing | Reflash `esp32_rover_firmware_v2`; the auto-reset-on-launch is unreliable on this board (BOOT/EN isn't wired to DTR/RTS) — **press the physical EN button** on the ESP32 right as the agent starts |
| USB device (ESP32 or lidar) randomly disappears from `lsusb` | Insufficient power — bus-powered hub, or marginal cable | Use direct Jetson ports or a **powered** hub (own wall adapter), especially for the RPLIDAR (spins a motor, power-hungry) |
| **`Error, unexpected error, code: 80008004` in `~/.ros/log/rplidar_node_*.log`, node crash-loops, no `/scan`** | `0x80008004` = `RESULT_OPERATION_NOT_SUPPORT` (NOT a timeout — that would be `0x80008002`). The node asked the lidar for a scan mode it does not have. **This rover has an RPLIDAR A1M8, which supports only `Standard` and `Express`.** `Sensitivity` and `Boost` are A2/A3/S1 modes. | Set `'scan_mode': 'Standard'` in the launch file, rebuild `my_robot_bringup`, relaunch. Confirm the model first with the raw health query below — `model=24` (0x18) is an A1M8. |
| **Lidar LED is on, motor never spins, `/scan` never appears** | Two different faults produce this. **(1)** Bad USB data link — the tell is `[Errno 5] Input/output error` when opening the port (see the port-open test below). Power lines work (LED/motor), data lines do not. **(2)** Unsupported scan mode — the port opens fine and the raw health query answers, but the node still fails with `80008004` (row above). **The LED only proves USB power — it never proves the data path works.** | Run the **port-open test**, then the **raw health query** (below). `Errno 5` → replug the cable, try a different Jetson port, or use a powered hub. Port opens + health GOOD but node still fails → it is the scan mode, not the hardware. |
| **Need to prove whether the lidar hardware is actually alive** | Distinguishes a dead data path from a driver/config problem without touching ROS. | **Port-open test:** `timeout 4 python3 -c "import serial; s=serial.Serial('/dev/ttyLIDAR',115200,timeout=1); print('PORT OK'); s.close()"` — `Errno 5` = bad USB link, stop debugging ROS. **Raw health query:** see `Raw RPLIDAR health check` at the bottom of this file. `health status = GOOD` means the hardware and cable are fine and the fault is in configuration. |
| Rover drives but turns are backwards / veers | Left/right wheel or inversion mapping wrong in firmware | Check `LEFT_MOTOR_SLAVE_ID`/`RIGHT_MOTOR_SLAVE_ID` and the `dir_cmd(..., inverted)` calls in `apply_command()` against §1's table |
| Odometry distance is off by a consistent factor | `wheel_radius` miscalibrated | Re-run the 1 m drive calibration (§4) |
| Odometry rotation is off by a consistent factor | `wheel_separation` miscalibrated | Re-run the full-turn spin calibration (§4) |
| Costmap spam: "map frame does not exist" | AMCL/slam_toolbox hasn't localized yet | Normal at startup; see §6 to localize |
| `/scan` has data but nothing renders in Foxglove | Fixed frame is `map` but `map→odom` doesn't exist yet, so `laser_frame` can't be placed in `map` | Temporarily set Fixed frame to `laser_frame` to see the raw scan, or just localize (§6) |
| Two nodes fighting over `odom→base_footprint` (jittery pose) | `rover_odometry`'s `publish_tf` param doesn't match the launch mode | SLAM launch → `publish_tf:=True`; Nav2 launch → `publish_tf:=False` (EKF owns it) |
| RealSense pointcloud topic missing | Wrong param name — Jetson build uses the NEON variant | Use `pointcloud__neon_.enable`, not `pointcloud.enable` |
| RealSense fails to open / "Device or resource busy" | Booth demo container still owns the camera | `docker rm -f realsense_demo` |
| RealSense depth capped around ~7-8Hz, 640×480 | Camera is on a USB-2 port | Move the D455 to a USB-3 (blue) port |
| RealSense IMU always fails ("HID Motion Sensor Failure") | The apt/RSUSB backend has no access to the D455's HID motion sensor — expected, not a bug | Not needed for depth navigation; would require a from-source kernel-backend librealsense build to fix |

---

## 8. Known open items / placeholders (don't be surprised by these)

- **Camera mount TF is still a placeholder** in `realsense.launch.py`
  (`base_link → camera_link` at x=0.20, z=0.20, level). Measure the real
  mount and edit it — until then, depth obstacles will be mis-placed in the
  costmap.
- **`drivetrain_fault_report.docx`** is an OLD, SUPERSEDED document from when
  the motors appeared broken. They were later proven fully healthy — the real
  cause was marginal encoder wiring, not defective hardware. It's stamped
  SUPERSEDED in the file itself; don't act on its conclusions.
- The D455 is **time-shared** with the booth demo on this same Jetson — only
  one process can hold the camera at a time (`docker rm -f realsense_demo`
  frees it for the rover).

---

## 9. Where things live (quick file map)

| What | Path |
|---|---|
| ESP32 firmware (production) | `uros_ws/src/esp32_rover_firmware_v2/src/main.cpp` |
| Standalone motor test/diagnostic firmwares | `uros_ws/src/motor_test/` (envs: `left_only`, `right_only`, `both`, `diag`, `dirprobe`, `health`) |
| Teleop node | `ros2_ws/src/rover_core/rover_core/rover_teleop_v2.py` |
| Odometry node | `ros2_ws/src/rover_core/rover_core/rover_odometry.py` |
| SLAM/mapping launch | `ros2_ws/src/my_robot_bringup/launch/slam_teleop.launch.py` |
| Navigation launch | `ros2_ws/src/my_robot_bringup/launch/master_navigation.launch.py` |
| Camera launch (separate, optional) | `ros2_ws/src/my_robot_bringup/launch/realsense.launch.py` |
| Nav2 costmap/params | `ros2_ws/src/my_robot_bringup/config/nav2_params.yaml` |
| EKF params | `ros2_ws/src/my_robot_bringup/config/ekf.yaml` |
| Robot URDF | `ros2_ws/src/rover_description/urdf/rover.urdf` |
| Saved map (for Nav2) | `ros2_ws/src/my_robot_bringup/maps/my_room_map.{pgm,yaml}` |
| RMCS-2303 manual | `rmcs_2303_manual.md` |
| Command cheat-sheet | `COMMANDS.md` |


---

## Raw RPLIDAR health check

Talks to the lidar directly over serial, bypassing ROS entirely. Use it whenever
you are unsure whether a lidar problem is hardware or configuration. Nothing else
may hold the port — stop the lidar node first (`pkill -f rplidar_node`).

```bash
timeout 15 python3 -c "
import serial, time
s = serial.Serial('/dev/ttyLIDAR', 115200, timeout=2)
s.dtr = False
s.reset_input_buffer()
s.write(b'\xA5\x25'); time.sleep(0.3)          # STOP
s.reset_input_buffer()
s.write(b'\xA5\x52'); time.sleep(0.5); h = s.read(10)   # GET_HEALTH
s.reset_input_buffer()
s.write(b'\xA5\x50'); time.sleep(0.5); i = s.read(27)   # GET_INFO
print('HEALTH ->', h.hex() if h else 'NO RESPONSE')
print('INFO   ->', i.hex() if i else 'NO RESPONSE')
if len(i) >= 27: print('  model=%d  fw=%d.%d  hw=%d' % (i[7], i[9], i[8], i[10]))
if len(h) >= 10: print('  health =', {0:'GOOD',1:'WARNING',2:'ERROR'}.get(h[7], h[7]))
s.close()"
```

**Reading the result**

| Output | Meaning | Do this |
|---|---|---|
| `NO RESPONSE` (or the open itself throws `Errno 5`) | Data path is broken. Power works, data does not. | Replug the USB cable, try a different Jetson port, use a powered hub. It is not a software problem. |
| `health = GOOD` + a model number | Hardware and cable are **fine**. Any remaining failure is configuration. | Check `scan_mode` matches the model (below). |
| `health = ERROR` | Lidar reports an internal fault. | Power-cycle it; if it persists, the unit needs service. |

**Model number → supported scan modes**

| `model=` | Unit | Valid `scan_mode` |
|---|---|---|
| 24 (0x18) | **A1M8 — this rover** | `Standard`, `Express` |
| 40 (0x28) | A2M8 | `Standard`, `Express`, `Boost`, `Sensitivity` |
| 73 (0x49) | A3M1 | `Standard`, `Express`, `Boost`, `Sensitivity` |

Asking for a mode outside that list is what produces `80008004`.


---

## USB power and topology (the root cause of most "random" failures)

Both the lidar and the ESP32 are CP210x USB-serial devices that need a *stable*
link. On 2026-09-07 both went down with `[Errno 5]` at once. The cause was not
either device — it was how they were plugged in.

**Check the tree first, before suspecting a device:**
```bash
lsusb -t
```

**The bad layout that caused it:**
```
Bus 01
  └── Realtek 0bda:5420  <-- the Orin devkit's ONBOARD hub. Self-powered
        │                    (MaxPower=0mA). Everything goes through it.
        │                    This one is fine and cannot be avoided.
        ├── Huasheng 214b:7250  <-- an EXTERNAL bus-powered hub (MaxPower=100mA)
        │     ├── CP210x  (LIDAR)          100 mA
        │     ├── CP210x  (ESP32)          100 mA
        │     └── Logitech C270 webcam     500 mA   <-- alone exceeds the hub
        └── RealSense D455                 496 mA
```
A **bus-powered** hub can supply only ~100 mA per downstream port, so the webcam
alone broke the sub-hub that both serial devices depended on.

**Telling the two kinds of hub apart — this is the important bit:**

| `bMaxPower` | Kind | Verdict |
|---|---|---|
| **0 mA** | Self-powered (onboard devkit hub, or a hub with its own wall adapter) | Fine. Not a power limit. |
| **100 mA** (or any non-zero) | Bus-powered — steals from upstream, gives ~100 mA per port | **This is what starves devices.** |

The Realtek pair `0bda:5420` (USB 2.0) + `0bda:0420` (USB 3.0) is the **AGX Orin
devkit's built-in hub**. Every USB-A port on the devkit hangs off it, so seeing a
"hub" in `lsusb -t` does **not** mean you plugged one in. Do not chase it.

**Rules that keep it stable**
- **Never add an external bus-powered hub.** Check with the `bMaxPower` loop
  below — non-zero means bus-powered.
- **Do not put a camera behind a bus-powered hub** alongside the serial devices.
- **Unplug peripherals you are not using.** Neither the D455 nor a webcam is
  needed for SLAM mapping.
- If you must run everything at once through a hub, use a **powered** one (its
  own wall adapter — it will then report `MaxPower=0mA`).

**Known-good layout (verified 2026-09-07):**
```
Orin onboard hub (self-powered, 0 mA)
  ├── CP2102   ESP32   100 mA
  ├── CP2102N  LIDAR   100 mA
  └── D455             496 mA
```

Check declared draw per device:
```bash
for f in /sys/bus/usb/devices/1-*/; do
  [ -f "$f/product" ] && echo "$(basename $f): $(cat $f/product) | $(cat $f/bMaxPower)"
done
```

---

## `ros2 run` says "No such file or directory" for a script that clearly exists

A confusing one, seen with `esp32_reset.py`:
```
FileNotFoundError: [Errno 2] No such file or directory:
  '.../install/my_robot_bringup/lib/my_robot_bringup/esp32_reset.py'
```
but `ls -l` shows the file present and executable.

**Why:** on Linux, `execve` returns `ENOENT` when the **interpreter named in the
shebang** is missing — not only when the script is missing. setuptools had
rewritten the installed copy's shebang to `#!python`, and Ubuntu 22.04 has only
`python3`. Python then reports the *script* as missing, which sends you looking
in the wrong place entirely.

**Check it:**
```bash
head -1 install/<pkg>/lib/<pkg>/<script>.py
```
`#!python` is broken. `#!/usr/bin/python3` is correct.

**Fix (permanent):** do not install scripts with `scripts=[...]` in `setup.py` —
setuptools mangles the shebang. Put the script inside the package module with a
`main()` and register it under `entry_points['console_scripts']`, which always
generates a correct shebang. This is what `my_robot_bringup` now does.


---

## Map builds as a noisy starburst with no straight walls

Symptom: after driving a full perimeter lap, the saved map is scattered dots and
radial spokes; the robot's displayed pose drifts far from where it physically is.

Two independent causes were found on 2026-09-07. Check both.

### 1. Scan mode too low-resolution (the big one)

`Standard` is the A1M8's **lowest** sample rate. Measured on the same stationary
scene:

| | Standard | Express |
|---|---|---|
| Sample rate | 2 kHz | **4 kHz** |
| Angular resolution | 1.00 deg | **0.50 deg** |
| Valid returns | 35.8% | **66.1%** |
| Bins hit on *every* scan | **0 / 360** | **383 / 720** |
| Points per scan | ~129 | **~476** |

Why `Standard` fails: with `angle_compensate: true` the driver spreads however
many measurements it got across a fixed bin array. At 2 kHz and ~7.5 rev/s that
is only ~267 measurements for 360 bins — a **74% maximum fill**, and which bins
land where shifts each revolution as motor speed drifts. So a bin aimed at a
solid wall still reads `inf` about a quarter of the time. **Zero bins were
reliable.** Scan matching cannot converge on walls that flicker.

**Use `Express`.** `Sensitivity`/`Boost` remain invalid on an A1M8.

### 2. The rover sees its own structure

Per-sector median range while stationary exposed it:
```
-150..-120 deg   median 0.47 m   70.5% of returns under 0.6 m   <== SELF-HIT
```
Chassis structure in the beam produces a phantom obstacle that travels with the
robot, so SLAM smears it across the map on every turn.

**Fix:** `min_laser_range: 0.55` in `config/slam_params.yaml` (now set). Better
still, physically clear whatever is in the beam.

### Diagnosing this yourself

Park the rover, leave it stationary, and check **per-bin reliability** — not the
average. The average hides the problem; "no bin is ever reliable" is the tell.
Scripts used are in the session scratchpad pattern:
- count valid returns per 15-degree sector -> finds blocked directions
- count how often each bin returns across 20 scans -> finds flicker
- median range per sector -> a sector stuck under 0.6 m is self-occlusion


---

## Odometry was open-loop (the root cause of every failed map)

Until 2026-09-07 the ESP32 did **not** measure anything. `/wheel_ticks` was
integrated from the *commanded* RPM:

```c
g_left_ticks  += (cur_left  / 60.0f) * ENCODER_CPR * dt;   // OLD - removed
```

That assumes the motors instantly hit the commanded speed and never slip, and it
had two defects that only showed up when turning.

### Proving it — the method matters more than the answer

Drive **scripted** motions, not hand-driven ones. Every hand-driven capture came
out mixed (`linear.x` *and* `angular.z` non-zero) and gave contradictory answers.
Publish to `/cmd_vel` directly, one axis at a time, and compare `/wheel_ticks`
against `/encoder_ticks`:

| Motion | Commanded (old `/wheel_ticks`) | Real (`/encoder_ticks`) |
|---|---|---|
| **Forward** | dL=+5900 dR=+5900 | dL=**-1976** dR=+1922 |
| **Spin** | dL=-6898 dR=+6898 | dL=+2248 dR=**+2186** |

Read the **sum** and **diff**, not the raw numbers — `rover_odometry` uses
`d_s=(dR+dL)/2` for distance and `d_th=(dR-dL)/L` for rotation. Driving forward
produced sum~0 / large diff (i.e. "pure spin") and spinning produced large sum /
diff~0 ("pure translation"). Exactly inverted.

### Defect 1 - left encoder sign

The left motor is mirror-mounted (the one driven `dir_cmd(..., inverted=true)`),
so its encoder counts **down** as the wheel rolls the robot forward. Firmware now
publishes `ticks_msg.x = -enc_msg.x`.

### Defect 2 - wrong ENCODER_CPR

`|real| / |commanded| = 0.326..0.335` on both wheels in both directions —
constant, so not slip. `1336/4000 = 0.334`: `ENCODER_CPR` was 4000 but the drive's
encoder is **334 lines x 4 = 1336 steps per motor revolution** (RMCS-2303 manual,
Address 10). A constant ratio like this **proves the motors do reach commanded
RPM** — the drives were never at fault.

### Why "calibration" had hidden it

`wheel_radius` had been set to 0.0257 against a nominal 0.05 — a fudge absorbing
part of the CPR error. Two compensating errors agree at exactly one speed and
load. When the motors were made faster the operating point moved and odometry
broke, which is why turning degraded after previously "working".

**Keep every constant physically meaningful.** A wheel radius should be the wheel's
actual radius. If a fudge factor is needed, the model is wrong.

### Calibrating honestly

1. **Linear** — scripted straight drive, tape-measure the distance:
   `encoder_cpr = 2*pi*wheel_radius * ticks / distance`
   Measured: 3921 ticks (1.6% skew) over 0.45 m at R=0.05 -> **2737**.
   Cross-check: 2737/1336 = 2.05, so the gearing is ~2:1 - and that independently
   explains why a commanded 0.12 m/s produced only ~0.056 m/s.
2. **Rotation** — spin to a *reported* 720 deg, read the actual angle by eye:
   `L_new = L_old * (reported / actual)`
   Two runs gave 0.4441 and 0.4549; mean **0.4495**, matching the ~45 cm physical
   track. Use 720 deg rather than 360 - it doubles the error signal.

### Current values (`rover_odometry.py`)

| Parameter | Value | Source |
|---|---|---|
| `wheel_radius` | 0.05 | measured wheel, 10 cm diameter |
| `encoder_cpr` | 2737 | scripted drive + tape measure |
| `wheel_separation` | 0.4495 | two scripted spins, mean |

`/wheel_ticks` now runs at ~6.5 Hz (150 ms Modbus poll, ~50% bus utilisation),
carries **real** counts, and passes the driver fault flags through in `z`
(0=OK, 1=left read failed, 2=right, 3=both). Old firmware kept as
`main.cpp.bak-openloop`.


---

## Verifying where the lidar actually is and which way it faces

A wrong `laser_frame` transform smears every map: the scan is laid down at the
wrong place or angle relative to the robot, so scan matching fights odometry.
On 2026-09-07 the URDF was wrong on **both** counts -- lidar declared at the front
with zero yaw when it is actually at the **rear, yawed 180 deg**.

### The trap: self-hit geometry is ambiguous

The rover's own deck corners show up in the scan at a predictable angle, and it is
tempting to infer the mount from that. **It cannot distinguish front-facing-forward
from rear-facing-backward** -- both put the corners at +/-149.6 deg for this
chassis. Inferring the mount from self-hits alone will silently give you the wrong
answer half the time.

### The test that actually works

Stand (or place a box) **directly in front of the rover, about 1 m out**, hold
still, and find the widest cluster in the scan:

- cluster centred near **0 deg** -> the lidar faces forward, `rpy="0 0 0"`
- cluster centred near **180 deg** -> the lidar is yawed 180 deg,
  `rpy="0 0 3.14159"`
- centred near **+/-90 deg** -> yawed a quarter turn; use `+/-1.5708`

Watch for the cluster wrapping the +/-180 boundary: it appears as *two* runs, one
near +180 and one near -180. They are one object. Add them before judging the
centre.

### Then confirm the position

With the yaw known, the self-hit geometry becomes usable. For this 44x44 cm deck
with the lidar longitudinally centred, the deck corners at the far end sit at
`sqrt(dx^2 + 0.22^2)` and `atan2(0.22, dx)`. Check the predicted distance and
angle against the measured self-hits; if they match, the position is right.

Cross-check with a third fact if you can -- here the mast, 22.5 cm forward of the
lidar, showed up as roughly half the returns missing in the 170-180 deg bins,
which independently confirmed the 180 deg yaw.

### Which edge did you measure from?

State it explicitly. "The lidar is 2-11 cm along and the mast is right behind it"
is ambiguous about which edge the tape started at, and getting it wrong flips the
sign of X -- a 31 cm error on this chassis.


---

## SLAM mode vs Navigation mode

**SLAM builds a map while you drive it. Navigation loads that map and drives
itself.** They are mutually exclusive -- never run both launches at once.

| | `slam_teleop.launch.py` | `master_navigation.launch.py` |
|---|---|---|
| Map | created live, in RAM (lost on Ctrl+C) | loaded from `room_map_v4.yaml` |
| Pose estimate | slam_toolbox, while mapping | AMCL, matching scans to the known map |
| Who drives | you, via teleop | Nav2 planner + controller |
| Starting pose | always (0,0) by definition | **you must supply it** via `/initialpose` |
| Obstacle avoidance | none | costmaps (lidar + D455 pointcloud) |
| Output | a map to save | motion toward a goal |

### Who owns each TF link -- the rule that breaks things

Both modes need the same chain, but different nodes publish it. **Exactly one
node may own each link.**

```
SLAM mode                          Navigation mode
map   <- slam_toolbox              map   <- AMCL
odom  <- rover_odometry            odom  <- EKF (robot_localization)
base_footprint <- robot_state_publisher (URDF, both modes)
base_link -> laser_frame, wheels   (URDF, both modes)
```

This is why the nav launch runs `rover_odometry` with **`publish_tf: False`** --
it still publishes the `/odom` *topic* for the EKF to fuse, but must not publish
the *transform*, because the EKF does. Two publishers on one link gives
`base_footprint` two parents and TF lookups become non-deterministic: the robot
flickers between poses, or scans intermittently land in the wrong place.

`config/ekf.yaml` sets `base_link_frame: base_footprint` for the same reason.

### Nav launch startup order

```
t=0s   esp32_reset.py          pulse DTR/RTS to reboot the ESP32
t=0s   robot_state_publisher   URDF TF chain
t=0s   rover_odometry          /odom topic only (publish_tf: False)
t=0s   EKF                     fuses /odom -> publishes odom->base_footprint
t=3s   micro_ros_agent         /wheel_ticks, /cmd_vel, /encoder_ticks
t=5s   rplidar_node            /scan, Express mode, respawn=True
       nav2_bringup            map_server, AMCL, planner, controller,
                               behaviour server, BT navigator, costmaps
       foxglove_bridge         ws://<jetson-ip>:8765
```
The delays exist because the ESP32 reset disturbs the USB bus; starting the agent
or the lidar too early catches it mid-reset.

Override the map without editing the launch file:
```bash
ros2 launch my_robot_bringup master_navigation.launch.py \
     map:=/home/rptech/AGX_Orin_Backup/rover_project/maps/<name>.yaml
```

### Failure modes unique to navigation mode

| Symptom | Cause | Fix |
|---|---|---|
| AMCL never converges; robot pose wanders | `/initialpose` not set, or set in the wrong place | Publish a `PoseWithCovarianceStamped` where the rover physically is. The scan should visibly snap onto the mapped walls. |
| Goal accepted, robot does not move | teleop is running and publishing zeros over the controller | `ros2 topic info /cmd_vel` -- there must be exactly ONE publisher. Ctrl+C teleop. |
| Plans through walls, or refuses an obvious path | wrong map loaded, bad initial pose, or phantom costmap obstacles | Confirm the map name in the launch output; re-set the initial pose. |
| `Lookup would require extrapolation into the future` | TF timing, usually two publishers on one link | Check `rover_odometry` has `publish_tf: False` and that no SLAM launch is still running. |

**Teleop is deliberately NOT in the nav launch.** Under Nav2 the controller owns
`/cmd_vel`, and a teleop node publishing zeros on its timer will fight it. Start
teleop in its own terminal only when you want manual override, and stop it before
sending a goal.


---

## Telling a self-hit from furniture (a mistake that cost real capability)

On 2026-09-07 a stationary sweep showed close returns at +/-138..159 deg,
median ~0.43 m. The rover's deck corners were *calculated* to sit at +/-149.6 deg
and 0.435 m. The numbers matched, so they were declared self-hits and suppressed
with a blanket minimum range: `min_laser_range: 0.55` for SLAM and later
`obstacle_min_range: 0.55` in both costmaps.

**The diagnosis was wrong.** A later sweep from a different parking spot found
**0.0% of returns under 0.6 m in every sector**. The original close returns were
furniture the rover happened to be parked beside. A geometric match alone is not
evidence -- with the corners near the lidar's own radius, plausible-looking
matches are easy to find.

The cost was real: at `obstacle_min_range: 0.55` the rover was blind inside 55 cm
while its own footprint radius is only 31 cm, leaving 24 cm of reaction distance.
It drove into a 30 cm box directly ahead that it could not see.

### The test that actually distinguishes them

A self-hit is rigidly attached to the robot. Furniture is not.

1. Run `python3 tools/check_lidar.py` and note any sector with a median under
   ~0.6 m.
2. **Physically move or rotate the rover to a different spot** and run it again.
3. Same angle, same distance -> genuinely a self-hit. Gone or moved -> it was
   the room.

Step 2 is the whole test, and skipping it is what went wrong here.

### If it IS a self-hit, do not use a minimum range

A minimum range is a blunt instrument: it blinds the robot in **every** direction
to solve a problem in **one**. Options in order of preference:

1. **Move the lidar** so nothing is in the beam. Always the right answer.
2. **Filter by angle AND range together** - drop a return only if it is both in
   the known self-hit sector and closer than the cutoff, so obstacles elsewhere
   and distant returns through that sector still get through. A node for this
   exists at `rover_core/scan_filter.py`; it is deliberately NOT in any launch
   file, since no self-hits were ultimately found.
3. A blanket minimum range only if neither is possible, and then size it against
   the footprint radius (~0.31 m here), not by guesswork.


---

## RealSense D455 on this rover

Calibrated mount (2026-09-09): **x=0.24, y=0.0, z=0.850, pitch=0.166 rad
(9.51 deg NOSE-DOWN)**, verified by floor-plane fit -- 10024 inliers, 8 mm
residual, residual pitch error -0.19 deg.

Role: **mid/far field and tall obstacles only.** With 9.5 deg of downward tilt
the lowest ray leaves at -38.5 deg, so the floor is invisible closer than
`0.85 / tan(38.5) = 1.07 m`. Low objects near the base are the **lidar's** job.
The camera feeds the LOCAL costmap's `voxel_layer` only.

### Three things that will waste your afternoon

**1. NEVER `kill -9` the RealSense node.** The driver does not release its V4L2
buffers, and every subsequent open fails with:
```
Frames didn't arrive within 5 seconds
xioctl(VIDIOC_QBUF) failed  Last Error: No such device
```
A graceful stop afterwards does NOT undo it -- only a physical replug does. Use
Ctrl+C, or `kill -INT`, and allow ~10 s.

**2. The booth watchdog will steal the camera mid-launch.** Cron runs
`~/realsense_demo/booth_watchdog.sh` **every 2 minutes** and relaunches the
`realsense_demo` container. Only one process may own a D455, so `docker stop`
alone is not enough -- the watchdog resurrects it within 2 minutes, grabs the
device while the ROS node is initialising, and leaves it wedged.

Pause it before any rover camera work:
```bash
crontab -l > ~/crontab.backup
crontab -l | sed 's|^\*/2 \* \* \* \* /home/rptech/realsense_demo/booth_watchdog.sh|#PAUSED-FOR-ROVER &|' | crontab -
docker stop realsense_demo
```
Restore afterwards -- do not leave the booth without its self-heal:
```bash
crontab -l | sed 's|^#PAUSED-FOR-ROVER ||' | crontab -
docker start realsense_demo
```

**3. Do NOT set `depth_module.depth_profile`.** On this unit ANY explicit depth
profile override kills the stream. Tried `480x270x15` and `640x360x30`; both
produced topics that publish at ~30 Hz while the depth image is **99.4% zero
pixels**. The driver default (848x480) works at 78-84% valid. Control data
volume with `decimation_filter.filter_magnitude: 3` instead (848x480 -> 284x160,
~38k points per cloud).

**Diagnose depth by PIXELS, never by topic rate.** The pointcloud publishes at a
healthy 30 Hz whether or not it contains any valid depth -- that is how this hid
for so long.

### Sign convention, verified empirically

In ROS, **positive pitch is nose-down**. Measured: with `pitch=0` declared and
the camera physically tilted down, the floor rose +15 deg with range; setting
`pitch=+0.26` flattened it to -1.5 deg. A nose-down camera declared level makes
the floor project upward, and Nav2 sees a wall rising ahead and refuses to move.

### Calibrating the mount

`python3 tools/check_camera_mount.py` fits a plane to the floor by RANSAC and
reports the pitch, roll and height error directly, plus the exact command to fix
them. It supersedes an earlier per-range-band slope method that gave
contradictory answers -- that version assumed a level camera when computing the
floor blind zone (so it discarded the densest near bands once the camera was
tilted) and used least squares, which a single sparse far band could drag from a
true 2.6 deg error to a reported 26 deg.


---

## "Nav2 is confused / AMCL doesn't work / can't follow a straight line"

On 2026-09-09 every one of those symptoms turned out to be **the rover not
executing its commands**. Nav2 was fine. AMCL was fine. The rover was ignoring
most of what it was told, and everything downstream inherited that.

Measured before the fix: commanded 0.60 m over 4 s, travelled **0.235 m**, all in
one 1.5 s burst, then frozen while /cmd_vel streamed in at 20 Hz. Repeat runs
gave 0.066 / 0.309 / 0.120 m -- erratic, which is exactly why it looked like a
localisation problem.

### Three firmware causes, all in the same place

The ESP32 loop does long BLOCKING Modbus operations and services micro-ROS
between them. Starve that and commands are silently lost.

1. **`ENC_POLL_MS = 150`** (was 500, I raised it for faster odometry). Two
   `mbus_read_pair` calls cost ~70-200 ms, so at 150 ms the next poll started as
   the last finished. The loop saturated, agent pings timed out, and the
   **micro-ROS session was torn down and rebuilt every ~12 seconds**. Each
   teardown destroys the /cmd_vel subscription. Now **250 ms** -> zero teardowns
   in 45 s, still 4 Hz odometry.

2. **`delay(300)` inside `stop_motors()`** -- a 300 ms blackout of the serial
   transport, called *from the watchdog handler*, so it was self-reinforcing: a
   brief gap fired the watchdog, the blackout lost more messages, the watchdog
   fired again. Replaced with a loop that spins the executor while braking, so
   the symmetric-braking behaviour is kept without the blackout.

3. **`CMD_WATCHDOG_MS = 1000`** turned every dropped message into a full motor
   stop. Now **1500 ms**.

Also raised the executor slice from 5 ms to 15 ms, added a spin between the two
encoder reads, and cut the Modbus read deadline from 100 ms to 60 ms (a healthy
round trip is ~35 ms, so 100 ms only lengthened failures).

### Result

| | before | after |
|---|---|---|
| 0.6 m drive, 4 consecutive runs | 0.066 / 0.309 / 0.120 m | 0.512 / 0.510 / 0.513 / 0.516 m |
| closed 0.8 m square, closure error | 16.6-31.9% | **5.9%** |
| square sides | wildly inconsistent | 0.782 / 0.780 / 0.781 / 0.785 m |
| square turns (90 deg cmd) | 8.6 / 17 / 88.7 / 0 deg | 87.4 / 87.3 / 87.2 / 87.8 deg |

### /rover_diag -- use it before blaming Nav2

The firmware now publishes `/rover_diag` (Point32) at ~4 Hz:
- `x` = /cmd_vel messages received since boot
- `y` = watchdog stop events since boot
- `z` = `cur_left`, the RPM the firmware believes it last commanded

```bash
ros2 topic echo /rover_diag
```
Drive the rover and watch. If `x` is not climbing, commands are not reaching the
ESP32 -- no amount of Nav2 tuning will help. If `y` climbs during a drive, the
watchdog is stopping the motors, which means messages are being dropped.

Adding this ended hours of inferring ESP32 behaviour from the outside. **Reach
for it first** whenever the rover moves oddly.

### The general lesson

Before tuning any autonomy parameter, prove the base moves as commanded:
```bash
python3 tools/... trace     # or the scripted drive in the session scratchpad
```
A closed square is the sharpest single test -- a straight-line drive can look
perfect while yaw is broken, and a loop turns any yaw error into visible
position error.

---

## The saved map said unknown space was free floor

**Symptom:** the planner routes through areas the lidar has never seen —
outside the building, through a wall gap, across the unsurveyed half of the
room — even with `track_unknown_space: true` and `allow_unknown: false`.

**Cause:** `map_saver` writes this into the map YAML:

```yaml
free_thresh: 0.25
```

The PGM stores unknown as grey **205**. `map_server` converts it with
`occ = (255 - 205) / 255 = 0.196`, and then classifies:

```
occ > occupied_thresh -> 100 (wall)
occ < free_thresh     ->   0 (free)      <-- 0.196 < 0.25, so unknown lands here
otherwise             ->  -1 (unknown)
```

So **every never-surveyed cell is published as free floor**. `track_unknown_space`
has nothing to act on, because by the time the costmap sees the map there is no
unknown left in it. On `room_map_v6` that was 9204 of 20532 cells — 45% of the
grid — reported as open floor.

**Fix:** `free_thresh: 0.19`, which puts 0.196 back on the unknown side.

```bash
grep free_thresh maps/*.yaml        # all of them should read 0.19
```

Verify from the published topic, not the file:

```bash
ros2 topic echo /map --once --field data | tr ',' '\n' | sort | uniq -c
```

Three values are correct: `-1` unknown, `0` free, `100` occupied. If `-1` is
missing, the threshold is still wrong.

The console fixes this automatically on every map it saves. A map saved any
other way needs the YAML edited by hand.

**Why it is easy to miss:** the map *looks* right in RViz and Foxglove, because
they render 0 and -1 differently only by shade. The difference is invisible
until the planner draws a path through the car park.

---

## nav2_container dies with exit code -11 (SIGSEGV) right after "Configuring global_costmap"

**Symptom:** the navigation stack comes up, then `component_container_isolated`
dies with `exit code -11`. Looks like memory corruption. It is not.

**Cause:** a costmap layer's YAML block is missing its `plugin:` key. nav2's
own plugin loader (`nav2_util::get_plugin_type_param`, in
`node_utils.hpp`) does this with no fallback:

```cpp
if (!node->get_parameter(plugin_name + ".plugin", plugin_type)) {
  RCLCPP_FATAL(node->get_logger(), "Can not get 'plugin' param value for %s", ...);
  exit(-1);
}
```

`exit(-1)` inside one composed node terminates the **whole container
process** — every other nav2 node sharing it dies too, mid-construction,
and a segfault on the way down is exactly what shows up in the launch log.
The real error is one line above it:

```
[FATAL] [global_costmap.global_costmap]: Can not get 'plugin' param value for static_layer
```

**Always read the FATAL line, not just the exit code.** grep for it:

```bash
grep FATAL logs/navigation-*.log
```

**Found here 2026-09-15:** the global costmap's `static_layer` block had
carried only `map_subscribe_transient_local: True` since the very first
commit — no `plugin:` line — while the local costmap's copy always had one.
Whether this crashes depends on exact timing in the composed container, so
it did not fail every single launch; it looked intermittent, which delayed
finding it. Every layer entry in every costmap needs its `plugin:` key. If
you copy a layer block between the local and global costmap sections, copy
the whole thing.
