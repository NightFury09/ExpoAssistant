# Web Dashboard

Drive the rover from a browser, with the RealSense feed and live metrics.
Companion to [COMMANDS.md](COMMANDS.md) and [TROUBLESHOOTING.md](TROUBLESHOOTING.md).

```bash
source /opt/ros/humble/setup.bash
source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash
ros2 launch my_robot_bringup teleop_dashboard.launch.py
```

Then open **http://192.168.3.224:8080** from any machine on the network.
The launch prints the URL once everything is up (~14 s).

Options:
```bash
ros2 launch my_robot_bringup teleop_dashboard.launch.py use_camera:=false
```

---

## Do not run it alongside SLAM or Nav2

`slam_teleop.launch.py` and `master_navigation.launch.py` each start their **own**
micro-ROS agent, `rover_odometry` and `rplidar`. Running two stacks gives two
publishers on `odom->base_footprint`; TF then returns whichever arrived last and
the robot's pose flickers between two estimates several times a second. Nothing
downstream works, and it looks like a localisation bug rather than a duplicate
process. This wasted hours on 2026-09-09.

Before launching anything, check:
```bash
ros2 node list | sort
```

---

## Controls

| Input | Action |
|---|---|
| `W` `A` `S` `D` (hold) | forward / left / back / right |
| On-screen arrows | same, click-and-hold, works on touch |
| `Space` or **STOP** | emergency stop; press again to re-arm |
| Speed slider | 0.05 - 1.00 m/s |
| Turn slider | 0.10 - 1.00 rad/s |

### Two independent deadmen

1. The **page** must keep sending. Release a key, close the tab, or lose Wi-Fi
   and commands stop within **0.6 s**.
2. The **ESP32** stops the motors after **1.5 s** of no `/cmd_vel`, whatever the
   Jetson is doing.

Commands are published at **10 Hz, deliberately not 20+**. The ESP32 loop does
blocking Modbus and its micro-ROS transport drops messages if flooded -- that
failure mode is documented in TROUBLESHOOTING.md and looked exactly like "Nav2
is broken" for several hours.

---

## Camera views

Four views, switched from the tabs in the camera panel header. Only the view you
are watching is encoded -- the node tracks which streams have been requested in
the last 5 s -- so the other three cost nothing.

| View | Topic | What it shows |
|---|---|---|
| **Colour** | `color/image_raw` | the plain RGB feed |
| **Depth** | `depth/image_rect_raw` | depth colourised with **Turbo** |
| **Overlay** | `aligned_depth_to_color/image_raw` | depth blended 55% over the colour frame |
| **IR** | `infra1/image_rect_raw` | the left infrared camera |

**For showing clients, lead with IR then Depth.** The IR view makes the
projector's dot pattern visible -- you can see the camera *painting* the scene
with structured light, which explains how it measures depth far better than
words. Then switch to Depth to show the result, and to Overlay to prove the two
line up.

**Turbo, not jet.** Turbo is perceptually monotonic, so a viewer reads "nearer"
and "further" correctly. Jet has a false bright band in the middle that makes
mid-range objects look like they pop forward.

### Colour ramp range

The legend under the video carries a slider for the **far limit** (1.5-10 m,
default 4 m). This matters more than it sounds: at a 6 m limit a normal indoor
scene sits entirely in the blue third of the ramp and looks flat, while at 3 m
the same scene uses the full violet-to-red range and reads instantly. **Set it
to roughly the depth of the space you are demoing in.**

Also on the HUD: **centre distance**, the median depth of a 9x9 patch at the
image centre -- point the rover at something and read the range off the badge.

### Cost

Enabling aligned depth and IR adds USB bandwidth and a little CPU. Both are
switched on in `realsense.launch.py`. If you ever need to claw back bandwidth,
`enable_infra1: False` and `align_depth.enable: False` drop the IR and Overlay
views while leaving Colour and Depth working.

---

## Metrics

**Motion** -- commanded vs *measured* linear and angular velocity. Divergence
between them means the rover is not doing what it is told; that is the first
thing to check when anything misbehaves.

**Odometry** -- x, y, heading, straight-line distance from start, and total path
length.

**Drivetrain & link** -- read straight from `/rover_diag`, the firmware's own
counters:

| Field | Meaning |
|---|---|
| cmd_vel received by ESP32 | must climb while you drive. If it does not, commands are not arriving and nothing on the Jetson will fix it. |
| Watchdog stops | should stay 0 during a drive. Climbing means messages are being dropped. |
| Commanded RPM (left) | what the firmware believes it last set |
| Wheel ticks L / R | real encoder counts |
| Encoder faults | 0 = both drives answering Modbus |

**Sensors** -- lidar rate, percentage of valid returns, **nearest obstacle in the
forward 60 deg arc**, and camera rate. The obstacle reading also drives a banner
across the video: amber under 1.0 m, red under 0.5 m.

The lidar is rear-mounted and yawed 180 deg, so the robot's forward direction is
180 deg in the scan frame. If the mount ever changes, update
`forward_lidar_angle_deg` on the `rover_dashboard` node.

---

## How it is built

`rover_core/rover_dashboard.py` is a ROS node that also serves its own web page:
`/` (page), `/stream.mjpg` (MJPEG), `/api/metrics` (JSON), `/api/cmd` and
`/api/estop` (POST).

Only PIL, numpy and the Python standard library -- no rosbridge, no web
framework. A ROS node has to bridge it regardless, since the page publishes
`/cmd_vel` and reads live topics.

The MJPEG stream is capped at 15 fps and downscaled to 640 px wide even when the
camera runs at 30 Hz, which keeps JPEG encoding off the critical path.
