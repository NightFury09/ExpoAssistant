# Expo Rover

A two-wheeled robot that maps a room with a lidar, then drives itself to named
demo stations at an expo, steering around people and furniture on the way.

- **Brain:** NVIDIA Jetson Orin Nano running ROS 2 Humble.
- **Muscles:** an ESP32 that turns speed commands into motor-driver commands
  and reports the wheel encoders back. It talks to ROS through micro-ROS.
- **Everyday use:** one screen, the **Rover Console**. You drive, map, pick a
  map and send the rover to a station from there; no terminal needed.

If you only want to *use* the rover, read [Using it](#using-it) and then
[CONSOLE.md](CONSOLE.md). Setting up a new machine starts at
[Setting up a new machine](#setting-up-a-new-machine).

---

## How the pieces fit

```
 Jetson Orin Nano ─ ROS 2 Humble ─────────────────────────────────────────┐
 │                                                                         │
 │  Rover Console (:8080) ── starts/stops ONE stack at a time:             │
 │    web page / desktop app      Mapping:     SLAM Toolbox builds a map   │
 │                                Navigation:  AMCL + Nav2 drive to a goal │
 │                                                                         │
 │  /cmd_vel ──► micro-ROS agent ◄── /wheel_ticks ──► rover_odometry ──► /odom
 │  rplidar_node ──► /scan_raw ──► scan_filter ──► /scan                   │
 │  RealSense D455 (optional) ──► depth obstacles for Nav2                 │
 └───────────────┬───────────────────────────────┬─────────────────────────┘
                 │ USB serial (/dev/ttyESP32)    │ USB serial (/dev/ttyLIDAR)
        ESP32 · micro-ROS firmware          RPLIDAR A1M8
                 │ Modbus ASCII, 9600 baud, one serial port per wheel
        2 × RMCS-2303 motor drivers ──► 2 × RMCS-5012 motors with encoders
```

What happens when you press **W**:

1. The console publishes a velocity on `/cmd_vel`.
2. The **micro-ROS agent** forwards it over USB to the **ESP32**.
3. The ESP32 converts it to a speed for each wheel and writes it to the two
   **motor drivers** over Modbus.
4. Every 250 ms the ESP32 reads both **encoders** back and publishes the counts
   on `/wheel_ticks`.
5. **rover_odometry** turns the counts into the rover's position (`/odom`).

The **lidar** scan, with the rover's own mast cut out by `scan_filter`, is
what SLAM uses to build the map and what AMCL uses to find the rover on it.

Two safety stops are always on:

- **The console's deadman:** if the page stops sending (tab closed, Wi-Fi
  lost), the console zeroes `/cmd_vel`.
- **The ESP32's watchdog:** it stops the motors after **1.5 s** without a
  command.

---

## Using it

**Open the console** in either of two ways:

| where | how |
|---|---|
| on the rover's own screen | the **Rover Console** app. It opens full screen at login and needs no network. See [desktop/README.md](desktop/README.md) |
| from a laptop or phone on the same Wi-Fi | `http://<rover-ip>:8080`. Get the IP on the rover with `hostname -I` |

The console runs as a service from boot, so there is nothing to start.

**A normal expo day**, in short (details in [CONSOLE.md](CONSOLE.md)):

1. Power up. Wait for the console.
2. **MAP** tab → pick the map → **NAVIGATE**.
3. **SET POSE**: drag from where the rover really is, in the direction it
   faces, until the scan lines up with the walls.
4. Press **GO** next to a demo station.
5. To stop: **Esc**, or **EMERGENCY STOP**.

**If the rover moves oddly, check the drivetrain first.** Run:

```bash
ros2 topic echo /rover_diag
```

`x` counts the commands the ESP32 has received. If it is not climbing while
you drive, commands are not reaching the motors, and no navigation tuning will
help. This one check has saved more time than anything else here; see
[SESSION_STATE.md](SESSION_STATE.md).

---

## Setting up a new machine

From a fresh Ubuntu 22.04 (JetPack 6 on a Jetson) to a working rover. These
steps were tested from a fresh clone on the Orin Nano.

> **Keep the user `rptech` and the path `~/AGX_Orin_Backup/rover_project`.**
> Both are written into the service, its launcher, the launch files and the
> Nav2 parameters. (The folder name dates from restoring the project off the
> AGX Orin it used to run on.)

### 1. ROS 2 Humble

Install **ROS 2 Humble** and the build tools by following the official guide:
<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

- `ros-humble-ros-base` is enough. It is what the Orin Nano runs; the desktop
  variant only adds RViz.
- Also install `ros-dev-tools`, which provides `colcon` and `rosdep`.

### 2. Get the code, including submodules

```bash
git clone --recurse-submodules https://github.com/NightFury09/ExpoAssistant ~/AGX_Orin_Backup/rover_project
```

The lidar drivers and the micro-ROS agent are **git submodules**. If you cloned
without `--recurse-submodules`, fill them in afterwards:

```bash
cd ~/AGX_Orin_Backup/rover_project && git submodule update --init --recursive
```

### 3. Install the dependencies

`rosdep` reads every package's dependency list and installs what is missing:
Nav2, SLAM Toolbox, the RealSense driver, Foxglove bridge, pyserial and so on.

```bash
cd ~/AGX_Orin_Backup/rover_project
sudo rosdep init        # first time on this machine only; an "already exists" error is fine
rosdep update
rosdep install --from-paths uros_ws/src ros2_ws/src --ignore-src -r -y
```

Give it **both** workspaces in one command. `ros2_ws` needs the micro-ROS
agent, which is not an apt package; listing `uros_ws` as well lets rosdep see
that it is built from source here.

### 4. Build both workspaces, micro-ROS first

```bash
source /opt/ros/humble/setup.bash
cd ~/AGX_Orin_Backup/rover_project/uros_ws
colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
cd ../ros2_ws
colcon build --symlink-install --parallel-workers 2
```

| workspace | what it holds | time on the Orin Nano |
|---|---|---|
| `uros_ws` | the micro-ROS agent, the ESP32's bridge into ROS | ~1.5 min. **Needs internet:** the build downloads Micro-XRCE-DDS-Agent |
| `ros2_ws` | the rover's own packages and the lidar drivers | ~1 min |

Warnings on stderr from `micro_ros_agent`, `rplidar_ros` and `sllidar_ros2`
are normal. What matters is `Summary: N packages finished` with no `failed`.

`build/`, `install/` and `log/` are **not** in git: they are machine-specific
and every machine builds its own.

### 5. Name the USB devices

The ESP32 and the lidar both appear as `/dev/ttyUSB*`, in whatever order they
were plugged in. A udev rule gives them fixed names, which everything uses:

```bash
cd ~/AGX_Orin_Backup/rover_project
sudo cp ros2_ws/src/my_robot_bringup/config/99-rover.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/ttyESP32 /dev/ttyLIDAR
```

Both use the same USB-serial chip (CP2102), so the rule tells them apart by
**serial number** and only matches *these* units. If you replace the ESP32
board or the lidar's USB adapter, read the new serial number and edit the rule:

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -m1 '{serial}'
```

### 6. Start the console at boot

```bash
~/AGX_Orin_Backup/rover_project/systemd/install.sh
```

The console then runs at **http://\<rover-ip\>:8080**, starting at power-on even
with nobody logged in.

### 7. Optional: the console on the rover's own screen

```bash
~/AGX_Orin_Backup/rover_project/desktop/install.sh --autostart
```

This opens the console full screen at login. To have it appear at boot with no
keyboard, also turn on **Settings → Users → Automatic Login**.
[desktop/README.md](desktop/README.md) explains the trade-off.

### 8. Check it

```bash
curl -s localhost:8080/api/metrics | python3 -m json.tool | head -20
```

Then open the console, plug in the ESP32 and lidar, and start **MAP ROOM** from
the MAP tab. In the header, the **ESP32** chip should change from `off` to
`on`, and **lidar** should show a scan rate.

---

## The ESP32 firmware

- **Current firmware:** [`uros_ws/src/esp32_rover_firmware_v2`](uros_ws/src/esp32_rover_firmware_v2),
  a PlatformIO project.
- **Older version:** `esp32_rover_firmware` (v1) estimated wheel movement from
  the commanded speed instead of reading the encoders.

Flash it from a machine with PlatformIO installed. Stop the console's stack
first, because the micro-ROS agent holds the serial port:

```bash
cd ~/AGX_Orin_Backup/rover_project/uros_ws/src/esp32_rover_firmware_v2
pio run --target upload --upload-port /dev/ttyESP32
```

The micro-ROS library is pinned to its **`#humble` branch** in
`platformio.ini`. Do not unpin it. A newer branch still *publishes* to the
Humble agent, but silently *receives* nothing, so the rover ignores every
command. See [docs/memory/feedback_microros_version.md](docs/memory/feedback_microros_version.md).

---

## Hardware reference

### Wiring: ESP32 to the motor drivers

Each wheel has its own driver on its own ESP32 serial port. **The mapping below
comes from the firmware**, which was confirmed by driving. Older notes with the
two sides the other way round are wrong.

| wheel | driver (Modbus ID) | ESP32 port | ESP32 RX ← driver TX | ESP32 TX → driver RX |
|---|---|---|---|---|
| **left** | RMCS-2303, ID **2** | UART2 | GPIO **14** | GPIO **13** |
| **right** | RMCS-2303, ID **7** | UART1 | GPIO **16** | GPIO **17** |

- **Common ground:** each driver's ground must connect to the ESP32's ground.
  Without it the serial link is garbage.
- **Link settings:** Modbus ASCII, 9600 baud, 8N1. Leave jumpers JP4–JP6
  open.
- **Driver IDs:** set with jumpers JP1–JP3, and read **only at power-up**.
  Changing a jumper does nothing until a full power cycle. See
  [rmcs_2303_manual.md](rmcs_2303_manual.md) §4. That manual's note and its
  table disagree about ID 7, so copy the jumpers off the working drive rather
  than trusting either.
- **Left motor:** it is mounted mirror-image, so the firmware inverts its
  direction and negates its encoder count.

### Power

| rail | feeds | notes |
|---|---|---|
| 12 V | both motor drivers and motors | high current |
| 5 V, low ripple | the lidar | 4.9–5.5 V, ripple under 50 mV, up to 600 mA at start-up. Underpowered, it boot-loops or drops scans |

Keep the lidar's 5 V separate from the motor supply, so motor noise does not
reach the scanner.

### Measured constants (do not guess these)

| constant | value | where it lives | how it was found |
|---|---|---|---|
| wheel radius | 0.05 m | `rover_odometry`, firmware | measured, 10 cm wheels |
| wheel separation | 0.4495 m | `rover_odometry`, firmware | mean of two scripted 720° spins |
| encoder counts per wheel turn | 2737 | `rover_odometry` `encoder_cpr` | 3921 counts over a tape-measured 0.45 m |
| motor-to-wheel gear ratio | 2.049 | firmware `GEAR_RATIO` | 2737 ÷ 1336 counts per motor turn |
| lidar position | x −0.155, z 0.275 m, turned 180° | `rover_description/urdf/rover.urdf` | rear-mounted; checked with an object in front |

[SESSION_STATE.md](SESSION_STATE.md) has the full calibration table, including
the camera mount.

### Coordinate frames

```
map ──(AMCL or SLAM Toolbox)──► odom ──(rover_odometry)──► base_footprint
                                                            ├── base_link
                                                            ├── laser_frame
                                                            ├── left_wheel / right_wheel
                                                            └── camera (from realsense.launch.py)
```

---

## What is where

```
rover_project/
├── ros2_ws/src/
│   ├── my_robot_bringup/   launch files, Nav2 / SLAM config, maps, udev rule
│   ├── rover_core/         the console, odometry, scan filter, teleop
│   ├── rover_description/  the robot model (URDF)
│   ├── rplidar_ros/        lidar driver (submodule)
│   └── sllidar_ros2/       alternative lidar driver (submodule)
├── uros_ws/src/
│   ├── micro_ros_setup/ uros/   the micro-ROS agent (submodules)
│   ├── esp32_rover_firmware_v2/ the ESP32 firmware (PlatformIO) -- current
│   ├── esp32_rover_firmware/    v1, open-loop odometry -- superseded
│   └── motor_test/              one-motor-at-a-time test firmware (old left/right labels)
├── systemd/                the console service and its installer
├── desktop/                the console as a desktop app
├── reception/              visitor check-in station, a separate app (service off for now; see its DEPLOY.md)
├── tools/                  diagnostics that answer with a number
├── foxglove/               a saved Foxglove Studio layout
├── config/                 demo stations; created when the first one is saved
├── maps/                   maps saved from the console (not in git)
├── logs/                   console and stack logs (not in git)
└── docs/memory/            hard-won findings, one per file
```

---

## Where to read next

| document | read it when |
|---|---|
| [CONSOLE.md](CONSOLE.md) | you are using the rover: every button and the expo-day routine |
| [COMMANDS.md](COMMANDS.md) | you need a terminal: the service, logs, health checks, emergency stop |
| [MAPPING_GUIDE.md](MAPPING_GUIDE.md) | you are mapping a new room; it covers driving technique, not just buttons |
| [TROUBLESHOOTING.md](TROUBLESHOOTING.md) | something is wrong: how the system fits together and symptom → cause → fix |
| [SESSION_STATE.md](SESSION_STATE.md) | you are picking the project up: what works, what is untested, next steps |
| [NEXT_STEPS.md](NEXT_STEPS.md) | you want the parked work: D455 IMU, lidar riser, drive registers |
| [desktop/README.md](desktop/README.md) | the console app, and full screen at boot |
| [tools/README.md](tools/README.md) | the diagnostic scripts |
| [docs/memory/](docs/memory/README.md) | the findings that each cost hours, one per file |
| [DASHBOARD.md](DASHBOARD.md) | the DRIVE tab in detail: camera views and metrics |
| [rmcs_2303_manual.md](rmcs_2303_manual.md) | motor driver registers and jumpers |
| `drivetrain_fault_report.docx` | history only; its conclusion was overturned (see `docs/memory/project_direction_rootcause.md`) |
