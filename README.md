# Rover Project: Comprehensive Handover Documentation

This document serves as an exhaustive, production-grade technical manual and handover guide for the **Expo Rover Project**. It outlines the complete hardware integration, electrical wiring, ESP32 micro-ROS firmware architecture, ROS 2 node layout, kinematics equations, configuration schemas, and deployment commands.

---

## 1. System Architecture

The Expo Rover is a differential-drive mobile robot designed for indoor SLAM (Simulated Localization and Mapping) and autonomous navigation. The control stack is divided into a high-level coordination layer (running ROS 2 Humble on an NVIDIA Jetson AGX Orin) and a low-level actuation layer (running micro-ROS on an ESP32).

### Data Flow Overview

```
graph TD
    %% High-level ROS 2 Stack
    subgraph Host_Orin [NVIDIA Jetson AGX Orin (ROS 2 Humble)]
        A[rover_teleop] -- "/cmd_vel (Twist)" --> B[micro_ros_agent]
        C[rplidar_node] -- "/scan (LaserScan)" --> D[slam_toolbox]
        E[rover_odometry] -- "/odom (Odometry)" --> F[ekf_filter_node]
        E -- "/tf (odom -> base_footprint)" --> G[TF Tree]
        F -- "/odometry/filtered" --> D
        H[robot_state_publisher] -- "/tf (base_footprint -> laser_frame)" --> G
        I[foxglove_bridge] -- WebSocket (Port 8765) --> J[Foxglove Studio Web UI]
    end

    %% Low-level Actuation Stack
    subgraph Low_Level [Actuation & Sensing (ESP32 & Drivers)]
        B -- "USB Serial (115200 bps)" --> K[ESP32 micro-ROS Firmware]
        K -- "Modbus ASCII (Serial1, 9600 bps)" --> L[Left Motor Driver (RMCS-2303, ID 7)]
        K -- "Modbus ASCII (Serial2, 9600 bps)" --> M[Right Motor Driver (RMCS-2303, ID 2)]
        L -- "12V PWM" --> N[Left Motor (RMCS-5012)]
        M -- "12V PWM" --> O[Right Motor (RMCS-5012)]
        N -- "Quadrature Encoder" --> L
        O -- "Quadrature Encoder" --> M
        K -- "/wheel_ticks (Point32) at 20Hz" --> B
    end

    %% Sensor Link
    P[RPLIDAR A1M8 Scanner] -- "USB Serial (/dev/ttyLIDAR)" --> C
```

---

## 2. Hardware Integration & Electrical Connections

The low-level electronics isolate the high-current motor circuits from the low-power logic circuits to prevent voltage ripple from corrupting sensor readings or causing ESP32 brownouts.

### 2.1. Logic & Serial Wire Map

The ESP32 uses two independent hardware serial interfaces to communicate with the two RMCS-2303 motor drivers over 3.3V-TTL logic:

**Left Motor Connection (Serial1):**
- ESP32 **GPIO 17** (TX1) → Driver **RX**
- ESP32 **GPIO 16** (RX1) → Driver **TX**
- **Common Ground:** Ground pin near Pin 19 on the ESP32 must connect directly to the driver ground.

**Right Motor Connection (Serial2):**
- ESP32 **GPIO 13** (TX2) → Driver **RX**
- ESP32 **GPIO 14** (RX2) → Driver **TX**
- **Common Ground:** Ground pin near Pin 13 on the ESP32 must connect directly to the driver ground.

### 2.2. Power Supply Configuration

- **Low-Ripple Logic Power (5.0V DC):**
  - Supplies the **Scanner Core** of the RPLIDAR A1M8 (requires voltage between 4.9V and 5.5V, with ripple < 50 mV to prevent processing failure).
  - Startup current for the scanner core requires up to 600 mA; underpowering will trigger boot loops.
- **High-Current Actuation Power (12.0V DC):**
  - Connected in parallel to both **RMCS-2303 drivers**.
  - Supplies power for the **RMCS-5012 motors** and the RPLIDAR **rotation motor** (which accepts 5.0V to 10.0V DC to drive the belt-rotation assembly).

### 2.3. Motor Driver Jumper Addressing (RMCS-2303)

The motor drivers read their addressing jumpers **only at boot initialization**. Adjusting jumpers while powered has no effect until a complete power cycle is executed.

**Left Motor Driver (Slave ID 7):**
| Jumper | State  | Bit | Value |
|--------|--------|-----|-------|
| JP1    | CLOSED | 0   | 1     |
| JP2    | CLOSED | 1   | 2     |
| JP3    | CLOSED | 2   | 4     |

Resulting ID: 1 + 2 + 4 = **7**

**Right Motor Driver (Slave ID 2):**
| Jumper | State | Bit | Value |
|--------|-------|-----|-------|
| JP1    | OPEN  | 0   | 0     |
| JP2    | CLOSED| 1   | 2     |
| JP3    | OPEN  | 2   | 0     |

Resulting ID: **2**

**Communication Settings (JP4–JP6):** Leave **OPEN** to select default Modbus ASCII Digital Serial Mode at **9600 bps (8N1)**.

---

## 3. ESP32 micro-ROS Firmware

The ESP32 firmware is located at:
```
uros_ws/src/esp32_rover_firmware/src/main.cpp
```
It runs a micro-ROS node named `esp32_rover_controller`.

### 3.1. Register Map & Control Codes

The firmware communicates with the RMCS-2303 drivers by sending Modbus ASCII frames (no external driver libraries, preventing blocking calls and custom timing issues).

| Register        | Address | Description                    |
|-----------------|---------|--------------------------------|
| `REG_CONTROL`   | 2       | Command register               |
| `REG_SPEED`     | 14      | Target speed in RPM            |

| Control Code    | Value | Action                                      |
|-----------------|-------|---------------------------------------------|
| `CTRL_CW`       | 257   | Enable motor + rotate Clockwise             |
| `CTRL_CCW`      | 265   | Enable motor + rotate Counter-Clockwise     |
| `CTRL_DISABLE`  | 256   | Disable driver output stage (coasts to stop)|

### 3.2. Modbus ASCII Frame Structure

```
ASCII Frame:  : [Slave ID] [Func Code] [Reg Address] [Data] [LRC Checksum] \r\n
```

**LRC Checksum:** Sum of all frame bytes (excluding `:` and `\r\n`), negated (two's complement), converted to a hex byte.

### 3.3. Non-Blocking Command State Machine

To respect RMCS-2303 serial processing time constraints, speed and direction writes are separated by a 20ms delay via a non-blocking state machine (`McState` enum):

```
[*] --> MC_IDLE
MC_IDLE --> MC_WRITE_SPEED       : g_new_cmd == true
MC_IDLE --> MC_WAIT_DISABLE      : g_new_cmd && dir_change == true

MC_WAIT_DISABLE --> MC_WRITE_SPEED  : millis() - timer >= 20ms

MC_WRITE_SPEED --> MC_WAIT_SPEED    : Write speeds to REG_SPEED (14)

MC_WAIT_SPEED --> MC_WRITE_DIR      : millis() - timer >= 20ms

MC_WRITE_DIR --> MC_IDLE            : Write directions to REG_CONTROL (2)
```

**Direction Inversion Logic:** The Left motor is mounted physically inverted relative to the Right motor. Handled in firmware: `LEFT_MOTOR_INVERTED = true`, `RIGHT_MOTOR_INVERTED = false`.

**Safety Watchdog:** If no `/cmd_vel` command is received for > 1000 ms (`CMD_WATCHDOG_MS`), the node triggers `stop_motors_immediate()`.

### 3.4. Simulated Encoder Tick Generation

Since physical encoder outputs connect to RMCS-2303 controllers for internal PID speed regulation, the ESP32 integrates commanded speed over time at 20Hz (50 ms intervals) to generate simulated dead-reckoned ticks:

```
ΔTicks = (Commanded RPM / 60) × CPR × Δt
```

Published as `geometry_msgs/msg/Point32` on `/wheel_ticks` (`ticks_msg.x` = Left, `ticks_msg.y` = Right).

---

## 4. ROS 2 Workspace Structure

High-level ROS 2 Humble workspace: `rover_project/ros2_ws`

```
ros2_ws/src/
├── my_robot_bringup/     # Config, map metadata, launch scripts, udev rules
├── rover_core/           # Python nodes for odometry and manual control
├── rover_description/    # Robot URDF and link mappings
├── rplidar_ros/          # Slamtec RPLIDAR A1 integration package
└── sllidar_ros2/         # Alternate RPLIDAR package for Cartographer SLAM
```

### 4.1. Custom Node Implementations

#### `rover_odometry`
- **File:** `ros2_ws/src/rover_core/rover_core/rover_odometry.py`
- **Script:** `rover_odometry`
- Subscribes to `/wheel_ticks` (`geometry_msgs/Point32`), converts ticks to wheel displacement, calculates differential drive kinematics, publishes `/odom` (`nav_msgs/Odometry`) and `/joint_states` (`sensor_msgs/JointState`), and broadcasts the `odom → base_footprint` TF transform.

#### `rover_teleop`
- **File:** `ros2_ws/src/rover_core/rover_core/rover_teleop.py`
- **Script:** `rover_teleop`
- Interactive keyboard control using Python `curses`. Publishes latched velocity parameters to `/cmd_vel` at 10Hz.

#### `teleop_diagnostic`
- **File:** `ros2_ws/src/rover_core/rover_core/teleop_diagnostic.py`
- Subscribes to both `/cmd_vel` and `/wheel_ticks` to log command-to-actuation delays and expected wheel speeds.

---

## 5. Kinematics & Transformation Tree

### 5.1. Forward Kinematics (ESP32 Integration)

Standard differential drive equations (L = 0.44 m wheel separation, R = 0.05 m wheel radius):

```
v_left  = v - (ω × L) / 2
v_right = v + (ω × L) / 2

RPM_left  = v_left  × 60 / (2π × R)
RPM_right = v_right × 60 / (2π × R)
```

### 5.2. Odometry Kinematics (`rover_odometry`)

Runge-Kutta 2nd order integration:

```
Δθ = (Δd_right - Δd_left) / L
Δs = (Δd_right + Δd_left) / 2

x(k+1) = x(k) + Δs × cos(θ(k) + Δθ/2)
y(k+1) = y(k) + Δs × sin(θ(k) + Δθ/2)
θ(k+1) = θ(k) + Δθ
```

### 5.3. Transformation Tree Structure

```
odom
 └── base_footprint          [Dynamic: rover_odometry or EKF]
      ├── base_link           [Fixed: base_joint]
      ├── laser_frame         [Fixed: laser_joint, Z = 0.23 m]
      ├── left_wheel          [Continuous: left_wheel_joint, Y = +0.22 m, Z = 0.05 m]
      └── right_wheel         [Continuous: right_wheel_joint, Y = -0.22 m, Z = 0.05 m]
```

---

## 6. Startup & Deployment Guide

### 6.1. udev Setup (Device Symlinks)

```bash
sudo cp ~/AGX_Orin_Backup/rover_project/ros2_ws/src/my_robot_bringup/config/99-rover.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# Verify symlinks
ls -la /dev/ttyESP32 /dev/ttyLIDAR
# Expected:
# /dev/ttyESP32 -> ttyUSB0
# /dev/ttyLIDAR -> ttyUSB1
```

### 6.2. Building the Workspace

```bash
cd ~/AGX_Orin_Backup/rover_project/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 6.3. Running the Launch Stack

```bash
ros2 launch my_robot_bringup slam_teleop.launch.py
```

This resets the ESP32, connects the micro-ROS agent, starts SLAM, and initializes the Foxglove Bridge.

### 6.4. Running Keyboard Teleop

Open a separate terminal:

```bash
source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash
ros2 run rover_core rover_teleop
```

### 6.5. Visualizing via Foxglove Studio

1. Open Foxglove Studio (desktop or at `https://studio.foxglove.dev`).
2. Select **Open Connection** → **Foxglove WebSocket**.
3. Enter the WebSocket address from your launch terminal (e.g., `ws://192.168.3.224:8765`).

---

## 7. Diagnostics & Troubleshooting

**Run diagnostics to inspect latency and command execution:**
```bash
python3 src/rover_core/rover_core/teleop_diagnostic.py
```

**ESP32 Connection Issues:**
If `/wheel_ticks` stops publishing or the micro-ROS agent reports session errors, trigger a hardware reset:
```bash
ros2 run my_robot_bringup esp32_reset.py
```

**Unstable LiDAR Scanning:**
If SLAM reports dropped scan messages, verify the RPLIDAR input voltage. The scanning core requires a low-ripple 5V supply isolated from the motor drivers.

---

*Documentation generated for the Expo Rover Project — NVIDIA Jetson AGX Orin + ESP32 micro-ROS stack.*
