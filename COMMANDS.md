# Rover — Command Reference

Startup and operations cheat‑sheet for the differential‑drive rover
(Jetson AGX Orin → micro‑ROS/ESP32 → RMCS‑2303 drivers → RMCS motors + RPLIDAR).

> **Every terminal must source both setup files first:**
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash
> ```
> The micro‑ROS **agent** needs a third source: `source ~/microros_ws/install/setup.bash`

---

## 1. Full SLAM bringup (the normal way to run everything)

**One command brings up the whole stack** (ESP32 reset → agent → odometry → RPLIDAR → SLAM → Foxglove):

```bash
source /opt/ros/humble/setup.bash && source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash && ros2 launch my_robot_bringup slam_teleop.launch.py
```

- Wait ~15 s. Watch the log for the agent connecting, `current scan mode: Sensitivity ... 10.0 Hz` (lidar), then SLAM registering the sensor.
- It prints a green **Foxglove** URL: `ws://192.168.3.224:8765`
- **Keep this terminal running the entire session.** Only Ctrl+C it when completely done (and after saving the map).

Then, in a **second terminal**, run teleop (below) to drive.

---

## 2. Teleop (drive the rover)

```bash
source /opt/ros/humble/setup.bash && source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash && ros2 run rover_core rover_teleop_v2
```

| Key | Action |
|-----|--------|
| `W` / `S` | Forward / Backward |
| `A` / `D` | Turn Left / Turn Right |
| `+` / `-` | Speed up / down (live, while driving) |
| `SPACE` / `X` | Stop |
| `ESC` | Quit |

Default start speed 0.40 m/s; range 0.05–1.50 m/s. Drive **slowly** for good maps.

---

## 3. Save the map

Run **while the launch/SLAM is still running** (separate terminal):

```bash
source /opt/ros/humble/setup.bash && source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f ~/AGX_Orin_Backup/rover_project/maps/my_map
```

Writes `maps/my_map.pgm` (image) + `maps/my_map.yaml` (metadata).
**If it says "Failed to spin map subscription", SLAM isn't running or hadn't started yet** — the map lives inside the running `slam_toolbox` node.

---

## 4. Reset the map / restart SLAM

**If the launch is still running** (fast — keeps lidar/odometry/agent up):
```bash
pkill -f async_slam_toolbox_node
```
then start a fresh SLAM in its own terminal:
```bash
source /opt/ros/humble/setup.bash && source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash && ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file ~/AGX_Orin_Backup/rover_project/ros2_ws/install/my_robot_bringup/share/my_robot_bringup/config/slam_params.yaml -p use_sim_time:=false
```
To reset again later: just Ctrl+C that SLAM terminal and re-run the command.

**If the launch is NOT running:** don't restart SLAM alone (nothing feeds it). Just re‑run the full launch (Section 1).

---

## 5. Run individual nodes (manual / debugging)

Only needed when NOT using the full launch. Each in its own terminal.

**micro‑ROS agent** (ROS ↔ ESP32 bridge):
```bash
source /opt/ros/humble/setup.bash && source ~/microros_ws/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyESP32 -b 115200
```

**Odometry** (publishes `/odom`, `/tf`, `/heading_deg`):
```bash
source /opt/ros/humble/setup.bash && source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash && ros2 run rover_core rover_odometry
```

**RPLIDAR** (publishes `/scan`):
```bash
source /opt/ros/humble/setup.bash && source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash && ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyLIDAR -p serial_baudrate:=115200 -p frame_id:=laser_frame -p scan_mode:=Sensitivity
```

---

## 6. Flash the ESP32 firmware (PlatformIO)

> **Free the serial port first** — the agent/monitor must be closed or the upload fails with "port busy":
> ```bash
> lsof -t /dev/ttyUSB0 2>/dev/null | xargs -r kill
> ```

**Main teleop firmware (firmware_v2):**
```bash
cd ~/AGX_Orin_Backup/rover_project/uros_ws/src/esp32_rover_firmware_v2 && pio run --target upload --upload-port /dev/ttyESP32
```

**Standalone motor test firmware** — pick one environment:
```bash
cd ~/AGX_Orin_Backup/rover_project/uros_ws/src/motor_test && pio run -e <ENV> -t upload --upload-port /dev/ttyESP32
```
| ENV | What it does |
|-----|--------------|
| `left_only`  | Left motor only, CW/CCW loop |
| `right_only` | Right motor only, CW/CCW loop |
| `both`       | Both motors, WASD sequence |
| `diag`       | Register dump + live speed/position sampling |
| `dirprobe`   | Direction-determinism probe |
| `health`     | Full motor health report card (PASS/FAIL) |

**Watch serial output** (for the standalone test firmwares — NOT firmware_v2, whose UART0 is used by micro‑ROS):
```bash
pio device monitor --port /dev/ttyESP32 --baud 115200
```

---

## 7. Rebuild ROS 2 packages (after editing source)

```bash
cd ~/AGX_Orin_Backup/rover_project/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select <package>
```
Packages: `rover_core` (teleop, odometry), `my_robot_bringup` (launch, configs),
`rover_description` (URDF). **Restart the affected node** after rebuilding for the change to take effect.

---

## 8. Health checks / verification

```bash
source /opt/ros/humble/setup.bash && source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash
```
| Check | Command |
|-------|---------|
| Running nodes | `ros2 node list` |
| Driver comms flag | `ros2 topic echo /encoder_ticks --field z` |
| Wheel command feedback | `ros2 topic echo /wheel_ticks` |
| Odometry pose | `ros2 topic echo /odom --field pose.pose.position` |
| Heading (deg, unwrapped) | `ros2 topic echo /heading_deg` |
| Lidar rate | `ros2 topic hz /scan` |
| TF map→odom | `ros2 run tf2_ros tf2_echo map odom` |

**`/encoder_ticks` `z` flag = instant driver diagnosis:**
`0` = both drivers OK · `1` = LEFT (id 2) silent · `2` = RIGHT (id 7) silent · `3` = neither (usually GND / power).

---

## 9. Shutdown / cleanup

```bash
# Stop everything cleanly: Ctrl+C the launch terminal. If nodes are orphaned:
pkill -9 -f "ros2 launch my_robot_bringup"
pkill -9 -f "slam_toolbox|micro_ros_agent|rover_odometry|rplidar_node|robot_state_pub|foxglove_bridge|rover_teleop"
# Confirm ports free:
lsof /dev/ttyUSB0 /dev/ttyUSB1 2>/dev/null || echo "PORTS FREE"
```

---

## Reference

**Serial devices:** `/dev/ttyESP32` → `ttyUSB0` (ESP32, 115200) · `/dev/ttyLIDAR` → `ttyUSB1` (RPLIDAR, 115200)
Modbus to drivers runs at **9600** baud.

**ESP32 ↔ RMCS‑2303 wiring** (cross TX↔RX; shared GND mandatory; 3.3 V direct):

| Wheel | Slave ID | ESP32 TX→RXD | ESP32 RX←TXD | GND |
|-------|----------|--------------|--------------|-----|
| LEFT  | 2 | GPIO 13 → Pin 2 | GPIO 14 ← Pin 3 | GPIO GND ↔ Pin 1 |
| RIGHT | 7 | GPIO 17 → Pin 2 | GPIO 16 ← Pin 3 | GPIO GND ↔ Pin 1 |

**Odometry calibration (in `rover_odometry.py`):** `wheel_radius = 0.0257 m`, `wheel_separation = 0.4621 m`.

**Foxglove:** connect to `ws://192.168.3.224:8765`; 3D panel → Fixed frame `map`; enable `/map` and `/scan`.

**Golden rules**
1. Source both setup files in every terminal.
2. Free the serial port before flashing (kill the agent).
3. Keep the launch running the whole time you map; save the map before stopping it.
4. Drive slowly for clean maps.
