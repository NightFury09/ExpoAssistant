# ESP32 Rover Firmware (micro-ROS)

PlatformIO project for the ESP32 differential drive rover.  
Subscribes to `/cmd_vel` (`geometry_msgs/Twist`) and commands left/right  
RMCS2303 motor controllers over Modbus (UART 1 & 2).

## Fixes vs original firmware

| Issue | Resolution |
|---|---|
| `delay()` inside ROS callback | Commands deferred to `loop()` via `g_new_cmd` flag |
| Executor spin timeout 100 ms | Reduced to 10 ms |
| No agent reconnection | State machine: `WAITING_FOR_AGENT` ↔ `AGENT_CONNECTED` |
| Fatal hang on ROS error | LED blink error indicator + clean entity destroy/rebuild |

## Hardware Wiring

| Signal | ESP32 Pin |
|---|---|
| UART1 RX (Left motor) | GPIO 16 |
| UART1 TX (Left motor) | GPIO 17 |
| UART2 RX (Right motor) | GPIO 14 |
| UART2 TX (Right motor) | GPIO 13 |

- Left motor slave ID : **1**
- Right motor slave ID: **2**
- Wheel radius: **0.05 m**, Wheel separation: **0.47 m**

## Build & Flash

```bash
# From this directory
pio run                  # build only
pio run --target upload  # build + flash
pio device monitor       # open serial monitor at 115200
```

## Running with micro-ROS Agent (on AGX Orin)

```bash
# Terminal 1 — start the agent (USB transport)
source /home/rptech/AGX_Orin_Backup/rover_project/uros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Terminal 2 — test: send a forward velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

## Changing Transport

Edit `setup()` in `src/main.cpp`:

```cpp
// USB Serial (default)
set_microros_transports();

// WiFi UDP
set_microros_wifi_transports("SSID", "PASSWORD", "192.168.1.100", 8888);
```
