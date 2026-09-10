---
name: feedback-microros-version
description: micro-ROS Arduino library must be pinned to the Humble branch; kilted version silently breaks subscriptions
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 94bb74df-26e3-4426-a987-fcd3fa136a8d
---

Always pin `micro_ros_arduino` to the Humble-compatible branch when using the Humble micro-ROS agent.

**Why:** The `kilted` version (default `HEAD` of micro_ros_arduino) uses a different type-hash format. When paired with a Humble agent, publishers still work (data FROM ESP32 reaches ROS 2) but subscriptions silently fail (data TO ESP32 never arrives). twist_callback never fires despite /cmd_vel being published and QoS matching.

**Fix applied (2026-07-03):** Changed `platformio.ini` lib_deps from `https://github.com/micro-ROS/micro_ros_arduino` to `https://github.com/micro-ROS/micro_ros_arduino.git#humble`. Delete `.pio/libdeps/esp32dev/micro_ros_arduino*` cache before rebuild.

**How to apply:** Any time micro_ros_arduino is reinstalled or platformio.ini is edited, verify the `#humble` branch pin is present. Agent version and library version must match ROS distro.
