---
name: project_esp32_loop_starvation
description: Rover "Nav2/AMCL broken" symptoms were the ESP32 dropping /cmd_vel — blocking Modbus starved micro-ROS; check /rover_diag first
metadata:
  type: project
---

On 2026-09-09 every "Nav2 is confused / AMCL doesn't work / can't drive
straight" symptom traced to **the ESP32 not executing commands**. Commanded
0.60 m, delivered 0.235 m in one burst then froze while /cmd_vel streamed at
20 Hz.

The ESP32 loop does long **blocking** Modbus operations and services micro-ROS
between them. Three things starved it:
1. `ENC_POLL_MS = 150` (two reads cost 70-200 ms) saturated the loop and the
   micro-ROS **session was torn down every ~12 s**, destroying the /cmd_vel
   subscription. Now 250 ms.
2. `delay(300)` in `stop_motors()` blacked out the transport, called from the
   watchdog handler, so stalls were self-reinforcing. Now spins the executor
   while braking.
3. `CMD_WATCHDOG_MS = 1000` turned any dropped message into a motor stop. Now
   1500 ms.

After: 0.6 m drives repeat to +/-0.6%; a closed 0.8 m square closes to 5.9%
with all four sides 0.78 m and all four turns ~87.4 deg.

**Why:** autonomy symptoms look like autonomy bugs. Hours went into AMCL,
costmap and DWB tuning against a base that was ignoring most commands.

**How to apply:** the firmware now publishes **`/rover_diag`** (x = cmd_vel
received, y = watchdog fires, z = cur_left RPM). `ros2 topic echo /rover_diag`
while driving — if x is not climbing, commands are not arriving and no Nav2
tuning will help. Always prove the base moves as commanded before touching
autonomy params; a **closed square** is the sharpest test, since a straight
drive can look perfect while yaw is broken.
Related: [[project_odometry_real_encoders]], [[project_parked_work]].
