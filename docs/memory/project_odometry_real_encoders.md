---
name: project_odometry_real_encoders
description: Rover odometry now uses real encoders (was open-loop from commanded RPM); calibrated cpr=2737, radius=0.05, separation=0.4495
metadata:
  type: project
---

Until 2026-09-07 the ESP32 published `/wheel_ticks` by integrating the
**commanded** RPM, not by reading encoders. Two defects followed:

1. **Left encoder sign inverted** — the left motor is mirror-mounted, so its
   encoder counts down when the wheel rolls the robot forward. Firmware now
   publishes `ticks_msg.x = -enc_msg.x`.
2. **`ENCODER_CPR` was 4000**, but the RMCS-2303 encoder is 334 lines x 4 =
   **1336 steps per motor revolution**.

Symptom: driving forward looked like a pure spin to odometry and spinning looked
like pure translation, so every SLAM map came out as a noisy starburst and the
pose drifted badly at every turn.

Calibrated values now in `rover_odometry.py`: `wheel_radius=0.05` (true, 10 cm
wheels), `encoder_cpr=2737` (scripted drive: 3921 ticks over a tape-measured
0.45 m; 2737/1336 = 2.05 so gearing is ~2:1), `wheel_separation=0.4495` (mean of
two scripted 720-deg spins; matches the ~45 cm physical track).

**Why:** the old `wheel_radius=0.0257` was a fudge absorbing part of the CPR
error. Two compensating errors only agree at one speed — when the motors were
made faster, odometry broke. Keep every constant physically meaningful; if a
fudge is needed, the model is wrong.

**How to apply:** diagnose with **scripted** `/cmd_vel` motions, one axis at a
time — hand-driven captures came out mixed and gave contradictory answers. Compare
`/wheel_ticks` vs `/encoder_ticks` and read the **sum** (distance) and **diff**
(rotation), since that is what the kinematics use. A constant real/commanded
ratio means a units error, not slip. See also [[project_rplidar_a1_scan_mode]].
