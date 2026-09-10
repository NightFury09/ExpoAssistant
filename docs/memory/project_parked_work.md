---
name: project_parked_work
description: Rover work deliberately parked 2026-09-07 — D455 IMU kernel build, lidar riser, drive tuning registers; see NEXT_STEPS.md
metadata:
  type: project
---

Parked on 2026-09-07 to avoid destabilising a nearly-working SLAM pipeline before
the expo. Full detail in `rover_project/NEXT_STEPS.md`.

1. **Unlock the D455 IMU** to cut rotational drift. librealsense is built with the
   RSUSB userspace backend, which cannot reach the camera's HID interface, so
   gyro/accel are off. Needs patched kernel modules for L4T R36.4.7 and a
   librealsense rebuild without `-DFORCE_RSUSB_BACKEND=ON`, then fuse **yaw rate
   only** into `ekf.yaml`. Two blockers must be cleared first: the camera is stuck
   at USB 2.0 on the AGX, and it is tripod-mounted on a screen mast rather than
   rigidly on the chassis. A BNO055/MPU6050 on the ESP32 over I2C is the more
   robust fallback.
2. **Raise the lidar ~5 cm** so it clears the corner bumpers, then drop
   `min_laser_range` from 0.55 back to ~0.20.
3. **Firmware never writes the RMCS-2303 tuning registers** (only REG_SPEED 14 and
   REG_CONTROL 2), so each drive uses its own EEPROM values. Mismatched
   `TRP_ACL_WORD` (addr 12) would explain the rover arcing after a 90 deg turn.
   Read both drives back before changing anything.

**Why:** each of these is a real improvement, but none blocks mapping, and the
kernel build in particular risks a working system for an unmeasured gain.

**How to apply:** finish and validate lidar SLAM first. Revisit in the order
above. See [[project_odometry_real_encoders]] for the calibration those depend on.
