# Next Steps / Parked Work

Companion to [COMMANDS.md](COMMANDS.md), [TROUBLESHOOTING.md](TROUBLESHOOTING.md),
[MAPPING_GUIDE.md](MAPPING_GUIDE.md).

---

## PINNED — Unlock the D455 IMU to kill rotational drift

**Parked 2026-09-07.** Deliberately deferred: the lidar SLAM path was one lap from
validation, and a kernel build risks a working system. Revisit only after the
mapping pipeline is signed off.

### Why it is worth doing

Residual heading drift is what ghosts walls at corners — the same wall laid down
twice from two poses. Wheel odometry infers heading from a tick *difference*, so
it inherits every scale error and any slip. A gyro measures rotation directly.
Fused through the EKF already configured in `config/ekf.yaml`, it would largely
remove that error class.

The D455 already contains a usable IMU. Nothing needs buying.

### Why it does not work today

librealsense is built with the **RSUSB userspace backend** (chosen to avoid
patching the kernel). RSUSB cannot reach the camera's HID interface, so gyro and
accel are unavailable — hence `enable_gyro: False` / `enable_accel: False` in
`launch/realsense.launch.py`.

### What the job involves

1. Build and install the patched `uvcvideo` / HID kernel modules for L4T R36.4.7
   (JetPack 6) using librealsense's `patch-realsense-ubuntu-L4T.sh` equivalent for
   this kernel.
2. Rebuild librealsense **without** `-DFORCE_RSUSB_BACKEND=ON`.
3. Set `enable_gyro: True`, `enable_accel: True`, `unite_imu_method: 2` (linear
   interpolation) in `realsense.launch.py`.
4. Add `/camera/camera/imu` as an `imu0` source in `ekf.yaml`, fusing **yaw rate
   only** (`vyaw`) — not absolute orientation, which drifts without a magnetometer.

### Blockers to solve FIRST — do not skip these

- **The camera is enumerating at USB 2.0** (480M). Bus 02 on the AGX is empty and
  the SuperSpeed lanes never come up, though the same cable does USB 3 on an Orin
  Nano. Test with a USB 3 flash drive in the same port: if nothing appears under
  Bus 02, the AGX USB-A SuperSpeed path is the fault — use the USB-C ports, which
  bypass the onboard Realtek hub entirely.
- **Rigid mounting.** The D455 currently sits on a tripod on top of a screen on a
  mast. An IMU that flexes relative to the chassis reports rotation the wheels
  never made, which is worse than no IMU. It must be bolted to the frame.

### Cheaper alternative if the kernel build stalls

A **BNO055** (on-chip fusion, absolute heading) or **MPU6050** (cheaper, needs
filtering) wired to the ESP32 over I2C, published as a micro-ROS topic. More
robust than the camera route because it mounts rigidly to the chassis and does
not depend on the camera being connected. Prefer this if the kernel work fights
back.

---

## Other parked items

- **Raise the lidar ~5 cm on standoffs** so the beam clears the purple corner
  bumpers. Then `min_laser_range` in `config/slam_params.yaml` can drop from 0.55
  back to ~0.20, restoring close-range obstacle detection for Nav2. Currently the
  rover is blind inside 55 cm while its own footprint radius is only ~31 cm.
- **Firmware never writes the drive tuning registers.** It only writes REG_SPEED
  (14) and REG_CONTROL (2), so each RMCS-2303 runs on whatever is in its own
  EEPROM. If the two units hold different `TRP_ACL_WORD` (addr 12, acceleration)
  values they ramp unevenly after every stop, which would explain the rover
  arcing after a 90 deg turn. Fix: write matching acceleration (and optionally
  `LINES_PER_ROT` = 334, addr 10) to BOTH drives at startup. Read both back first
  to confirm they actually differ before changing anything.
- **Camera mount TF is still a placeholder** in `launch/realsense.launch.py`
  (x=0.20, z=0.20). Measure it properly before trusting the D455 pointcloud in
  the Nav2 costmap.
