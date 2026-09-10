---
name: project_usb_eio_bad_link
description: On this rover, [Errno 5] EIO when opening a USB serial port means a bad USB data link — not a software bug
metadata:
  type: project
---

`[Errno 5] Input/output error` when opening `/dev/ttyESP32` or `/dev/ttyLIDAR`
means the **USB data path is failing at the kernel level**. The device still
enumerates in `lsusb` and the symlink still exists, and on the lidar the LED
lights and the motor can spin — because **power and data are separate
conductors**. Power working proves nothing about data.

Seen on both the ESP32 and the RPLIDAR on this rig; it is the most common
hardware failure here.

**Root cause (found 2026-09-07):** USB power starvation from bad topology —
an EXTERNAL bus-powered hub (Huasheng 214b:7250) added under the Orin's onboard
hub, with a Logitech C270 webcam (500 mA) and a
RealSense D455 (496 mA) sharing the same 500 mA USB-2 port as both CP210x
devices. A bus-powered hub supplies only ~100 mA per downstream port. Fixed by
removing the external hub and the webcam.

**Do not misread the topology:** the Realtek pair `0bda:5420` / `0bda:0420` in
`lsusb -t` is the **AGX Orin devkit's onboard hub** — every USB-A port goes
through it and it is self-powered (`bMaxPower=0mA`), so it is never the problem.
Tell hubs apart by `bMaxPower`: 0 mA = self-powered and fine; non-zero =
bus-powered and the likely culprit.

**Why:** ROS-level symptoms (node crash loops, missing topics) look like driver
or config bugs, so it is easy to waste hours debugging launch files when the
fault is below ROS entirely.

**How to apply:** Before debugging any ROS lidar/ESP32 issue, run the port-open
test:
`timeout 4 python3 -c "import serial; s=serial.Serial('/dev/ttyLIDAR',115200,timeout=1); print('PORT OK'); s.close()"`
`Errno 5` → fix the cable/port first; nothing in software will help. `PORT OK`
→ the fault is configuration, e.g. [[project_rplidar_a1_scan_mode]].
