---
name: project_rplidar_a1_scan_mode
description: Rover's lidar is an RPLIDAR A1M8 — use Express (not Standard); Sensitivity causes error 80008004
metadata:
  type: project
---

The rover's lidar is an **RPLIDAR A1M8** (raw GET_INFO reports `model=24` /
0x18, fw 1.29, hw 7). It supports **only `Standard` and `Express`** scan modes.
`Sensitivity` and `Boost` are A2/A3/S1-only.

Every launch file in `ros2_ws/src/my_robot_bringup/launch/` originally specified
`scan_mode: 'Sensitivity'`, which made `rplidar_node` crash-loop with
`Error, unexpected error, code: 80008004`. Corrected on 2026-09-07, first to
`'Standard'` (to prove the hardware worked), then to **`'Express'`** for mapping.

**Always use `Express`.** Measured A/B on one stationary scene: Standard gives
2 kHz / 1.0 deg / 35.8% valid returns and **0 of 360 bins reliable**; Express
gives 4 kHz / 0.5 deg / 66.1% valid and **383 of 720 bins reliable** — 3.7x more
points per scan. SLAM cannot scan-match on Standard, and the resulting maps are
noisy starbursts with no straight walls.

**Why:** `0x80008004` is `RESULT_OPERATION_NOT_SUPPORT`, not a timeout
(a timeout is `0x80008002`) — it is a configuration error, not a hardware fault,
and it is easy to misread as a broken lidar or cable.

**How to apply:** If `/scan` is missing, first check whether the port opens at
all (`Errno 5` = bad USB link, a separate fault — see
[[project_usb_eio_bad_link]]), then run the raw health query documented in
`rover_project/TROUBLESHOOTING.md`. Health GOOD + node still failing means the
scan mode is wrong, not the hardware.
