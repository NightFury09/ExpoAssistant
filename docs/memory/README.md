# Distilled findings

Hard-won facts about this rover, kept in the repo so they survive a machine
failure and are readable by anyone (or any assistant) picking the project up.

These are **conclusions, not narrative** — each records something that was
expensive to learn and is not derivable from the code:

| File | The finding |
|---|---|
| `project_esp32_loop_starvation.md` | "Nav2 is broken" was really the ESP32 dropping commands |
| `project_odometry_real_encoders.md` | Odometry runs on real encoders; the calibration constants |
| `project_rmcs2303_stop_command.md` | The stop command is `0x0000`, not `0x0100` |
| `project_rplidar_a1_scan_mode.md` | It's an A1M8 — use Express, never Sensitivity |
| `project_usb_eio_bad_link.md` | `[Errno 5]` means a bad USB data link, not a software bug |
| `project_direction_rootcause.md` | The motors are healthy — do not buy RMCS-2301 |
| `project_parked_work.md` | Deliberately deferred work and why |
| `feedback_microros_version.md` | micro-ROS must be pinned to the `humble` branch |

`MEMORY.md` is the index.

The live copies live in the assistant's memory directory outside this repo;
these are the versioned, durable copies. If they diverge, prefer whichever is
newer and re-sync.

Detailed symptom → cause → fix tables are in [../../TROUBLESHOOTING.md](../../TROUBLESHOOTING.md).
