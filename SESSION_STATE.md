# Where the project stands

Updated 2026-09-10. Read this first when resuming — it is the handover note.
Detail lives in [TROUBLESHOOTING.md](TROUBLESHOOTING.md),
[DASHBOARD.md](DASHBOARD.md), [MAPPING_GUIDE.md](MAPPING_GUIDE.md) and
[NEXT_STEPS.md](NEXT_STEPS.md).

---

## Working right now

| | State |
|---|---|
| Teleop | works, via keyboard node or the web dashboard |
| Odometry | real encoders; 0.6 m drives repeat to ±0.6%, closed square closes to 5.9% |
| Lidar | RPLIDAR A1M8, **Express** mode, ~7.5 Hz, 383/720 bins reliable |
| SLAM mapping | works — `room_map_v6` has clean single-line walls |
| RealSense D455 | USB 3.0, mount calibrated, 4 dashboard views |
| Web dashboard | `http://192.168.3.224:8080` — drive + camera + metrics |
| Nav2 | plans and drives; **path following just changed, UNTESTED** |

## The one thing that is untested

The controller was switched from DWB to **RotationShim + RegulatedPurePursuit**
at 10 Hz to fix the rover snaking down straight paths. **This has not been
driven yet.** That is the first thing to test on resuming.

Cause of the snaking: odometry is capped at ~3.9 Hz (Modbus to the RMCS-2303 is
fixed at 9600 baud, two encoder reads cost 70–200 ms) while the controller ran
at 20 Hz. Four of every five DWB cycles scored trajectories against an unchanged
pose, and `PathAlign.scale: 32` snapped hard toward the path — over-correct, see
it 250 ms late, over-correct back.

If it still snakes, the next levers in order: raise `lookahead_dist` (0.6 → 0.8),
lower `desired_linear_vel`, then lower `controller_frequency` to 5 Hz.
Previous config is in git history and at
`scratchpad/nav2_params.before_rpp` if that session's scratch survives.

---

## The lesson that cost the most time

**Autonomy symptoms are usually not autonomy bugs.** Hours went into AMCL,
costmap and DWB tuning while the real fault was the ESP32 silently dropping
`/cmd_vel` — blocking Modbus starved the micro-ROS transport and the session was
torn down every ~12 s.

So, before touching any autonomy parameter:

```bash
ros2 topic echo /rover_diag     # x = cmd_vel received, y = watchdog fires
```

If `x` is not climbing while you drive, commands are not arriving and nothing
above it can work. The dashboard shows the same counters.

Second-order lesson: **run one stack at a time.** `slam_teleop`,
`master_navigation` and `teleop_dashboard` each start their own micro-ROS agent,
`rover_odometry` and `rplidar`. Two at once gives two publishers on
`odom→base_footprint` and everything downstream behaves erratically.

```bash
ros2 node list | sort        # before launching anything
```

---

## Calibration constants — measured, do not guess

| Where | Value | How it was obtained |
|---|---|---|
| `rover_odometry` `wheel_radius` | 0.05 | measured wheel, 10 cm diameter |
| `rover_odometry` `encoder_cpr` | 2737 | 3921 ticks over a tape-measured 0.45 m |
| `rover_odometry` `wheel_separation` | 0.4495 | mean of two scripted 720° spins |
| firmware `GEAR_RATIO` | 2.049 | 2737 / 1336 steps-per-motor-rev |
| URDF `laser_frame` | `-0.155 0 0.275`, yaw 180° | rear-mounted, verified by object-in-front test |
| D455 mount | x 0.24, z 0.850, pitch 0.166 rad | floor-plane fit, 8 mm residual |

---

## Next: demo navigation

Goal: named demo stations on the map; the rover drives to one on request,
avoiding obstacles, and later this is driven by Product_RAG_ — a client asks
about a demo, the assistant offers to show it, the rover goes there and the
interaction resumes on arrival.

```
Product_RAG_  --HTTP-->  demo_navigator  --action-->  Nav2
 "show me the             name -> pose,               plan / avoid /
  thermal camera"         status, cancel              re-route
        <---- arrived -----+
```

| # | Piece | State |
|---|---|---|
| 1 | `demo_waypoints.yaml` + `tools/save_waypoint.py` | **built** |
| 2 | `demo_navigator` node (name → NavigateToPose, status, cancel) | not started |
| 3 | Dashboard panel: list stations, Go, live status | not started |
| 4 | HTTP API for Product_RAG_ | not started |

### Open decisions, needed before building piece 2

1. **Where does Product_RAG_ run?** Same Jetson (HTTP to localhost is trivial),
   the Nano, or elsewhere — this decides how the API is bound.
2. **What happens on arrival and after?** Park and wait for a "return" command,
   auto-return to a home station after N minutes, or stay until the next
   request. This shapes the state machine and is awkward to retrofit.

### Capturing stations

Poses live in the **map frame**, so they belong to the map loaded when captured.
Re-survey the space and they must be re-captured.

```bash
python3 tools/save_waypoint.py thermal_camera --label "Thermal Camera" \
    --say "This is our thermal imaging demo."
```

Drive the rover there and **point it the way it should face a visitor** — the
heading is captured too, and on arrival it should face the person, not the wall.

---

## Still parked

See [NEXT_STEPS.md](NEXT_STEPS.md): the D455 IMU (needs a kernel build and a
rigid mount), raising the lidar ~5 cm, and writing matching tuning registers to
both RMCS-2303 drives.
