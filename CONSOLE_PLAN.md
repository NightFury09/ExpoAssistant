# Rover Console — plan

> **Status (2026-09-11): phases 1-4 are built.** See `CONSOLE.md` for how to
> use it. Phase 5 (the Product_RAG_ hook) is partly done -- the HTTP API it
> needs already exists; what is left is the assistant side. This file is kept
> as the design record.

A single web UI that replaces the terminal for day to day operation: build maps,
choose maps, place and name demo stations, set the base, drive, send goals, and
watch what the rover is doing. Terminal only for development.

Verdict: **all of it is feasible.** The ROS side already exposes everything
needed, and the existing dashboard already does the hard parts of live video,
metrics and manual driving.

---

## The one architectural change

The dashboard currently runs **inside** `master_navigation.launch.py`. To start
and stop SLAM vs Navigation it has to run **outside** them, as a supervisor:

```
rover_console                       always running, one command or a systemd unit
├── web UI on :8080
├── ROS node   live topics in, /cmd_vel and /goal_pose out
└── supervisor starts and stops:
      ├── slam_teleop.launch.py         mapping mode
      ├── master_navigation.launch.py   navigation mode, map passed as an argument
      └── realsense.launch.py           camera, either mode
```

This also fixes a problem that has bitten repeatedly: **two stacks running at
once.** With one supervisor owning mode changes, that becomes impossible by
construction rather than by remembering.

---

## Features and how each is done

| Feature | Mechanism | Risk |
|---|---|---|
| Live map view | render `/map` `OccupancyGrid` to canvas | low |
| Robot pose, scan, plan overlay | `/tf`, `/scan`, `/plan` | low |
| Click to send a goal | canvas pixel to map metres, publish `/goal_pose` | low |
| Click to set initial pose | same, publish `/initialpose` | low |
| Place and name demo stations | write `demo_waypoints.yaml` | low |
| Set base / home | a waypoint with the reserved id `home` | low |
| Go to a station | `NavigateToPose` action, report status | low |
| Choose a map | `nav2_msgs/srv/LoadMap` on `/map_server/load_map` | medium |
| Enter SLAM mode | supervisor starts `slam_teleop.launch.py` | **high** |
| Save a new map | `slam_toolbox/srv/SaveMap`, then it appears in the picker | medium |
| Return to base on failure | goal ABORTED then navigate to `home` | low |

All four services were verified present on this machine:
`nav2_msgs/srv/LoadMap`, `nav2_msgs/srv/SaveMap`,
`slam_toolbox/srv/SaveMap`, `slam_toolbox/srv/SerializePoseGraph`.

---

## What is genuinely hard

**Process supervision.** Starting and stopping ROS launches cleanly is the part
most likely to cause trouble, and this project has already lost hours to
orphaned nodes: duplicate `rover_odometry` publishing TF, a stale `slam_toolbox`
fighting AMCL, a wedged RealSense after a hard kill. The supervisor must:

* launch each stack in its **own process group** and signal the group, not the
  process, so no child is orphaned;
* stop with SIGINT and wait, escalating only after a timeout, because
  `kill -9` is what wedges the RealSense;
* verify the previous stack is **fully gone** before starting the next, by node
  name and not just by PID;
* refuse to start a second stack while one is running.

**Surviving stack restarts.** The console keeps running while the ROS graph
underneath it disappears and returns. Subscriptions must tolerate that and
re-establish rather than silently going dead.

**Coordinate transforms.** Pixel to map maths must be exact or every click lands
somewhere else. The map's `origin` and `resolution` come from the
`OccupancyGrid`, and the y axis is flipped between image and map frames.

Everything else is ordinary web work.

---

## Build order

Each phase is usable on its own.

**1. Map view.** Canvas render of `/map` with robot pose, live scan and plan
overlaid, pan and zoom. This is the foundation every later phase draws on, and
on its own it already replaces Foxglove for normal use.

**2. Click to act.** Click to send a goal, shift-click to set the initial pose.
Needs only phase 1's coordinate maths.

**3. Waypoints.** Place, name, list, delete, go to. Set base. Saves to
`demo_waypoints.yaml`, which `save_waypoint.py` already writes, so the format is
settled.

**4. Mode and map control.** IDLE / MAPPING / NAVIGATION buttons, map picker,
save map. This is the phase carrying the process supervision risk, which is why
it comes after the useful parts rather than before.

**5. Product_RAG_ hook.** `POST /api/demo/goto {"id": "thermal_camera"}` and
`GET /api/demo/status`, so the assistant can drive the rover. The console is
already an HTTP server, so this is a small addition once phase 3 exists.

---

## Open questions

1. **Should the console autostart on boot?** A systemd unit means the rover is
   usable from a browser the moment it powers on, with no SSH at all. That is
   the right answer for an expo, but it is one more thing that can fail quietly.

2. **Should MAPPING mode include teleop?** It must, since mapping requires
   driving. The console already has manual control, so mapping mode just needs
   the drive panel enabled and Nav2 absent.

3. **What happens to a goal when the mode changes?** Simplest and safest: mode
   changes cancel any active goal and stop the rover.
