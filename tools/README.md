# tools/

Diagnostics that answer a question with a **number** instead of an opinion.
Each was written to settle a specific problem that cost real time. Every one has
`--help`.

All of them need ROS sourced:

```bash
source /opt/ros/humble/setup.bash
source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash
```

| Script | Answers |
|---|---|
| `check_lidar.py` | Is the lidar data good enough to map with? |
| `check_camera_mount.py` | Is the D455 mount transform right? |
| `check_localization.py` | Is the robot actually localised, or is the pose wrong? |
| `set_initial_pose.py` | Tell AMCL where the robot is |
| `find_goal.py` | Where can I safely send the robot? |
| `send_goal.py` | Send it there |
| `save_waypoint.py` | Record the current pose as a named demo station |

---

## The Nav2 sequence

```bash
ros2 launch my_robot_bringup master_navigation.launch.py     # terminal 1
```

Then, in another terminal:

```bash
python3 tools/set_initial_pose.py 0 0 0     # roughly where the robot is
python3 tools/check_localization.py         # expect 40-70% at first
# drive around with teleop - AMCL converges on MOTION, never while parked
# STOP teleop before continuing
python3 tools/check_localization.py         # want >70%
python3 tools/find_goal.py
python3 tools/send_goal.py 1.68 0.02
```

`send_goal.py --cancel` halts the robot by targeting its current position.

---

## Why each exists

**`check_lidar.py`** — the average valid-return rate hid a fatal problem. The
lidar averaged 36% valid returns and looked merely noisy, but **zero of 360 bins
returned reliably**, so every wall flickered and scan matching could never
converge. Read *per-bin reliability*, not the average. It also flags sectors
where the robot is seeing its own chassis — a phantom obstacle that travels with
the robot and smears the map at every turn.

**`check_camera_mount.py`** — uses the floor as ground truth. Transforms the
depth cloud into the robot frame and checks where the floor lands: it should sit
at z ≈ 0 and stay flat as range increases. A **height offset** means `cam_z` is
wrong; a floor that **drifts with range** means `cam_pitch` is wrong, and that is
the dangerous one — a nose-down camera declared level makes the floor project
upward, so Nav2 sees a wall rising ahead and refuses to move.

Mount geometry: level at 0.82 m with a 58° vertical FOV, the camera **cannot see
the floor closer than ~1.5 m** (`0.82 / tan(29°)`). Missing floor samples nearer
than that are correct, not a fault. It also means low objects near the base are
invisible to the camera — the lidar remains the near-field sensor.

**`check_localization.py`** — almost every "Nav2 is broken" report is a bad
initial pose. Eyeballing scan-vs-map alignment in Foxglove is subjective; this
scores it. Under 40% means re-set the pose; a stationary robot never converges,
so if the number will not climb while driving, the guess is simply wrong.

**`set_initial_pose.py`** — until this runs, the `map` frame does not exist and
Nav2 floods the console with costmap `lookupTransform` errors. Those are
downstream noise, not separate faults, and they all stop at once.

**`find_goal.py`** — reads `/global_costmap/costmap`, **not** `/map`. That
distinction is the whole point: a cell can be perfectly free on the map and still
be unreachable, because the costmap inflates every obstacle by `inflation_radius`
and the robot's *centre* may not enter the inscribed band. The first version of
this script read `/map` and recommended goals with costmap cost 99 — the planner
could never reach them, and Nav2 just span in recovery behaviours, which looks
exactly like "Nav2 is broken".

It also reports the cost of the cell the robot is standing in. A robot sitting at
cost ≥90 is effectively wedged and needs moving before anything will plan.

Costmap values (Nav2 publishes `OccupancyGrid` scaled **0–100**, not the internal
0–255 — getting this wrong makes walls look like empty space):

| Value | Meaning |
|---|---|
| 0 | free |
| 1–89 | inflation — passable, discouraged |
| 90–99 | **inscribed** — robot centre cannot be here |
| 100 | lethal |
| −1 | unknown |

**`save_waypoint.py`** — map coordinates are meaningless by eye. The only
reliable way to define a demo station is to drive the rover there, point it the
way it should face a visitor, and capture `map → base_footprint`. That records
the **heading** too, which matters: on arrival the rover should face the person,
not the wall.

```bash
python3 save_waypoint.py thermal_camera --label "Thermal Camera" \
    --say "This is our thermal imaging demo."
python3 save_waypoint.py --list
```

Writes to `ros2_ws/src/my_robot_bringup/config/demo_waypoints.yaml`. Poses are in
the **map frame**, so they belong to the map that was loaded when you captured
them — re-survey the space and they must be re-captured.

**`send_goal.py`** — publishes to `/goal_pose`, and reminds you of the two
preconditions that catch people: localisation above ~70%, and **no teleop
running**. `rover_teleop_v2` publishes zeros on a timer and will fight the
controller, so the robot twitches and goes nowhere. `ros2 topic info /cmd_vel`
must show exactly **one** publisher.

---

Background and the full symptom→cause→fix tables are in
[../TROUBLESHOOTING.md](../TROUBLESHOOTING.md).
Mapping technique is in [../MAPPING_GUIDE.md](../MAPPING_GUIDE.md).
