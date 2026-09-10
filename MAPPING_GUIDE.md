# Building a Map — Step by Step

Companion to [COMMANDS.md](COMMANDS.md) and [TROUBLESHOOTING.md](TROUBLESHOOTING.md).
This is the *skill* of mapping — the driving technique, not just the commands.

## The core concept

SLAM builds the map by matching each new lidar scan against what it already
knows. **If a new scan doesn't overlap enough with the known map, the match
fails** — the map stops growing, and the robot's displayed position (now
running on odometry alone) can drift outside the mapped area. Every rule
below exists to prevent that one failure mode.

---

## Step 1 — Launch the SLAM stack
```bash
source /opt/ros/humble/setup.bash && source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash && ros2 launch my_robot_bringup slam_teleop.launch.py
```
Brings up, in order: ESP32 reset → micro-ROS agent (~3s) → odometry → RPLIDAR
(~5s) → SLAM toolbox (~10s delay, so odometry/TF exist first) → Foxglove
bridge. Wait ~15s.

**If `/wheel_ticks` doesn't start flowing within ~10s, press the ESP32's
physical EN/RESET button once** — auto-reset is unreliable on this board.

## Step 2 — Set up Foxglove
Connect to `ws://192.168.3.224:8765`. 3D panel: **Fixed frame = `map`**,
enable `/map` and `/scan`. **Turn OFF `/slam_toolbox/graph_visualization`**
(the pose-graph overlay) — it traces your driven path, not the walls, and is
easy to mistake for the room boundary.

## Step 3 — Drive to build the map
Bring teleop speed **down**, then:

1. **Move slowly, in short bursts.** Pause, let the gray area catch up in
   Foxglove before continuing.
2. **Always keep a wall or distinctive object in the lidar's view.** Don't
   drive into open space with nothing nearby to match against.
3. **Turn gently** — a fast spin can outrun scan-matching just like driving
   too fast can.
4. **Trace the perimeter first** (walls = strongest features), fill the
   middle after.
5. **Loop back near your start point periodically** — closes the loop,
   correcting drift across the *whole* map, not just the recent bit.

## Step 4 — Recognize healthy vs. broken, live
- **Healthy:** robot icon stays *inside* the gray area, which visibly grows
  as you move.
- **Broken:** robot icon floats into black (unmapped) space, gray stops
  growing even though you're still driving. SLAM lost tracking.

If it breaks: drive slowly back into the known gray area — it usually
re-acquires. If not, reset SLAM (Step 5) and be more careful on the retry.

## Step 5 — Reset the map (start over without restarting everything)
```bash
pkill -f async_slam_toolbox_node
```
then, in a fresh terminal (lidar/odometry/agent keep running):
```bash
source /opt/ros/humble/setup.bash && source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash && ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file ~/AGX_Orin_Backup/rover_project/ros2_ws/install/my_robot_bringup/share/my_robot_bringup/config/slam_params.yaml -p use_sim_time:=false
```
If the whole launch is already down, don't restart SLAM alone (nothing feeds
it) — just re-run the full launch (Step 1). That *is* the reset.

## Step 6 — Save the map (the #1 rule people forget)
**The map only exists inside the running `slam_toolbox` node.** The moment
you Ctrl+C the launch, `/map` disappears. Save it *before* that, from a
separate terminal, while everything is still running:
```bash
source /opt/ros/humble/setup.bash && source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f ~/AGX_Orin_Backup/rover_project/maps/<name>
```
Use a real name (e.g. `expo_floor_map`) so you don't overwrite other maps.
Writes `<name>.pgm` (image) + `<name>.yaml` (metadata). You can re-save at
any checkpoint during a long session — each save overwrites that path.

If the save errors with "Failed to spin map subscription", you ran it either
before SLAM finished starting or after the launch was already stopped.

## Step 7 — Only then shut down
Ctrl+C the launch terminal — **after** confirming the save (check
`ls -la ~/AGX_Orin_Backup/rover_project/maps/`).

---

## Mapping the expo floor specifically (bigger, harder than a room)

- **Range matters more.** RPLIDAR's usable range is ~12 m. In a large open
  hall, far walls may be out of range, leaving nothing to match against in
  open floor. **Use booths, pillars, and structures as anchor features** —
  plan a driving path that always keeps *something* within range.
- **Plan the path like overlapping paint passes**, not one loop — bigger
  area needs more passes and more time.
- **Map before the crowd if you can.** People walking through the beam show
  up as noise; SLAM tolerates some, but a packed floor is much harder. Early
  setup time is ideal.
- **Watch for glass/mirrored booth walls** — can give bad or missing lidar
  returns.
- **Save checkpoints periodically** during a long mapping run (Step 6, any
  time) — cheap insurance against having to redo the whole floor if
  something interrupts the session.
