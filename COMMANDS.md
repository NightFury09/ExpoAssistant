# Rover — Command Reference

**Almost everything is done in the browser, not the terminal.**
The console at **http://192.168.3.224:8080** starts and stops the stacks, drives
the rover, builds and loads maps, marks demo points and sends goals. See
`CONSOLE.md`.

This file is for the rest: managing the service, diagnosing a problem, and the
few things that have no button.

> Every terminal that talks to ROS needs both of these first:
> ```bash
> source /opt/ros/humble/setup.bash && source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash
> ```
> Shortcut — add to `~/.bashrc`: `alias rs='source /opt/ros/humble/setup.bash && source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash'`

---

## 1. The service — the only thing that must be running

It starts at boot. Note the **`--user`**: without it systemctl looks for a
system-wide unit that does not exist and says "could not be found".

```bash
systemctl --user status rover-console
```
```bash
systemctl --user restart rover-console
```
```bash
systemctl --user stop rover-console
```
```bash
systemctl --user start rover-console
```

**`restart` and `stop` also stop whatever stack the console launched** — they
share its cgroup. Save your map before restarting.

Re-install or upgrade the unit:

```bash
~/AGX_Orin_Backup/rover_project/systemd/install.sh
```

Run it in a terminal instead (only when the service is stopped):

```bash
source /opt/ros/humble/setup.bash && source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash && ros2 run rover_core rover_dashboard
```

A second console for testing, on its own port **and its own node name**:

```bash
ros2 run rover_core rover_dashboard --ros-args -r __node:=rover_dashboard_test -p port:=8081
```

---

## 2. Logs — where to look when something fails

The console's own output. **Not `journalctl`** — this machine has no
`/var/log/journal`, so its journal is wiped on every reboot.

```bash
tail -f ~/AGX_Orin_Backup/rover_project/logs/console.log
```

Every stack launch writes its own file. When a stack fails to come up, the
reason is in the newest one:

```bash
ls -t ~/AGX_Orin_Backup/rover_project/logs/ | head
```
```bash
tail -40 $(ls -t ~/AGX_Orin_Backup/rover_project/logs/navigation-*.log | head -1)
```

Errors only, from the newest navigation log:

```bash
grep -E "\[ERROR\]|\[WARN\]" $(ls -t ~/AGX_Orin_Backup/rover_project/logs/navigation-*.log | head -1) | tail -20
```

---

## 3. After changing code

```bash
cd ~/AGX_Orin_Backup/rover_project/ros2_ws && colcon build --packages-select rover_core --symlink-install
```

Smoke-test on a spare port **before** restarting the service — it catches
errors that only appear at runtime, without taking the rover down:

```bash
ros2 run rover_core rover_dashboard --ros-args -r __node:=smoke -p port:=8089
```

Then:

```bash
systemctl --user restart rover-console
```

The browser reloads itself when the server reports a new build, so no manual
refresh is needed.

---

## 4. Is the rover healthy?

One line for everything:

```bash
curl -s localhost:8080/api/metrics | python3 -m json.tool | head -40
```

The assistant-facing summary — small, and the one to poll:

```bash
curl -s localhost:8080/api/places | python3 -m json.tool
```

**The drivetrain check. Reach for this first whenever the rover moves oddly.**
`x` = /cmd_vel messages received, `y` = watchdog stops, `z` = commanded RPM.
If `x` is not climbing while you drive, commands are not reaching the ESP32 and
no amount of Nav2 tuning will help:

```bash
ros2 topic echo /rover_diag
```

Devices present?

```bash
ls -l /dev/ttyESP32 /dev/ttyLIDAR && lsusb | grep -i 8086
```

**Camera on USB 2 or USB 3?** 5000 = USB 3, 480 = USB 2 and about a quarter of
the frame rate:

```bash
for d in /sys/bus/usb/devices/*/; do [ "$(cat $d/idVendor 2>/dev/null)" = "8086" ] && echo "D455 link: $(cat $d/speed) Mbit/s"; done
```

---

## 5. Diagnostics with no button

All under `tools/`, all read-only unless stated.

```bash
cd ~/AGX_Orin_Backup/rover_project
```

| command | what it answers |
|---|---|
| `python3 tools/check_lidar.py` | Is the lidar data any good? Run **stationary**. Shows per-sector ranges — how you find self-hits. |
| `python3 tools/check_localization.py` | Does the live scan match the loaded map? Run after setting the pose. |
| `python3 tools/check_reachable.py X Y` | Can the planner reach this point — **without moving the rover**. |
| `python3 tools/find_goal.py` | Suggests goals the planner can actually reach. Reads the costmap, not the map. |
| `python3 tools/check_camera_mount.py` | Fits the floor to check the D455 mount. **Needs clear floor** — see the warning in `CONSOLE.md`; it fits desks and chair seats in a cluttered room and gives a different answer every run. |

---

## 6. Driving it from another program (Product_RAG_)

```bash
python3 tools/rover_client.py places
```
```bash
python3 tools/rover_client.py ready
```
```bash
python3 tools/rover_client.py go robotic_arm --wait
```
```bash
python3 tools/rover_client.py cancel
```
```bash
python3 tools/rover_client.py estop on
```

`ready` exits 0 when it is safe to offer somebody a walk, non-zero otherwise
with the reason. In Python: `from rover_client import Rover`.

---

## 7. Emergencies

**Stop the rover now** — halts the motors and cancels the goal:

```bash
curl -s -X POST localhost:8080/api/estop -d '{"on":true}'
```
```bash
curl -s -X POST localhost:8080/api/estop -d '{"on":false}'
```

Cancel just the goal, leave the rover live:

```bash
curl -s -X POST localhost:8080/api/nav_cancel -d '{}'
```

Stop everything, cleanly:

```bash
systemctl --user stop rover-console
```

Something is holding port 8080 and you do not know what:

```bash
ss -lptnH 'sport = :8080'
```

**Never `kill -9` the RealSense node** — it wedges the V4L2 device until the
camera is physically unplugged. The console's own shutdown escalates slowly for
exactly this reason.

---

## 8. Things that are NOT commands any more

Launching a stack by hand fights the console, which refuses to start a second
one and reports `EXTERNAL`. Use the MAP tab. If you must, stop the console
first.

<details>
<summary>The old by-hand bringup, for reference only</summary>

```bash
ros2 launch my_robot_bringup slam_teleop.launch.py
```
```bash
ros2 launch my_robot_bringup master_navigation.launch.py map:=/home/rptech/AGX_Orin_Backup/rover_project/maps/room_map_v6.yaml
```
```bash
ros2 launch my_robot_bringup realsense.launch.py
```

Both stacks take `use_dashboard`, which must stay **false**: the console
already owns :8080 and a second copy cannot bind it.
</details>

---

## 9. Git

```bash
cd ~/AGX_Orin_Backup/rover_project && git status --short
```
```bash
cd ~/AGX_Orin_Backup/rover_project && git log --oneline -10
```

The remote is SSH (`git@github.com:NightFury09/ExpoAssistant.git`). Maps and
launch logs are gitignored; `config/demo_waypoints.yaml` is **not** — the demo
points are the deliverable.
