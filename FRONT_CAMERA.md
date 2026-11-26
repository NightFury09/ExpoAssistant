# Front camera and smoother driving

Two problems, one document: the rover does not reliably see **chair legs** (or anything
thin and low), and it **drives in a staggered way**. This is the plan, what is built, and
what is left to do once the Jetson is on the rover. Nothing here has run on the real
rover or the real C270 yet. It has been tested on synthetic scenes, a real Nav2 costmap
node, and the real scan node. Read [What is proven](#what-is-proven-and-what-is-not).

## Read this first: the camera spot matters more than the code

The C270 was meant to go **under the lidar**. That spot cannot see the floor ahead.

From `rover.urdf`: the deck is 43 × 44 cm and its top is at **23 cm**. The lidar plate is at
27.5 cm, so a lens under it is about 2 cm above the deck. Every ray that points down lands
on the deck. Only rays that are nearly level reach the floor, and they reach it 4 m away.

```
python3 tools/camera_mount_planner.py
```

| mount | lens | pitch | view blocked by the rover | first visible floor, past the front edge |
|---|---|---|---|---|
| under the lidar, x = −0.155 | z 0.25 | 0° | 42% | 4.3 m |
| under the lidar | z 0.25 | 20° | **92%** | 4.7 m |
| under the lidar | z 0.25 | 35° | **100%** | never |
| under the lidar, raised on a mast | z 0.45 | 25° | 36% | 0.39 m |
| under the lidar, raised on a mast | z 0.55 | 25° | 11% | 0.27 m |
| front edge, x = 0.22 | z 0.25 | 25° | 0% | 0.29 m |

So: **mount the camera at the front edge of the deck, about 25 cm up, tilted 20–25° down**,
or keep it at the rear on a mast at least 45 cm up. A chair leg 40 cm ahead of the front
edge is then in view. Under the lidar at 25 cm, it is not.

Two caveats on those numbers. The tool treats the deck as one box from the URDF and looks
straight ahead only, so measure the real deck height with a tape before drilling. And the
mast that carries the screen and the D455 stands about 22 cm from the lidar (see the comment
in `rover.urdf`), so check it is not in the picture from any rear position.

`config/front_cam.yaml` currently holds the under-the-lidar position you asked for. When
the camera comes up with that mount, `front_cam_scan` logs an error saying the camera sees
the deck. Change the numbers to wherever the camera really is.

## The pipeline

```
 C270 on /dev/video0
        │  front_cam            OpenCV capture, newest frame only      rover_core/front_cam.py
        ▼
 /front_cam/image_raw + camera_info
        │  front_cam_scan       floor-boundary vision                  rover_core/front_cam_scan.py
        ▼                       (colour of the floor in front = "floor";   rover_core/ground_vision.py
 /front_cam/scan                 first non-floor pixel in each column
 (virtual laser scan)            = where an obstacle meets the floor)
        │
        ▼
 local_costmap / front_cam_layer   its own ObstacleLayer, marks and clears   nav2_params.yaml
        │  max with the lidar layer and the D455 voxel layer
        ▼
 controller (RegulatedPurePursuit) → /cmd_vel_nav → velocity_smoother → /cmd_vel
```

**Why floor-boundary vision.** A chair leg stands on the floor. A camera that can see the
floor sees where the leg meets it, and a calibrated camera at a known height turns that
pixel into a distance. It needs no GPU, has no model to download, and took 30 to 40 ms a frame
on the Nano in the pipeline test. It is the same idea as ROS's `depthimage_to_laserscan`, with colour instead
of depth.

**Why a scan, and why its own layer.**
- A `LaserScan` goes into Nav2's existing `ObstacleLayer` with no new code.
- It is a **separate layer**, not another source on the lidar's. One shared layer would let
  the camera's "floor all the way" rays clear cells the lidar had marked. The costmap test
  proves a camera ray through a lidar mark leaves it alone.
- It is **local only**. The global costmap and the planner are unchanged.
- It **fails open**. `expected_update_rate: 0` means a dead camera never stalls the
  controller. The rover then has the lidar and the D455, as before.

### What it cannot do

It sees colour, so it is blind to anything the same colour as the floor (a white leg on white
tile), it reads a rug edge as a wall, and it is unreliable in glare or darkness. So:
- it publishes **nothing** when it does not trust the picture (covered lens, a strip in front
  that is not one colour, too little floor in view), and says why on `/front_cam/status`;
- it sits **beside** the lidar and depth camera. It never replaces them.

## Bring-up, when the Jetson is on the rover

The camera has one owner at a time. **`reception.py` currently holds `/dev/video0`**
(the visitor greeter, still running as the `reception` service). Stop it first, or the
capture node just retries and says why in its log.

1. **Mount** the camera (see above). Measure x, z and the tilt as well as you can.
2. **Edit** `ros2_ws/src/my_robot_bringup/config/front_cam.yaml`, `front_cam_scan` section:
   `x`, `y`, `z`, `pitch_deg`, `yaw_deg`.
3. **Calibrate the lens** (once per camera and resolution). Print a 9 × 6 inner-corner
   chessboard, measure a square:
   ```bash
   python3 tools/calibrate_front_cam.py capture --out ~/front_cam_calib
   python3 tools/calibrate_front_cam.py solve --images ~/front_cam_calib --square 0.025 --write
   ```
4. **Rebuild** so the new files are installed, then start it:
   ```bash
   cd ros2_ws && colcon build --packages-select rover_core my_robot_bringup && source install/setup.bash
   ros2 launch my_robot_bringup front_cam.launch.py
   ```
   (`master_navigation.launch.py use_front_cam:=true` starts the same thing with the stack.
   The console's own start button does not pass it yet, so start it by hand.)
5. **Check the tilt against a real object.** Stand a pole or bottle on the floor dead ahead,
   measure from the front edge to its near face:
   ```bash
   python3 tools/check_front_cam_mount.py --live --distance 1.20
   ```
   A 5° tilt error makes a 1.2 m object read 50% too far (try `--demo`), so this step is not
   optional. Two distances (`--measure near.png:0.8 --measure far.png:1.6`) also solve the
   lens height.
6. **Look at it.** `ros2 topic echo /front_cam/status` should say `ok: true`; the overlay is on
   `/front_cam/debug` (green = floor, red dots = where things meet it).
7. **Run the chair-leg test, rover stationary:**
   ```bash
   python3 tools/check_obstacle_visibility.py --x0 0.4 --x1 1.2
   ```
   Put a chair in the box at 0.5, 1.0 and 1.5 m. It prints which of lidar, depth camera,
   front camera and costmap saw it. That table decides what to fix next.

## The staggered driving

What the configs say, before any measurement:

| suspect | evidence in the repo | how to confirm |
|---|---|---|
| Slow odometry | the notes in `nav2_params.yaml` put it at ~4 Hz; the controller runs at 10 Hz | `drive_profile.py` rates |
| Commands reach the base as raw steps | Nav2 already runs a velocity smoother, but there was **no block for it**, so it ran on defaults (2.5 m/s²), which never limit a 0.22 m/s rover | `drive_profile.py` command step size |
| Speed floor and cost scaling | `regulated_linear_scaling_min_speed 0.10`, cost scaling on, `desired_linear_vel 0.22` | speed efficiency |
| Stop to turn | `rotate_to_heading` after 0.6 rad, shim at 0.55 rad | pivot time |
| Waiting after failures | the roam tree waits 3 s after each failed round, the booth tree 2 to 4 s | stalls |

**Changed:** a `velocity_smoother:` block in `nav2_params.yaml`: 0.5 m/s² to speed up, 0.9 m/s²
to slow down, open loop (it ignores the slow odometry), 0.5 s command timeout. It never
clips what the controller asks for, and it does not touch manual driving from the console.
This is a reasoned setting, **not a measured improvement**.

**Not changed:** the controller, its speeds, the shim thresholds, the planner. Those were
tuned in earlier sessions for reasons recorded in the file, and retuning them blind could
undo that. Measure first:

```bash
python3 tools/drive_profile.py record --seconds 40 --out before.csv   # while it drives a goal
python3 tools/drive_profile.py report before.csv
```

It listens to `/cmd_vel_nav`, `/cmd_vel` and `/odom` and says which link looks guilty. Record
the same drive before and after each change.

## The footprint

`python3 tools/footprint_report.py`. The body is 0.43 × 0.44 m with a pivot sweep of
**0.308 m**; the costmap circle is **0.33 m**, 2 cm over. For a rover that rotates in place
that circle is the common choice: one number, the same clearance in every direction. A
polygon would only let it drive straight through a narrower gap (0.43 m against 0.66 m),
which is the wrong thing to enable while chair legs are still hard to see. So the
footprint is **unchanged**. Two things to check with a tape: anything not in the URDF that
sticks out (the screen, the front camera mount), and the 2 cm margin, which is thin.

## What is proven, and what is not

| claim | how it was checked |
|---|---|
| The projection from a pixel to a floor point is consistent, with lens distortion and yaw | unit tests (round trip) |
| The scan finds a 3.5 cm leg at 0.6–1.8 m within 10% and the right bearing, on plain, tiled and wood floors | unit tests on rendered scenes |
| Tile grout does not read as obstacles; a covered lens or a noisy strip is refused | unit tests |
| The ROS scan node publishes the right scan and goes silent on a bad picture | `tools/test_front_cam_pipeline.py` (real node) |
| Nav2's costmap marks a camera hit, clears it again, and the camera cannot clear a lidar mark | `tools/test_front_cam_costmap.py` (real `nav2_costmap_2d`) |
| The camera layer is inert with no camera running | same |
| The launch file starts the nodes and the TF is right; a busy camera is handled, not fatal | run on an isolated domain with a fake device |
| Calibration recovers the focal length; a wrong tilt is detected from a known distance | unit tests |

**Not proven:** anything on a real floor or with the real C270. The renderer uses the same
projection as the code under test, so those tests show the code is internally right, not that
the floor is the colour the model expects. The velocity smoother values and the real stagger
are unmeasured. The camera was never run against the live costmap with a rover moving.

## Not built yet

- **A console button** for the front camera, and a camera line in the console status.
- **Chair detection** (a TensorRT detector marking the whole chair footprint) and a
  **learned depth model** (the [Nav2 AI depth tutorial](https://docs.nav2.org/tutorials/docs/depth_ai_integration.html)).
  Both need a model download and a memory budget on a Nano that already ran out of memory
  once. Floor-boundary vision is the cheap first step; try these if it is not enough.
- **Nav2 collision monitor.** A stop/slow zone around the body driven straight from the scan,
  independent of the costmap. It needs the stock Nav2 launch replaced so it sits between the
  smoother and the base, which is a bigger change to the drive path than should be made
  without the rover.
- **Faster odometry.** Probably the biggest smoothness lever, and a firmware and serial job.

## Tests

```bash
cd ros2_ws/src/rover_core
python3 -m pytest test/test_ground_vision.py test/test_camera_calibration.py \
    test/test_drive_metrics.py test/test_obstacle_probe.py test/test_front_cam_config.py

# real ROS nodes, on domains no rover uses
ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 python3 tools/test_front_cam_pipeline.py
ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1 python3 tools/test_front_cam_costmap.py
```

## Reference designs this follows

- [Nav2: AI depth estimation with the costmap](https://docs.nav2.org/tutorials/docs/depth_ai_integration.html): a USB camera into a `PointCloud2` and the VoxelLayer. The route to take if floor-boundary vision is not enough.
- [Nav2: semantic segmentation layer](https://docs.nav2.org/lyrical/tutorials/general_tutorials/navigation2_with_semantic_segmentation/navigation2_with_semantic_segmentation/): class-aware obstacles such as "chair".
- [Isaac ROS nvblox](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html): GPU 3D mapping into Nav2. Heavy for this Nano.
- [Spatio-temporal voxel layer](https://index.ros.org/p/spatio_temporal_voxel_layer/): voxels that fade over time, the usual answer to ghost obstacles.
- [Nav2 velocity smoother](https://docs.nav2.org/configuration/packages/configuring-velocity-smoother.html) and the [controller timing guide](https://wimblerobotics.github.io/navigation/nav2-controller-frequency-and-timing/).
- [Wimble Robotics: the voxel layer and obstacle heights](https://wimblerobotics.github.io/navigation/nav2-voxel-layer-for-3d-obstacles/).
