# Foxglove layouts

Import instead of rebuilding the panel setup every time Foxglove resets.

**Import:** Foxglove → layout dropdown (top bar) → *Import from file…* → pick the
JSON. Then connect to `ws://192.168.3.224:8765`.

| File | For |
|---|---|
| `rover_navigation.json` | driving Nav2: map, scan, plan, particle cloud, plus `/rover_diag` |

## What it presets

* **Fixed frame `map`** — the usual first mistake is leaving it on `odom`, where
  the robot sits still and the world slides past.
* **Publish → `/goal_pose`.** Foxglove defaults the 2D-pose tool to
  `/move_base_simple/goal`, the **ROS 1** name. Nav2 does not listen there, so
  clicking a goal silently does nothing. This is the single most common reason
  "Nav2 ignores my goals".
* Pose estimate → `/initialpose`, theta deviation 15°.
* `/slam_toolbox/graph_visualization` **off** — it traces the driven path, not
  walls, and is constantly mistaken for the room outline.
* Costmaps and the depth pointcloud are present but hidden; switch them on when
  debugging obstacle behaviour rather than leaving them cluttering the view.

## If the topic list looks empty

`/goal_pose` and `/initialpose` will not necessarily appear in the topic list —
that list shows what Foxglove is *receiving*, and those are topics Nav2
subscribes to. **You do not need them listed to publish to them**; the publish
tool takes any topic name.

If nothing at all appears, the connection is down rather than the topics being
missing. Check from the Jetson:

```bash
ros2 node list | grep foxglove_bridge
ss -ltn | grep 8765
```

## WebGL errors in the 3D panel

`Error creating WebGL context` is your browser's GPU, not the rover. Enable
hardware acceleration (`chrome://settings/system`), close other 3D tabs
(browsers cap WebGL contexts around 16), or use the **Foxglove desktop app**,
which manages its own context and is far more reliable for this.
