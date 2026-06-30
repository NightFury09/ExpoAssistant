# ros2_ws — Directory Structure

This document outlines the actual structure of the `ros2_ws` workspace, detailing the custom ROS 2 packages and their contents.

```
ros2_ws/src/
│
├── my_robot_bringup/              ← Main orchestration package (launch, config, maps, rviz)
│   ├── package.xml
│   ├── setup.py                   ← Installs launch/, config/, maps/, rviz/, scripts/ to share/
│   ├── setup.cfg
│   │
│   ├── config/
│   │   ├── 99-rover.rules         ← udev rules for ESP32 and RPLIDAR port mappings
│   │   ├── cartographer.lua       ← Cartographer SLAM tuning parameters
│   │   ├── ekf.yaml               ← EKF configuration (fuses /odom at 20 Hz)
│   │   ├── minimal_slam.yaml      ← SLAM Toolbox parameters for lightweight testing
│   │   ├── nav2_params.yaml       ← Navigation2 configuration parameters
│   │   ├── slam_params.yaml       ← SLAM Toolbox online parameters
│   │   └── slam_toolbox_params.yaml ← Alternative SLAM Toolbox parameters
│   │
│   ├── launch/
│   │   ├── master_navigation.launch.py     ← Starts URDF + LIDAR + EKF + Nav2 + RViz
│   │   ├── teleop_launch.py                ← Starts micro-ROS agent + WASD keyboard teleop
│   │   ├── slam_teleop.launch.py           ← Starts micro-ROS agent + RPLIDAR + SLAM Toolbox + Foxglove Bridge (with ESP32 reset script)
│   │   ├── online_slam.launch.py           ← SLAM Toolbox + RPLIDAR + TF (static publisher)
│   │   ├── cartographer.launch.py          ← Cartographer + sllidar_ros2 + occupancy grid
│   │   ├── navigation.launch.py            ← Nav2 bringup with RPLIDAR + EKF against pre-built map
│   │   ├── minimal_slam.launch.py          ← Lightweight SLAM for testing
│   │   ├── odom_slam.launch.py             ← SLAM Toolbox with EKF odometry
│   │   ├── slam_online.launch.py           ← Alternate SLAM Toolbox launch
│   │   ├── rplidar_with_transform.launch.py ← LIDAR-only with static TF for testing
│   │   └── occupancy_grid.launch.py        ← Occupancy grid publisher (used by Cartographer)
│   │
│   ├── maps/
│   │   ├── my_room_map.pgm        ← Saved room occupancy grid
│   │   └── my_room_map.yaml       ← Map metadata (resolution: 0.05 m/px)
│   │
│   ├── rviz/
│   │   └── nav2_config.rviz       ← RViz2 configuration layout for Nav2 visualization
│   │
│   ├── scripts/
│   │   └── esp32_reset.py         ← Script to reset ESP32 via serial DTR/RTS (used in slam_teleop)
│   │
│   └── my_robot_bringup/
│       └── __init__.py            ← Empty Python package marker
│
├── rover_core/                    ← Core logic package (teleop, odometry, diagnostics)
│   ├── package.xml
│   ├── setup.py                   ← Defines console scripts for rover_teleop & rover_odometry
│   ├── setup.cfg
│   │
│   ├── rover_core/
│   │   ├── __init__.py
│   │   ├── rover_odometry.py      ← Computes kinematics and publishes /odom, /joint_states, TF
│   │   ├── rover_teleop.py        ← Interactive curses-based keyboard teleoperation node
│   │   └── teleop_diagnostic.py   ← Diagnostic script monitoring /cmd_vel and /wheel_ticks
│   │
│   └── test/
│       ├── test_copyright.py
│       ├── test_flake8.py
│       └── test_pep257.py
│
├── rover_description/             ← Robot physical description package
│   ├── package.xml
│   ├── setup.py                   ← Installs urdf/ to share/
│   ├── setup.cfg
│   │
│   └── urdf/
│       └── rover.urdf             ← URDF file specifying footprint, base link, wheels, and laser mounting
│
├── rplidar_ros/                   ← Slamtec RPLIDAR driver (3rd-party, source-built)
│
├── sllidar_ros2/                  ← Alternate Slamtec LIDAR driver (used by Cartographer)
│
├── build/                         ← colcon build output directory
├── install/                       ← colcon install output directory (contains sourced environment)
├── log/                           ← colcon build logs
└── ros2.repos                     ← VCS import file
```

## Key File Relationships

```
                      slam_teleop.launch.py (or master_navigation.launch.py)
                      ┌──────────────┼──────────────┬───────────────┐
                      │              │              │               │
                      v              v              v               v
                rover.urdf        ekf.yaml     slam_params.yaml  rplidar_node
              (via robot_state    (used by      (SLAM Toolbox)     (/dev/ttyLIDAR)
                 publisher)       ekf_node)         │               │
                      │              │              │               │
                      v              v              v               v
                /tf (static)   /odometry/filtered /map            /scan
                               (fuses /odom)
```
