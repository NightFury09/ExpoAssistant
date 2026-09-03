# realsense.launch.py
#
# Brings up the Intel RealSense D455 as a ROS 2 node for the rover:
# depth + color + aligned depth + pointcloud + IMU, plus the static transform
# that mounts the camera on the robot.
#
# Run this SEPARATELY from slam_teleop.launch.py (the camera is optional and
# time-shared with the booth demo). Bring it up only when you want 3D perception.
#
# ---------------------------------------------------------------------------
# PREREQUISITES (one-time):
#   1. The camera is a shared resource. Free it from the booth demo first:
#        docker rm -f realsense_demo
#      (only one process may own the D455 at a time)
#   2. realsense2_camera is NOT yet installed on the host. Install it, e.g.:
#        sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description
#      If apt has no Jetson build, build librealsense (RSUSB backend) + realsense-ros
#      from source — the booth demo's Dockerfile.nano shows the proven build flags
#      (-DFORCE_RSUSB_BACKEND=ON, and -DBUILD_WITH_CUDA=true on the AGX).
#
# Usage:
#   source /opt/ros/humble/setup.bash
#   source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash
#   ros2 launch my_robot_bringup realsense.launch.py
# ---------------------------------------------------------------------------

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # -------------------------------------------------------------------------
    # RealSense D455 driver node.
    #
    # Modest 640x480 @ 15 fps keeps USB bandwidth and CPU/GPU load reasonable —
    # plenty for Nav2 obstacle avoidance. Raise later if you need it.
    #
    # NOTE: depth/color *profile* parameter names have changed across
    # realsense2_camera versions. The names below match the 4.5x line (Humble).
    # If the node rejects them, check the installed version's params with:
    #   ros2 param list /camera   (after it starts with defaults)
    # -------------------------------------------------------------------------
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            # --- streams ---
            'enable_depth':          True,
            'enable_color':          True,
            'align_depth.enable':    True,      # depth aligned into the color frame
            # Pointcloud filter. On Jetson (ARM) the filter is the NEON-optimized
            # build, so its params carry a '__neon_' suffix. On x86 it would be
            # 'pointcloud.enable'. Publishes /camera/camera/depth/color/points.
            'pointcloud__neon_.enable':     True,
            'pointcloud__neon_.ordered_pc': False,

            # --- resolution / rate (see NOTE above on param names) ---
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile':   '640x480x15',

            # --- IMU: DISABLED. The apt librealsense uses the RSUSB userspace
            # backend, which does not expose the D455's HID motion sensor
            # ("No HID info provided, IMU is disabled"). To get the IMU you'd
            # need a kernel-backend librealsense build (kernel module patch).
            # Not needed for depth-based navigation.
            'enable_gyro':           False,
            'enable_accel':          False,

            # --- robustness ---
            # initial_reset was causing a disconnect/re-enumerate churn on the
            # USB-2 port at startup. Leave it off now that the camera is free;
            # set True only to recover a hung device.
            'initial_reset':         False,
            'enable_sync':           True,

            # --- frames ---
            # The node publishes its own internal TF (camera_link -> optical frames).
            # We only need to connect base_link -> camera_link (below).
            'camera_name':           'camera',
            'publish_tf':            True,
            'tf_publish_rate':       0.0,       # static internal TF (0 = latched once)
        }],
    )

    # -------------------------------------------------------------------------
    # Mount transform: base_link -> camera_link
    #
    # PLACEHOLDER VALUES — measure your actual mount and edit:
    #   x = forward from base_link origin (m)
    #   y = left (+) / right (-) (m)
    #   z = height above base_link origin (m)
    #   roll pitch yaw (rad) — e.g. tilt the camera down with a negative pitch
    # Args order: --x --y --z --yaw --pitch --roll --frame-id --child-frame-id
    # -------------------------------------------------------------------------
    camera_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera',
        arguments=[
            '--x', '0.20', '--y', '0.0', '--z', '0.20',
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'camera_link',
        ],
        output='screen',
    )

    return LaunchDescription([
        realsense_node,
        camera_mount_tf,
    ])
