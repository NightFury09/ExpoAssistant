# realsense.launch.py
#
# Intel RealSense D455 for the rover: depth + colour + pointcloud, plus the
# static transform that mounts the camera on the robot.
#
# ---------------------------------------------------------------------------
# WHAT THIS CAMERA IS AND IS NOT FOR ON THIS ROVER
#
# Mounted mast-top at 0.85 m, pitched 9.5 deg NOSE-DOWN, facing forward.
# That geometry decides the role, so be explicit about it:
#
#   D455 depth FOV is 58 deg vertical -> half-angle 29 deg. With 9.5 deg of
#   downward tilt the lowest ray leaves at -(29 + 9.5) = -38.5 deg, so it meets
#   the floor at
#       0.85 / tan(38.5 deg) = 1.07 m
#   The camera cannot see the floor closer than ~1.1 m ahead. (Level at 0.82 m
#   it was 1.48 m -- tilting down bought ~0.4 m of near coverage, at the cost of
#   seeing less high up: the top ray now reaches 1.9 m of height at 3 m range
#   instead of 2.5 m.)
#
# Consequences, and they matter:
#   * Low objects near the base (a box, a foot, a curb) that do not reach the
#     lidar plane at 0.275 m are INVISIBLE to the camera. The 2D lidar remains
#     the primary near-field obstacle sensor. This camera does not replace it.
#   * Drop-offs and steps near the base cannot be seen at all.
#   * The ~0.52 m Min-Z dead zone is irrelevant here: nothing that close is in
#     the frustum anyway.
#
# What the geometry is genuinely good for:
#   * Mid/far-field obstacles (beyond ~1.5 m) and anything tall enough to enter
#     the frustum closer in. At 0.8 m ahead the frustum spans heights
#     0.38-1.26 m, so a tall obstacle close by IS seen -- complementary to a
#     lidar that only samples 0.275 m.
#   * People and object perception. 0.82 m is about adult torso height and the
#     useful depth range (~0.5-6 m) lands exactly where people are. Person
#     detection / following / signage recognition is what this mount is
#     optimised for.
#
# So: LiDAR = near-field safety. Camera = mid/far field, tall obstacles,
# and perception. Configure the costmap accordingly (see nav2_params.yaml).
# ---------------------------------------------------------------------------
#
# PREREQUISITES
#   The camera is a shared resource -- only one process may own it:
#       docker rm -f realsense_demo
#
# Usage:
#   ros2 launch my_robot_bringup realsense.launch.py
#   ros2 launch my_robot_bringup realsense.launch.py cam_x:=0.25 cam_z:=0.82
#
# Verify the mount before trusting the costmap:
#   ros2 run tf2_ros tf2_echo base_link camera_link
#   python3 tools/check_camera_mount.py
# ---------------------------------------------------------------------------

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ---- Mount geometry, overridable without editing this file --------------
    # x: forward from base_link origin (chassis centre) to the camera lens
    # y: left(+) / right(-) of the centre line
    # z: lens height above the FLOOR (base_link sits at ground level)
    # pitch: negative = nose-down. Keep 0 unless the camera is physically
    #        tilted; a pitch error rotates the whole cloud and makes the floor
    #        project upward as a phantom wall.
    args = [
        # 0.24 m: measured 2026-09-09, lens at the exact mid-point of the deck's
        # forward overhang. base_link is the chassis centre, so this is forward
        # of it.
        DeclareLaunchArgument('cam_x',     default_value='0.24',
                              description='camera lens forward offset from chassis centre (m)'),
        DeclareLaunchArgument('cam_y',     default_value='0.0',
                              description='camera lens lateral offset, +left (m)'),
        # 0.850 measured by floor-plane fit, not by tape. Tape said 0.82 to the
        # camera body; the fit resolves the lens's optical centre.
        DeclareLaunchArgument('cam_z',     default_value='0.850',
                              description='camera lens height above the floor (m)'),
        # 0.166 rad = 9.51 deg NOSE-DOWN, calibrated 2026-09-09 by fitting a
        # plane to the floor (10024 inliers, 8 mm residual, residual pitch error
        # -0.19 deg). Note the sign: in ROS, POSITIVE pitch is nose-down.
        # Verify after any change to the mount:
        #   python3 tools/check_camera_mount.py
        DeclareLaunchArgument('cam_pitch', default_value='0.166',
                              description='camera pitch in radians, POSITIVE = nose-down'),
    ]

    # ---- D455 driver --------------------------------------------------------
    # Resolution is deliberately modest. Two reasons:
    #  1. The camera has been enumerating at USB 2.0 on this AGX (SuperSpeed
    #     lanes do not come up on its USB-A ports), which caps throughput.
    #  2. An undecimated 480x270 cloud is ~130k points; at 15 Hz that is ~2M
    #     points/sec into the costmap, which it cannot digest usefully. The
    #     decimation filter below cuts that by 4x.
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'enable_depth':          True,
            'enable_color':          True,
            # Aligned depth gives pixel-for-pixel correspondence with the colour
            # image. Not needed for costmap obstacles, but it is what makes the
            # dashboard's depth-over-colour blend view line up, which is the
            # view clients actually understand. Cheap enough on an Orin.
            'align_depth.enable':    True,
            # Left IR stream. This is the raw view the stereo engine works from,
            # and it shows the projector's dot pattern -- a good demo of HOW the
            # camera measures depth rather than just the result.
            'enable_infra1':         True,
            'enable_infra2':         False,

            # On Jetson ARM the pointcloud filter is registered under the NEON
            # name, not plain 'pointcloud.enable'. Publishes
            # /camera/camera/depth/color/points
            'pointcloud__neon_.enable':     True,
            'pointcloud__neon_.ordered_pc': False,

            # DO NOT set depth_module.depth_profile or rgb_camera.color_profile.
            # Measured 2026-09-09 on this unit: ANY explicit depth profile
            # override kills the stream -- the node starts, publishes topics,
            # and then logs "Frames didn't arrive within 5 seconds" forever with
            # depth 99.4% zero pixels. Tried 480x270x15 and 640x360x30; both
            # dead, even though rs-enumerate-devices lists them as supported.
            # The driver default (848x480) streams correctly at 78-84% valid
            # pixels. Control the data volume with decimation below instead of
            # with resolution.
            #
            # If you ever change this, verify with the DEPTH PIXELS, not the
            # topic rate -- the pointcloud publishes at ~30 Hz either way, and
            # looks perfectly healthy while carrying almost no valid depth.

            # Decimation x3: 848x480 -> 284x160, about 38k valid points per
            # cloud instead of ~400k. Cheaper than filtering downstream, and the
            # costmap does not need every pixel.
            'decimation_filter.enable':           True,
            'decimation_filter.filter_magnitude': 3,
            # Suppress depth speckle that would otherwise appear as short-lived
            # phantom obstacles in the costmap.
            'temporal_filter.enable': True,
            'spatial_filter.enable':  True,

            # The IMU is unreachable on the RSUSB userspace backend (no HID
            # access). Unlocking it needs patched kernel modules -- see
            # NEXT_STEPS.md. Leave off rather than log errors every frame.
            'enable_gyro':           False,
            'enable_accel':          False,

            # initial_reset caused USB re-enumeration churn on this machine.
            'initial_reset':         False,
            'enable_sync':           False,

            'camera_name':           'camera',
            'publish_tf':            True,
            'tf_publish_rate':       0.0,   # internal TF is static, latch once
        }],
    )

    # ---- Mount transform: base_link -> camera_link --------------------------
    # tf2 static_transform_publisher arg order: --x --y --z --yaw --pitch --roll
    mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_mount_tf',
        arguments=[
            '--x',     LaunchConfiguration('cam_x'),
            '--y',     LaunchConfiguration('cam_y'),
            '--z',     LaunchConfiguration('cam_z'),
            '--yaw',   '0.0',
            '--pitch', LaunchConfiguration('cam_pitch'),
            '--roll',  '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
        output='screen',
    )

    return LaunchDescription(args + [realsense_node, mount_tf])
