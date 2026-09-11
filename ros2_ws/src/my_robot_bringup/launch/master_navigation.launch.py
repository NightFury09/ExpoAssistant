# master_navigation.launch.py
#
# Full autonomous navigation on a saved map, with the D455 pointcloud feeding
# Nav2's costmaps (obstacle layer) alongside the 2D lidar.
#
# TF chain (single owner per edge):
#   map -> odom            : AMCL          (nav2_bringup)
#   odom -> base_footprint : rover_odometry (publish_tf:=true)
#   base_footprint -> ...  : robot_state_publisher (URDF)
# rover_odometry publishes both the /odom topic and the odom->base_footprint
# TF, the same as in SLAM mode. The EKF was removed 2026-09-09: with a single
# pose source and no IMU it could add no information, only lag.
#
# The RealSense camera is NOT started here (it is time-shared with the booth).
# Run it in parallel once the camera is free:
#   docker rm -f realsense_demo          # free the D455
#   ros2 launch my_robot_bringup realsense.launch.py
# Its /camera/camera/depth/color/points then flows into the costmaps.
#
# Usage:
#   source /opt/ros/humble/setup.bash
#   source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash
#   ros2 launch my_robot_bringup master_navigation.launch.py
# Headless: visualize/interact in Foxglove Studio (Windows) at
#   ws://<jetson-ip>:8765
# Set the AMCL initial pose by publishing geometry_msgs/PoseWithCovarianceStamped
# to /initialpose, and send goals by publishing geometry_msgs/PoseStamped to
# /goal_pose (both from the Foxglove 3D panel's publish tools).

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            ExecuteProcess, TimerAction)
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    my_robot_bringup_path  = get_package_share_directory('my_robot_bringup')
    rover_description_path = get_package_share_directory('rover_description')
    nav2_bringup_path      = get_package_share_directory('nav2_bringup')

    # Default map. room_map_v6 (2026-09-08) is the best survey so far:
    # 49.3% free floor and 5.9% walls in an 8.7 x 5.9 m extent, versus 16.1% /
    # 1.9% for v4. Crisp single-line walls, no ghosting.
    # Built after the lidar transform was corrected (rear mount, 180 deg yaw),
    # Express scan mode enabled, and odometry moved onto real encoders.
    # my_room_map is from July 2025 and predates all of that.
    # Override without editing this file:
    #   ros2 launch my_robot_bringup master_navigation.launch.py \
    #        map:=/home/rptech/AGX_Orin_Backup/rover_project/maps/<name>.yaml
    default_map_path = os.path.join(my_robot_bringup_path, 'maps', 'room_map_v6.yaml')
    nav2_params_path = os.path.join(my_robot_bringup_path, 'config', 'nav2_params.yaml')

    # The behaviour tree path can only reach Nav2 through the params file --
    # bringup_launch.py accepts params_file and nothing else -- but hardcoding
    # an absolute install path in a committed YAML is wrong. RewrittenYaml is
    # nav2's own mechanism for exactly this: substitute at launch time.
    #
    # booth_recovery.xml drops Spin and BackUp from the stock recovery tree and
    # clears/waits instead. At an expo the usual reason planning fails is a
    # PERSON in the way; spinning beside a booth table alarms visitors and
    # sweeps the chassis corners through unchecked space, and reversing is worse
    # because the rover is blind behind at camera height where the crowd is.
    booth_bt_path = os.path.join(my_robot_bringup_path, 'behavior_trees',
                                 'booth_recovery.xml')
    nav2_params_rewritten = RewrittenYaml(
        source_file=nav2_params_path,
        root_key='',
        param_rewrites={'default_nav_to_pose_bt_xml': booth_bt_path},
        convert_types=True)
    rviz_config_path = os.path.join(my_robot_bringup_path, 'rviz',   'nav2_config.rviz')
    urdf_path        = os.path.join(rover_description_path, 'urdf',   'rover.urdf')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_map_cmd = DeclareLaunchArgument(
        'map', default_value=default_map_path,
        description='Full path to the map YAML to load')

    # The D455 is optional and time-shared with the booth demo, so it is OFF by
    # default. Enable with:  ros2 launch ... master_navigation.launch.py use_camera:=true
    # Free the camera first if the demo holds it:  docker rm -f realsense_demo
    # Its depth feeds the LOCAL costmap's voxel_layer only -- mid/far-field and
    # tall obstacles. The lidar stays the near-field sensor; see the geometry
    # note at the top of realsense.launch.py for why.
    declare_use_camera_cmd = DeclareLaunchArgument(
        'use_camera', default_value='false',
        description='Also bring up the RealSense D455 for 3D obstacles')

    # The web dashboard can run ALONGSIDE navigation: it only subscribes to
    # topics and publishes /cmd_vel while you are actively driving, staying
    # silent otherwise so Nav2 keeps the wheel. Grab a key and you take over
    # instantly; release and it hands back. Good for demoing autonomy and
    # manual control together, with the camera and metrics on screen throughout.
    # (Do NOT launch teleop_dashboard.launch.py as well -- that starts its own
    # agent, odometry and lidar, which would collide with these.)
    # Prefer running `ros2 run rover_core rover_dashboard` OUTSIDE this launch
    # and leaving it up across restarts: it is the console -- map view, demo
    # points, goals -- and it binds :8080, so a second copy started here would
    # fail to bind and the console would silently stop updating. Only set this
    # true if nothing else is already serving :8080.
    declare_use_dashboard_cmd = DeclareLaunchArgument(
        'use_dashboard', default_value='false',
        description='Also serve the console on :8080 (leave false if it is '
                    'already running standalone)')

    # --- Reset ESP32, then start the micro-ROS agent (produces /wheel_ticks) ---
    esp32_reset_action = ExecuteProcess(
        cmd=['ros2', 'run', 'my_robot_bringup', 'esp32_reset.py'],
        output='screen')

    micro_ros_agent_node = Node(
        package='micro_ros_agent', executable='micro_ros_agent', name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyESP32', '-b', '115200'],
        output='screen')
    delayed_micro_ros = TimerAction(period=3.0, actions=[micro_ros_agent_node])

    # --- URDF TF (base_footprint -> base_link -> laser_frame / camera_link) ---
    robot_description = ParameterValue(Command(['cat ', urdf_path]), value_type=str)
    robot_state_publisher_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen')

    # --- Wheel odometry: publish the /odom TOPIC only (EKF owns the TF here) ---
    # publish_tf TRUE -- rover_odometry owns odom->base_footprint here, exactly
    # as it does in slam_teleop.launch.py.
    #
    # The EKF (robot_localization) used to own this edge and has been removed.
    # It fused x/y/yaw POSE from a SINGLE source with no IMU, which cannot
    # improve anything -- an EKF needs either multiple sources or velocity+pose
    # to add information. All it contributed was latency, an extra timestamp
    # hop, and `odom0_differential: True` differentiating a pose whose
    # covariance was all zeros (i.e. claiming infinite certainty).
    # It was also the main thing that differed between SLAM mode (which built
    # clean maps) and Nav mode (which could not follow a straight line).
    # Bring it back only if a real second source is added -- see NEXT_STEPS.md
    # for the IMU work, which is when an EKF starts to earn its place.
    rover_odometry_node = Node(
        package='rover_core', executable='rover_odometry', name='rover_odometry',
        parameters=[{'publish_tf': True}],
        output='screen')

    # --- RPLIDAR (delayed 5s + respawn: USB settles after the ESP32 reset) ---
    rplidar_node = Node(
        package='rplidar_ros', executable='rplidar_node', name='rplidar_node',
        parameters=[{
            'serial_port':      '/dev/ttyLIDAR',
            'serial_baudrate':  115200,
            'frame_id':         'laser_frame',
            'angle_compensate': True,
            'scan_mode':        'Express',
        }],
        respawn=True, respawn_delay=3.0, output='screen')
    delayed_rplidar = TimerAction(period=5.0, actions=[rplidar_node])


    # --- Full Nav2 stack (map_server + AMCL + planner + controller + costmaps) ---
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_path, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map':          LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file':  nav2_params_rewritten,
        }.items())

    # Foxglove bridge instead of RViz — this is a headless Jetson (SSH from a
    # Windows PC), so visualization/interaction happens in Foxglove Studio over
    # the network at ws://<jetson-ip>:8765. Set the AMCL initial pose by
    # publishing to /initialpose and send goals via /goal_pose (see the launch
    # header notes).
    # Delayed 8s: the camera is the heaviest USB device on the bus and the
    # ESP32 reset at t=0 disturbs it. Bringing it up after the serial devices
    # have settled avoids re-enumeration churn.
    realsense_launch = TimerAction(period=8.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_robot_bringup_path, 'launch', 'realsense.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_camera')))])

    dashboard_node = TimerAction(period=12.0, actions=[Node(
        package='rover_core', executable='rover_dashboard', name='rover_dashboard',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_dashboard')))])

    foxglove_bridge_node = Node(
        package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
        parameters=[{'port': 8765}], output='screen')

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_cmd,
        declare_use_camera_cmd,
        declare_use_dashboard_cmd,
        esp32_reset_action,           # reset ESP32
        delayed_micro_ros,            # agent -> /wheel_ticks
        robot_state_publisher_node,   # URDF TF
        rover_odometry_node,          # /odom topic + odom->base_footprint TF
        delayed_rplidar,              # LiDAR on /dev/ttyLIDAR
        nav2_bringup_launch,          # AMCL + Nav2 stack (costmaps use the pointcloud)
        realsense_launch,             # optional D455 -> local costmap voxel_layer
        dashboard_node,               # optional web UI on :8080 (manual override)
        foxglove_bridge_node,         # visualization over network (ws://<jetson-ip>:8765)
    ])
