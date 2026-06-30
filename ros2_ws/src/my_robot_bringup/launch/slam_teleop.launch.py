# slam_teleop.launch.py
#
# All-in-one launch for building a map with SLAM while driving with WASD.
# This combines: micro-ROS agent + RPLIDAR + SLAM Toolbox + Foxglove Bridge.
#
# Usage:
#   source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash
#   ros2 launch my_robot_bringup slam_teleop.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    bringup_dir        = get_package_share_directory('my_robot_bringup')
    description_dir    = get_package_share_directory('rover_description')
    slam_params_file   = os.path.join(bringup_dir, 'config', 'slam_params.yaml')
    urdf_path          = os.path.join(description_dir, 'urdf', 'rover.urdf')

    # -------------------------------------------------------------------------
    # Node 0: Hardware Reset ESP32
    # Toggles DTR/RTS to hard reset the ESP32 before starting the agent
    # -------------------------------------------------------------------------
    esp32_reset_action = ExecuteProcess(
        cmd=['ros2', 'run', 'my_robot_bringup', 'esp32_reset.py'],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # Node 1: micro-ROS agent — connects ROS 2 to the ESP32
    # Delayed by 3 seconds to let ESP32 boot after reset
    # -------------------------------------------------------------------------
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyESP32', '-b', '115200'],
        output='screen'
    )
    delayed_micro_ros = TimerAction(
        period=3.0,
        actions=[micro_ros_agent_node]
    )

    # -------------------------------------------------------------------------
    # Node 2: Robot State Publisher 
    # Publishes the TF tree for all fixed joints (base_footprint -> laser_frame)
    # -------------------------------------------------------------------------
    robot_description = ParameterValue(
        Command(['cat ', urdf_path]),
        value_type=str
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # Node 3: Odometry & Kinematics (rover_odometry)
    # Subscribes to /wheel_ticks from ESP32.
    # Publishes /odom, /tf (odom->base_footprint), and /joint_states for wheels
    # -------------------------------------------------------------------------
    rover_odometry_node = Node(
        package='rover_core',
        executable='rover_odometry',
        name='rover_odometry',
        output='screen'
    )

    # -------------------------------------------------------------------------
    # Node 4: RPLIDAR A1M8
    # auto_standby=True: Motor stops when no one subscribes to /scan
    # -------------------------------------------------------------------------
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port':      '/dev/ttyLIDAR',
            'serial_baudrate':  115200,
            'frame_id':         'laser_frame',
            'angle_compensate': True,
            'scan_mode':        'Sensitivity',
            'inverted':         False,
            'auto_standby':     True,
        }],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # Node 5: SLAM Toolbox — builds /map from /scan
    # -------------------------------------------------------------------------
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params_file, {'use_sim_time': False}],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # Node 6: Foxglove Bridge
    # Opens websocket on port 8765 for Foxglove Studio
    # -------------------------------------------------------------------------
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
        }],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # Helper: Print clickable URL for Foxglove Studio
    # -------------------------------------------------------------------------
    print_url_action = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['echo', '-e', '\n\033[1;32m'
                     '====================================================================================\n'
                     'Foxglove Studio is ready! Ctrl+Click the link below to auto-connect in your browser:\n'
                     '\n'
                     '    https://studio.foxglove.dev/?ds=foxglove-websocket&ds.url=ws://192.168.3.224:8765\n'
                     '\n'
                     '====================================================================================\033[0m\n'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        esp32_reset_action,
        delayed_micro_ros,
        robot_state_publisher_node,
        rover_odometry_node,
        rplidar_node,
        slam_toolbox_node,
        foxglove_bridge_node,
        print_url_action,
    ])

