# teleop_dashboard.launch.py
#
# Drive the rover from a web browser, with the RealSense feed and live metrics.
#
#   ros2 launch my_robot_bringup teleop_dashboard.launch.py
#   -> open http://192.168.3.224:8080
#
# Self-contained: NO SLAM, NO Nav2. Bring this up on its own.
#
# IMPORTANT -- never run this alongside slam_teleop.launch.py or
# master_navigation.launch.py. Each of those starts its OWN micro-ROS agent,
# rover_odometry and rplidar. Running two stacks gives two publishers on
# odom->base_footprint, TF returns whichever arrived last, and everything
# downstream behaves erratically. That cost hours of debugging on 2026-09-09.
# Check before launching anything else:
#     ros2 node list | sort
#
# Startup order matters: the ESP32 reset at t=0 disturbs the USB bus, so the
# agent, lidar and camera are staggered behind it.
#
# Arguments:
#   use_camera:=false   skip the RealSense (dashboard still shows metrics)
#   port:=8080          dashboard HTTP port

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, TimerAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup = get_package_share_directory('my_robot_bringup')
    desc    = get_package_share_directory('rover_description')
    urdf    = os.path.join(desc, 'urdf', 'rover.urdf')

    args = [
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='bring up the RealSense D455'),
        DeclareLaunchArgument('port', default_value='8080',
                              description='dashboard HTTP port'),
    ]

    esp32_reset = ExecuteProcess(
        cmd=['ros2', 'run', 'my_robot_bringup', 'esp32_reset.py'], output='screen')

    agent = TimerAction(period=3.0, actions=[Node(
        package='micro_ros_agent', executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyESP32', '-b', '115200'],
        output='screen')])

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description':
                     ParameterValue(Command(['cat ', urdf]), value_type=str)}],
        output='screen')

    # publish_tf true: nothing else owns odom->base_footprint in this mode.
    odom = Node(
        package='rover_core', executable='rover_odometry', name='rover_odometry',
        parameters=[{'publish_tf': True}], output='screen')

    lidar = TimerAction(period=5.0, actions=[Node(
        package='rplidar_ros', executable='rplidar_node', name='rplidar_node',
        parameters=[{'serial_port': '/dev/ttyLIDAR', 'serial_baudrate': 115200,
                     'frame_id': 'laser_frame', 'angle_compensate': True,
                     'scan_mode': 'Express'}],
        respawn=True, respawn_delay=3.0, output='screen')])

    camera = TimerAction(period=8.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'realsense.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_camera')))])

    # Last: it only reads topics and publishes /cmd_vel, so nothing waits on it.
    dashboard = TimerAction(period=11.0, actions=[Node(
        package='rover_core', executable='rover_dashboard', name='rover_dashboard',
        output='screen')])

    banner = TimerAction(period=14.0, actions=[ExecuteProcess(
        cmd=['bash', '-c',
             'echo ""; echo "======================================================"; '
             'echo "  Rover dashboard:  http://$(hostname -I | awk \'{print $1}\'):8080"; '
             'echo "  W A S D to drive, Space = emergency stop"; '
             'echo "======================================================"; echo ""'],
        output='screen')])

    return LaunchDescription(args + [
        esp32_reset, rsp, odom, agent, lidar, camera, dashboard, banner])
