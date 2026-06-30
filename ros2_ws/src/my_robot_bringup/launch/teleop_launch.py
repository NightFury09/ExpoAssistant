# teleop_launch.py
# Launches the micro-ROS agent (ESP32 bridge) + WASD teleop node.
# Run this to drive the rover from the keyboard.
#
# Usage:
#   source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash
#   ros2 launch my_robot_bringup teleop_launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Node 1: micro-ROS agent — bridges ROS 2 to the ESP32 over USB serial
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyESP32', '-b', '115200'],
        output='screen'
    )

    # Node 2: WASD teleop — keyboard control node
    # WASD keys, Q/E to adjust speed, SPACE to stop.
    # Runs directly in the terminal (no xterm required).
    teleop_node = Node(
        package='rover_core',
        executable='rover_teleop',
        name='rover_teleop',
        output='screen',
        emulate_tty=True,  # Required for terminal key capture
        prefix='',
    )

    return LaunchDescription([
        micro_ros_agent_node,
        teleop_node,
    ])