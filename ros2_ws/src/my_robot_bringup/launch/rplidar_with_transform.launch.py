import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Using Launch Configurations like the generic file for flexibility ---
    # This allows you to override these values from the command line if needed.
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyLIDAR')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200') # RPLIDAR A1/A2 use 115200
    frame_id = LaunchConfiguration('frame_id', default='laser_frame') # We'll use this consistent name
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    # This is the key parameter that worked in the generic launch file
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity') 
    
    return LaunchDescription([
        # --- Declare launch arguments so they can be seen by 'ros2 launch -s' ---
        DeclareLaunchArgument(
            'channel_type', default_value='serial', description='Specifying channel type of lidar'),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyLIDAR', description='Specifying usb port to connected lidar'),
        DeclareLaunchArgument(
            'serial_baudrate', default_value='115200', description='Specifying usb port baudrate to connected lidar'),
        DeclareLaunchArgument(
            'frame_id', default_value='laser_frame', description='Specifying frame_id of lidar'),
        DeclareLaunchArgument(
            'inverted', default_value='false', description='Specifying whether or not to invert scan data'),
        DeclareLaunchArgument(
            'angle_compensate', default_value='true', description='Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument(
            'scan_mode', default_value='Sensitivity', description='Specifying scan mode of lidar'),

        # --- Node 1: The Static Transform Publisher (from our original custom file) ---
        # This node is CRUCIAL for SLAM. It tells ROS where the LIDAR is on the robot.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_base_link_broadcaster',
            # IMPORTANT: Adjust these X, Y, Z offsets to match your robot's physical measurements!
            arguments=['0.15', '0.0', '0.22', '0', '0', '0', 'base_link', LaunchConfiguration('frame_id')]
        ),
        
        # --- Node 2: A static transform for odom, required by SLAM Toolbox ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),

        # --- Node 3: The RPLIDAR Node (using the known-good parameters) ---
        # This uses the same parameters that worked in the generic launch file.
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode
            }],
            output='screen'
        ),
    ])