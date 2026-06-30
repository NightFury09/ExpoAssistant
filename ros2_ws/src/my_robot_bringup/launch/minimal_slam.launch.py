import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        
    # Path to the new minimal parameters file.
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("my_robot_bringup"),
                                   'config', 'minimal_slam.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

        # RPLIDAR node with known-good settings
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyLIDAR', # Ensure this is your LIDAR's port
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Sensitivity'
        }],
        output='screen'
    )

    # SLAM Toolbox node using the minimal parameters
    slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': False}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    return LaunchDescription([
        declare_slam_params_file_cmd,
        rplidar_node,
        slam_toolbox_node
    ])
