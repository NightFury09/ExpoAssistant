import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the paths to the config and rplidar launch files
    bringup_dir = get_package_share_directory('my_robot_bringup')
    slam_params_file = os.path.join(bringup_dir, 'config', 'slam_params.yaml')

    # SLAM Toolbox Node with parameters
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': False}]
    )

    # Static Transform Publisher Node (base_link to laser_frame)
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0.15', '0.0', '0.20', '0', '0', '0', 'base_link', 'laser_frame']
    )

    # RPLIDAR Node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyLIDAR', # Ensure this is correct
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Sensitivity'
        }]
    )

    return LaunchDescription([
        static_transform_publisher_node,
        rplidar_node,
        slam_toolbox_node
    ])
