import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Parameters & Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Define the path to your custom slam_toolbox_params.yaml file
    slam_params_file = LaunchConfiguration('slam_params_file')

    # --- Declare Launch Arguments ---
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("my_robot_bringup"),
                                   'config', 'slam_toolbox_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # --- Node Definitions ---

    # Node 1: Static Transform for odom -> base_link
    # Required placeholder for SLAM Toolbox to start.
    static_transform_publisher_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )
    
    # Node 2: Static Transform for base_link -> laser_frame
    # MUST match the physical mounting of your LIDAR.
    static_transform_publisher_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0.15', '0.0', '0.20', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )

    # Node 3: The RPLIDAR node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyLIDAR', # Ensure this is your LIDAR's port
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Sensitivity'
        }],
        output='screen'
    )

    # Node 4: The SLAM Toolbox node with the specified parameters file
    slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    # --- LaunchDescription ---
    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_slam_params_file_cmd,
        static_transform_publisher_odom_node,
        static_transform_publisher_lidar_node,
        rplidar_node,
        slam_toolbox_node
    ])