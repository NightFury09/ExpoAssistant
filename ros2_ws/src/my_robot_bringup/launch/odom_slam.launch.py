import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # --- Parameters & Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # --- Declare Launch Arguments ---
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("my_robot_bringup"),
                                   'config', 'slam_toolbox_params.yaml'),
        description='Full path to the ROS2 parameters file for slam_toolbox')

    # --- Node Definitions ---

    # 1. RPLIDAR Node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyLIDAR', # VERIFY your LiDAR's serial port!
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
            'serial_baudrate': 115200,
        }],
        output='screen'
    )
    # 2. Static Transform Publisher for base_link -> laser_frame
    static_tf_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0.15', '0.0', '0.20', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )

    # 3. SLAM Toolbox Node
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
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        declare_slam_params_file_cmd,
        rplidar_node,
        static_tf_laser_node,
        slam_toolbox_node
    ])