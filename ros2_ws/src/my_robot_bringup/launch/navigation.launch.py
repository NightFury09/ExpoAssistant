import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    
    # --- File Paths ---
    my_robot_bringup_path = get_package_share_directory('my_robot_bringup')
    nav2_bringup_path = get_package_share_directory('nav2_bringup')

    map_file_path = os.path.join(my_robot_bringup_path, 'maps', 'my_room_map.yaml')
    nav2_params_path = os.path.join(my_robot_bringup_path, 'config', 'nav2_params.yaml')
    rviz_config_path = os.path.join(my_robot_bringup_path, 'rviz', 'nav2_config.rviz')
    
    # --- NEW: Path to the EKF config file for robot_localization ---
    ekf_config_path = os.path.join(my_robot_bringup_path, 'config', 'ekf.yaml')

    # --- Declare Launch Arguments ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # --- Node & Launch Configurations ---

    # 1. NEW: RPLIDAR Node (replaces sllidar_node)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyLIDAR', # <-- VERIFY your LiDAR's serial port!
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
            'serial_baudrate': 115200,
        }],
        output='screen'
    )

    # 2. Static Transform for base_link -> laser_frame
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser', 
        arguments=['0.15', '0.0', '0.20', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )

    # 3. NEW: Robot Localization EKF Node
    # This node takes the /odom topic from your ESP32 and provides the odom->base_link transform
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[ekf_config_path]
    )

    # 4. Nav2 Bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_path, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params_path
        }.items(),
    )

    # 5. RViz2
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_path],
    #     output='screen'
    # )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        
        rplidar_node,
        static_tf_node,
        robot_localization_node,
        nav2_bringup_launch,
        #rviz_node
    ])