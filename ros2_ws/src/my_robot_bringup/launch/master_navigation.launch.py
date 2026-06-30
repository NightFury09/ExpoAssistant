# master_navigation.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # --- File Paths ---
    my_robot_bringup_path  = get_package_share_directory('my_robot_bringup')
    rover_description_path = get_package_share_directory('rover_description')
    nav2_bringup_path      = get_package_share_directory('nav2_bringup')

    map_file_path    = os.path.join(my_robot_bringup_path, 'maps',  'my_room_map.yaml')
    nav2_params_path = os.path.join(my_robot_bringup_path, 'config', 'nav2_params.yaml')
    ekf_config_path  = os.path.join(my_robot_bringup_path, 'config', 'ekf.yaml')
    rviz_config_path = os.path.join(my_robot_bringup_path, 'rviz',   'nav2_config.rviz')
    urdf_path        = os.path.join(rover_description_path, 'urdf',   'rover.urdf')

    # --- Declare Launch Arguments ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # --- Node & Launch Configurations ---

    # 1. Robot State Publisher (replaces static_transform_publisher for base_link→laser_frame)
    #    Reads rover.urdf to publish TF for all fixed joints.
    robot_description = ParameterValue(
        Command(['cat ', urdf_path]),
        value_type=str
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # 2. RPLIDAR Node
    #    Port: /dev/ttyLIDAR if udev rules are applied, else /dev/ttyUSB1
    #    Run: ls -la /dev/ttyLIDAR to confirm symlink exists before launching.
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port':    '/dev/ttyLIDAR',  # Change to /dev/ttyLIDAR after udev setup
            'frame_id':       'laser_frame',
            'angle_compensate': True,
            'scan_mode':      'Sensitivity',
            'serial_baudrate': 115200,
        }],
        output='screen'
    )

    # 3. Static Transform for odom → base_link
    #    Required by Nav2 / EKF as a seed frame. The EKF will override this
    #    dynamically once /odom data from ESP32 is received.
    static_tf_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )

    # 4. Robot Localization EKF Node
    #    Fuses /odom from ESP32 → publishes /odometry/filtered + odom→base_link TF.
    #    NOTE: Once ESP32 firmware publishes /odom, the static odom TF above
    #    will be superseded by the EKF's dynamic TF broadcast.
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[ekf_config_path]
    )

    # 5. Nav2 Bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_path, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map':          map_file_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file':  nav2_params_path
        }.items(),
    )

    # 6. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_state_publisher_node,   # URDF + TF (replaces static laser TF)
        rplidar_node,                 # LiDAR on /dev/ttyUSB1
        static_tf_odom_node,          # Seed odom→base_link (overridden by EKF)
        robot_localization_node,      # EKF fusing /odom
        nav2_bringup_launch,          # Full Nav2 stack
        rviz_node,                    # Visualization
    ])
