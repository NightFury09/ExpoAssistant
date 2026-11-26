# front_cam.launch.py
#
# The front webcam: capture, floor-boundary scan, and the TF that mounts it.
#
#   ros2 launch my_robot_bringup front_cam.launch.py
#   ros2 launch my_robot_bringup front_cam.launch.py cam_x:=0.25 cam_z:=0.25 cam_pitch:=22
#
# Publishes /front_cam/image_raw, /front_cam/camera_info, /front_cam/scan (the one
# the local costmap's front_cam_layer reads), /front_cam/status.
# master_navigation.launch.py includes this when you pass use_front_cam:=true.
#
# PREREQUISITE: the camera has one owner at a time. The reception greeter
# (reception.py, systemd 'reception') opens /dev/video0, so stop it first or the
# capture node just keeps retrying and says why.
#
# The mount comes from config/front_cam.yaml and drives both the TF below and the
# scan node's projection. Overriding cam_* here changes both together.

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg_path = os.path.join(get_package_share_directory('my_robot_bringup'),
                            'config', 'front_cam.yaml')
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    m = cfg['front_cam_scan']['ros__parameters']

    device_arg = DeclareLaunchArgument(
        'cam_device', default_value='',
        description='V4L2 device; empty = the one in front_cam.yaml')
    args = [DeclareLaunchArgument('cam_' + k, default_value=str(m[v]),
                                  description='front camera mount: ' + v)
            for k, v in (('x', 'x'), ('y', 'y'), ('z', 'z'),
                         ('pitch', 'pitch_deg'), ('yaw', 'yaw_deg'))]
    L = {k: LaunchConfiguration('cam_' + k) for k in ('x', 'y', 'z', 'pitch', 'yaw')}

    def f(sub):
        return ParameterValue(sub, value_type=float)

    mount_params = {'x': f(L['x']), 'y': f(L['y']), 'z': f(L['z']),
                    'pitch_deg': f(L['pitch']), 'yaw_deg': f(L['yaw'])}

    capture_params = [cfg_path]
    # An override only when one was given, so the yaml stays the default.
    capture_dev = Node(package='rover_core', executable='front_cam', name='front_cam',
                       parameters=[cfg_path, {'device': LaunchConfiguration('cam_device')}],
                       respawn=True, respawn_delay=3.0, output='screen',
                       condition=IfCondition(PythonExpression(
                           ["'", LaunchConfiguration('cam_device'), "' != ''"])))
    capture = Node(package='rover_core', executable='front_cam', name='front_cam',
                   parameters=capture_params, respawn=True, respawn_delay=3.0,
                   output='screen',
                   condition=IfCondition(PythonExpression(
                       ["'", LaunchConfiguration('cam_device'), "' == ''"])))
    scan = Node(package='rover_core', executable='front_cam_scan', name='front_cam_scan',
                parameters=[cfg_path, mount_params], respawn=True, respawn_delay=3.0,
                output='screen')

    # static_transform_publisher takes yaw pitch roll in radians.
    def rad(sub):
        return PythonExpression(['float(', sub, ') * ', str(math.pi / 180.0)])

    # base_link -> front_cam_scan: the camera's position and heading, NO tilt, so
    # the scan is a level plane Nav2 can treat like a lidar's.
    tf_scan = Node(package='tf2_ros', executable='static_transform_publisher',
                   name='front_cam_scan_tf',
                   arguments=[L['x'], L['y'], L['z'], rad(L['yaw']), '0', '0',
                              'base_link', 'front_cam_scan'])
    # base_link -> front_cam_link (x forward, tilted) -> optical frame (z forward)
    tf_link = Node(package='tf2_ros', executable='static_transform_publisher',
                   name='front_cam_link_tf',
                   arguments=[L['x'], L['y'], L['z'], rad(L['yaw']), rad(L['pitch']), '0',
                              'base_link', 'front_cam_link'])
    tf_optical = Node(package='tf2_ros', executable='static_transform_publisher',
                      name='front_cam_optical_tf',
                      arguments=['0', '0', '0', str(-math.pi / 2), '0', str(-math.pi / 2),
                                 'front_cam_link', 'front_cam_optical_frame'])
    return LaunchDescription([device_arg] + args +
                             [tf_scan, tf_link, tf_optical, capture, capture_dev, scan])
