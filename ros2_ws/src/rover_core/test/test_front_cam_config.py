"""The front-camera and smoother configuration is internally consistent.

These guard mistakes that ROS does not report: a params file ROS refuses to parse
(the nodes then die at start), a parameter name with a typo (silently ignored, so
the default is used), and a costmap layer set up in a way that would let the
camera erase what the lidar saw.

    python3 -m pytest ros2_ws/src/rover_core/test/test_front_cam_config.py
"""
import ast
import dataclasses
import os
import re

import yaml

from rover_core.ground_vision import VisionParams

ROOT = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')
BRINGUP = os.path.join(ROOT, 'ros2_ws', 'src', 'my_robot_bringup')
CORE = os.path.join(ROOT, 'ros2_ws', 'src', 'rover_core', 'rover_core')


def load(*parts):
    with open(os.path.join(*parts)) as f:
        return yaml.safe_load(f)


FRONT = load(BRINGUP, 'config', 'front_cam.yaml')
NAV2 = load(BRINGUP, 'config', 'nav2_params.yaml')
LOCAL = NAV2['local_costmap']['local_costmap']['ros__parameters']
GLOBAL = NAV2['global_costmap']['global_costmap']['ros__parameters']


def declared(node_file):
    with open(os.path.join(CORE, node_file)) as f:
        return set(re.findall(r"\bd\('([a-z_0-9]+)'", f.read()))


def test_every_section_is_a_node_with_ros_parameters():
    # ROS refuses a params file with a section that has no ros__parameters, and
    # the node dies at start before it can log why.
    for node, body in FRONT.items():
        assert isinstance(body, dict) and 'ros__parameters' in body, node


def test_every_yaml_key_is_a_parameter_the_node_declares():
    cap = declared('front_cam.py')
    scan = declared('front_cam_scan.py') | {f.name for f in dataclasses.fields(VisionParams)}
    for key in FRONT['front_cam']['ros__parameters']:
        assert key in cap, 'front_cam does not declare %r (a typo is silently ignored)' % key
    for key in FRONT['front_cam_scan']['ros__parameters']:
        assert key in scan, 'front_cam_scan does not declare %r' % key


def test_the_mount_is_complete_and_sane():
    p = FRONT['front_cam_scan']['ros__parameters']
    for k in ('x', 'y', 'z', 'pitch_deg', 'yaw_deg'):
        assert isinstance(p[k], (int, float)), k
    assert 0.0 < p['z'] < 1.5 and abs(p['pitch_deg']) < 80 and abs(p['yaw_deg']) < 90


def test_scan_range_matches_the_costmap_layer_range():
    scan = FRONT['front_cam_scan']['ros__parameters']
    src = LOCAL['front_cam_layer']['front_scan']
    assert src['raytrace_max_range'] <= scan['max_range'] + 1e-9, \
        'the layer would clear further than the camera can see'
    assert src['obstacle_max_range'] <= scan['max_range']
    assert src['obstacle_min_range'] <= scan['min_range'] + 1e-9


# ---------------- the costmap layer ----------------
def test_the_camera_has_its_own_layer_before_inflation():
    plugins = LOCAL['plugins']
    assert 'front_cam_layer' in plugins
    assert plugins.index('front_cam_layer') < plugins.index('inflation_layer')
    assert LOCAL['front_cam_layer']['plugin'] == 'nav2_costmap_2d::ObstacleLayer'
    # NOT a source on the lidar's layer: it would clear the lidar's marks
    assert 'front_scan' not in LOCAL['obstacle_layer'].get('observation_sources', '')


def test_the_camera_layer_is_local_only():
    assert 'front_cam_layer' not in GLOBAL['plugins']


def test_the_camera_layer_fails_open():
    src = LOCAL['front_cam_layer']['front_scan']
    assert src['expected_update_rate'] == 0.0, \
        'a silent camera must not mark the costmap not-current and stall the controller'
    assert src['inf_is_valid'] is True, 'without it the layer can never clear its marks'
    assert src['marking'] is True and src['clearing'] is True
    assert src['data_type'] == 'LaserScan' and src['topic'] == '/front_cam/scan'


# ---------------- the velocity smoother ----------------
SM = NAV2['velocity_smoother']['ros__parameters']
CTRL = NAV2['controller_server']['ros__parameters']['FollowPath']


def test_the_smoother_never_clips_what_the_controller_asks_for():
    assert SM['max_velocity'][0] >= CTRL['desired_linear_vel']
    assert SM['max_velocity'][2] >= CTRL['rotate_to_heading_angular_vel']
    assert SM['min_velocity'][0] <= -0.0 and SM['min_velocity'][2] <= -CTRL['rotate_to_heading_angular_vel']


def test_the_smoother_actually_limits_something():
    # Nav2's built-in default is 2.5 m/s^2: for a 0.22 m/s rover that is a no-op.
    assert 0.0 < SM['max_accel'][0] < 1.5
    assert abs(SM['max_decel'][0]) >= SM['max_accel'][0], 'stop harder than you start'
    assert SM['feedback'] == 'OPEN_LOOP', 'closed loop inherits the slow odometry'
    assert SM['velocity_timeout'] <= 1.0


def test_the_launch_files_parse_and_wire_the_camera():
    for name in ('front_cam.launch.py', 'master_navigation.launch.py'):
        with open(os.path.join(BRINGUP, 'launch', name)) as f:
            ast.parse(f.read())
    with open(os.path.join(BRINGUP, 'launch', 'master_navigation.launch.py')) as f:
        src = f.read()
    assert "'use_front_cam', default_value='false'" in src, 'the camera must be opt-in'
    assert 'front_cam.launch.py' in src
