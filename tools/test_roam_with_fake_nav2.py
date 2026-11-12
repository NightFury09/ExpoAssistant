#!/usr/bin/python3
"""Free roam, end to end, with no rover: the REAL console node against a FAKE Nav2.

    source /opt/ros/humble/setup.bash
    source ~/AGX_Orin_Backup/rover_project/uros_ws/install/local_setup.bash
    source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash
    ROS_DOMAIN_ID=81 ROS_LOCALHOST_ONLY=1 python3 tools/test_roam_with_fake_nav2.py

Takes about two minutes and prints PASS/FAIL per check, ending in
ALL PASSED (exit 0) or the list of failures (exit 1). Run it after changing
anything to do with roaming, goals, or the stop paths.

The map is the saved room map; the sensors are faked as live; the map->base TF
is a real broadcast, so the pose reaches the console by the path it takes on the
rover; navigate_to_pose is a fake action server this script steers (succeed,
abort, or hang). Use an unused ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY so it cannot
see, or disturb, a real rover on the network.

It covers: refusals to start, normal roaming, repeated failure, stop-while-
driving, each safety stop (keyboard, e-stop, silent lidar, stack change, lost
position), an operator goal pre-empting a roam, and the per-goal timeout.
"""
import math, sys, threading, time
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import rover_core.rover_dashboard as D
from rover_core.roam import load_pgm_map

import os
MAP = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    '~/AGX_Orin_Backup/rover_project/ros2_ws/src/my_robot_bringup/maps/room_map_v6.yaml')
FAILS = []


def check(name, cond, detail=''):
    print(('PASS  ' if cond else 'FAIL  ') + name + (f'   [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


class FakeNav2(Node):
    """behaviour: succeed | abort | hang"""
    def __init__(self):
        super().__init__('fake_nav2')
        self.behaviour = 'succeed'
        self.goals, self.cancels, self.finished = [], 0, 0
        # Reentrant + multithreaded: execute() blocks while "driving", and on a
        # single thread it would never get to process the cancel that ends it.
        self.srv = ActionServer(self, NavigateToPose, 'navigate_to_pose',
                                self.execute, cancel_callback=self.on_cancel,
                                callback_group=ReentrantCallbackGroup())
        # The real map->base_footprint transform, so the console's own pose
        # timer sees it. tf_on=False simulates AMCL dying (the stamp goes stale).
        self.tf_on, self.tf_xy = True, (3.22, -0.42)
        self.br = TransformBroadcaster(self)
        self.create_timer(0.1, self.pub_tf)

    def pub_tf(self):
        if not self.tf_on:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'base_footprint'
        t.transform.translation.x, t.transform.translation.y = self.tf_xy
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

    def on_cancel(self, _):
        self.cancels += 1
        return CancelResponse.ACCEPT

    def execute(self, gh):
        p = gh.request.pose.pose.position
        self.goals.append((p.x, p.y))
        t0 = time.time()
        while time.time() - t0 < (0.6 if self.behaviour != 'hang' else 60):
            if gh.is_cancel_requested:
                gh.canceled()
                return NavigateToPose.Result()
            time.sleep(0.05)
        self.finished += 1
        if self.behaviour == 'abort':
            gh.abort()
        else:
            gh.succeed()
        return NavigateToPose.Result()


rclpy.init()
dash = D.Dash()
D.NODE = dash
fake = FakeNav2()
ex1, ex2 = SingleThreadedExecutor(), MultiThreadedExecutor(num_threads=4)
ex1.add_node(dash); ex2.add_node(fake)
threading.Thread(target=ex1.spin, daemon=True).start()
threading.Thread(target=ex2.spin, daemon=True).start()

grid, meta = load_pgm_map(MAP)
meta['w'], meta['h'] = grid.shape[1], grid.shape[0]
dash.map_grid, dash.map_meta = grid, meta
dash.sup.status = lambda: {'state': 'navigation', 'mode': 'navigation', 'ready': '', 'opts': {}}
dash.roam_dwell = 0.3
POSE = {'x': 3.22, 'y': -0.42, 'yaw': 0.0}
running = True
feed_lidar = True


def feed():                       # keep the sensors looking alive
    while running:
        with dash.lock:
            if feed_lidar:
                dash.scan_stats = ({'valid_pct': 50, 'n_valid': 300, 'nearest_fwd': 2.0}, time.time())
            dash.diag = ((0, 0, 0), time.time())
        time.sleep(0.2)


threading.Thread(target=feed, daemon=True).start()


def wait_for(cond, timeout, step=0.1):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(step)
    return False


def view():
    with dash.lock:
        return dash._roam_view(time.time())


def reset():
    global feed_lidar
    feed_lidar = True
    dash.sup.status = REAL
    dash.set_cmd(0.0, 0.0)
    dash.roam_stop('test reset')
    with dash.lock: dash.nav_cancel_req = True          # also ends a goal an operator set
    wait_for(lambda: dash.nav['state'] not in ('sending', 'active', 'cancelling'), 5)
    fake.goals.clear(); fake.cancels = 0
    with dash.lock:
        fake.tf_on = True; dash.estop = False; dash.mode = 'idle'
        dash.roam['msg'] = ''
    time.sleep(1.2)               # sensors and pose refresh; manual-drive tail (0.8 s) expires
    dash.roam_goal_timeout = 120.0


REAL = dash.sup.status
time.sleep(2.0)                    # let the action client and TF settle
check('pose arrives through the real TF path', dash.map_pose is not None, str(dash.map_pose))
print('--- refusals')
dash.estop = True
ok, msg = dash.roam_start(); check('refuses while e-stopped', not ok, msg); dash.estop = False
real = dash.sup.status
dash.sup.status = lambda: {'state': 'mapping', 'opts': {}}
ok, msg = dash.roam_start(); check('refuses outside navigation mode', not ok and 'Navigation' in msg, msg)
dash.sup.status = real
fake.tf_on = False; time.sleep(3.6)        # pose goes stale
ok, msg = dash.roam_start(); check('refuses when not localised', not ok and 'localised' in msg, msg); fake.tf_on = True; time.sleep(0.6)
feed_lidar = False
with dash.lock: dash.scan_stats = ({}, time.time() - 10)
ok, msg = dash.roam_start(); check('refuses when the lidar is silent', not ok and 'lidar' in msg, msg)

print('--- normal roaming (Nav2 succeeds)')
reset(); fake.behaviour = 'succeed'
ok, msg = dash.roam_start(); check('starts', ok, msg)
check('reaches 4 stops in 25 s', wait_for(lambda: view()['goals'] >= 4, 25), str(view()))
check('still on, no failures', view()['on'] and view()['fails'] == 0, str(view()))
g = list(fake.goals)
check('goals are distinct, >= 1 m apart from the previous 6',
      all(math.hypot(a[0] - b[0], a[1] - b[1]) >= 0.99 for i, a in enumerate(g) for b in g[max(0, i - 6):i]),
      f'{len(g)} goals')
check('every goal is on clear floor', all(
    grid[int((y - meta['oy']) / meta['res']), int((x - meta['ox']) / meta['res'])] == 0 for x, y in g))
dash.roam_stop('test done')
check('stop turns it off', not view()['on'])

print('--- Nav2 keeps aborting')
reset(); fake.behaviour = 'abort'
ok, msg = dash.roam_start(); check('starts (abort run)', ok, msg)
check('gives up after max fails', wait_for(lambda: not view()['on'], 30), str(view()))
check('says why', 'gave up' in view()['msg'], view()['msg'])
check('tried max_fails times, no more', len(fake.goals) == dash.roam_max_fails, f'{len(fake.goals)} goals')

print('--- stop while driving cancels the goal')
reset(); fake.behaviour = 'hang'
ok, msg = dash.roam_start(); check('starts (hang run)', ok, msg)
check('goal goes out', wait_for(lambda: len(fake.goals) == 1 and dash.nav['state'] == 'active', 8), dash.nav['state'])
dash.roam_stop('operator pressed stop')
check('Nav2 sees the cancel', wait_for(lambda: fake.cancels >= 1, 5), f'cancels={fake.cancels}')
check('nav returns to idle', wait_for(lambda: dash.nav['state'] == 'idle', 5), dash.nav['state'])
time.sleep(1.5)
check('no further goal is sent after stopping', len(fake.goals) == 1, f'{len(fake.goals)} goals')

print('--- safety stops')
def trip_lidar():
    global feed_lidar
    feed_lidar = False
    with dash.lock: dash.scan_stats = ({}, time.time() - 10)

for name, trip, expect in [
    ('keyboard takeover', lambda: dash.set_cmd(0.2, 0.0), 'keyboard'),   # a drive command, as the page sends it
    ('e-stop', lambda: setattr(dash, 'estop', True), 'e-stop'),
    ('lidar goes silent', trip_lidar, 'lidar'),
    ('stack leaves navigation', lambda: setattr(dash.sup, 'status', lambda: {'state': 'idle', 'opts': {}}), 'idle'),
]:
    reset(); fake.behaviour = 'hang'
    ok, msg = dash.roam_start()
    wait_for(lambda: dash.nav['state'] == 'active', 8)
    check(f'[{name}] was roaming and driving', ok and dash.nav['state'] == 'active', msg)
    trip()
    check(f'[{name}] stops roaming', wait_for(lambda: not view()['on'], 4) and expect in view()['msg'], view()['msg'])
    check(f'[{name}] cancels the goal', wait_for(lambda: fake.cancels >= 1, 4), f'cancels={fake.cancels}')

print('--- losing the map position')
reset(); fake.behaviour = 'hang'; ok, msg = dash.roam_start(); check('starts (position-loss run)', ok, msg)
wait_for(lambda: dash.nav['state'] == 'active', 8)
fake.tf_on = False                      # AMCL dies: the last pose goes stale
check('survives a short loss (2.5 s)', not wait_for(lambda: not view()['on'], 2.5), str(view()))
check('stops once the position has been stale ~6 s', wait_for(lambda: not view()['on'], 10) and 'position' in view()['msg'], view()['msg'])
fake.tf_on = True

print('--- operator goal while roaming wins')
reset(); fake.behaviour = 'hang'; ok, msg = dash.roam_start(); check('starts (operator-goal run)', ok, msg)
wait_for(lambda: dash.nav['state'] == 'active', 8)
n0 = fake.cancels
dash.roam_stop('a goal was set by hand', cancel=False)          # what /api/goal does
with dash.lock: dash.nav_req = {'x': 3.0, 'y': -0.4, 'yaw': 0.0}
check('operator goal is sent', wait_for(lambda: len(fake.goals) == 2, 5), f'{len(fake.goals)} goals')
check('roam is off and did not cancel it', not view()['on'] and fake.cancels == n0, f'cancels {fake.cancels - n0}')
time.sleep(1.5)
check('roam does not send more goals', len(fake.goals) == 2, f'{len(fake.goals)} goals')

print('--- goal timeout')
reset(); fake.behaviour = 'hang'; dash.roam_goal_timeout = 2.0; ok, msg = dash.roam_start(); check('starts (timeout run)', ok, msg)
check('cancels a goal that takes too long', wait_for(lambda: fake.cancels >= 1, 8), f'cancels={fake.cancels}')
check('counts it as a failure and carries on', wait_for(lambda: len(fake.goals) >= 2, 8) and view()['on'], str(view()))
dash.roam_stop('end')

print('--- stop when nothing is roaming is harmless')
check('roam_stop returns False when off', dash.roam_stop('x') is False)

running = False
print('\nRESULT:', 'ALL PASSED' if not FAILS else f'{len(FAILS)} FAILED: {FAILS}')
dash.destroy_node(); fake.destroy_node(); rclpy.shutdown()
sys.exit(1 if FAILS else 0)
