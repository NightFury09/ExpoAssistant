#!/usr/bin/python3
"""Odometry TF, end to end, with no rover: the REAL rover_odometry node fed simulated encoder ticks.

    source /opt/ros/humble/setup.bash
    source ~/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash
    ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 python3 tools/test_odometry_tf.py

About 25 seconds; prints PASS/FAIL per check and ends in ALL PASSED (exit 0).
Use an unused ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY so it cannot see, or disturb,
a real rover on the network.

It checks that odom->base_footprint is published at ~20 Hz and always fresh, that
the predicted position tracks a straight run within 1.5 cm, that a stop overshoots
by no more than the 6 cm cap and settles on the true stopping point, that the
transform goes STALE about a second after the ticks stop (a dead ESP32 must not
leave a fresh-looking pose), and -- the reason it exists -- that essentially every
scan can be looked up at its own timestamp the moment it arrives (measured on the
rover before the fix: 44%). That last check is what AMCL's scan filter needs.
"""
import math, subprocess, sys, time, threading
import rclpy, tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Point32
from tf2_msgs.msg import TFMessage

R, CPR, V, TICK_HZ = 0.05, 2737.0, 0.2, 3.8
TICKS_PER_M = CPR / (2 * math.pi * R)
FAILS = []


def check(name, cond, detail=''):
    print(('PASS  ' if cond else 'FAIL  ') + name + (f'   [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


rclpy.init()
n = Node('odom_tf_probe')
now = lambda: n.get_clock().now().nanoseconds * 1e-9
pub = n.create_publisher(Point32, '/wheel_ticks', 10)
buf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0)); tf2_ros.TransformListener(buf, n)
arrivals = []                                  # (arrival time, stamp, x, y)
def on_tf(m):
    for t in m.transforms:
        if t.child_frame_id == 'base_footprint' and t.header.frame_id == 'odom':
            arrivals.append((now(), t.header.stamp.sec + t.header.stamp.nanosec * 1e-9,
                             t.transform.translation.x, t.transform.translation.y))
n.create_subscription(TFMessage, '/tf', on_tf, 100)

state = {'moving': True, 'alive': True, 'x_true': 0.0, 't_last': None, 'stop_t': None, 'x_at_stop': None}
count = {'l': 0.0, 'r': 0.0}
truth = []                                      # (time, x_true) sampled when each tick is sent

def tick():
    if not state['alive']:
        return
    t = now()
    if state['t_last'] is not None and state['moving']:
        dx = V * (t - state['t_last'])
        state['x_true'] += dx
        count['l'] += dx * TICKS_PER_M; count['r'] += dx * TICKS_PER_M
    state['t_last'] = t
    truth.append((t, state['x_true']))
    pub.publish(Point32(x=count['l'], y=count['r']))
n.create_timer(1.0 / TICK_HZ, tick)

def spin_for(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)

import os, signal
proc = subprocess.Popen(['ros2', 'run', 'rover_core', 'rover_odometry'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
spin_for(3.0)

print('--- cruising at 0.2 m/s for 8 s')
mark = len(arrivals); t0 = now(); spin_for(8.0); t1 = now()
a = arrivals[mark:]
rate = len(a) / (t1 - t0)
check('TF published at ~20 Hz (was ~3.5)', 17 <= rate <= 23, f'{rate:.1f} Hz')
ages = [arr - stamp for arr, stamp, _, _ in a]
check('every transform arrives fresh (stamp age < 60 ms)', max(ages) < 0.06, f'max age {max(ages)*1000:.0f} ms')
# truth at an arbitrary instant: interpolate the straight line from the tick log
x_at = lambda t: V * (t - truth[0][0]) + truth[0][1]
off_cruise = off = a[0][2] - x_at(a[0][1])     # constant start offset (first tick sets the origin)
errs = [abs((x - off) - x_at(stamp)) for _, stamp, x, _ in a if stamp > truth[3][0]]
check('predicted position tracks the true path within 1.5 cm', max(errs) < 0.015, f'max err {max(errs)*100:.2f} cm, mean {sum(errs)/len(errs)*100:.2f} cm')
xs = [x for _, _, x, _ in a]
check('position never jumps backwards while moving', all(b >= a_ - 1e-4 for a_, b in zip(xs, xs[1:])), f'min step {min(b - a_ for a_, b in zip(xs, xs[1:]))*1000:.2f} mm')

print('--- the question AMCL asks: can a scan be used the moment it arrives?')
ok = tot = 0; scan_lat = 0.13
end = time.time() + 6.0
next_scan = time.time()
pending = []
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.005)
    t = now()
    if time.time() >= next_scan:                # a scan is measured now, arrives in 0.13 s
        pending.append((t + scan_lat, t)); next_scan += 1 / 7.8
    while pending and pending[0][0] <= t:
        _, stamp = pending.pop(0)
        tot += 1
        try:
            buf.lookup_transform('odom', 'base_footprint', Time(seconds=stamp)); ok += 1
        except Exception:
            pass
pct = 100 * ok / tot
check('scans usable on arrival: 100% (measured 44% before)', pct >= 97, f'{pct:.0f}% of {tot}')

print('--- the rover stops')
state['moving'] = False
x_stop = state['x_at_stop'] = state['x_true']
mark = len(arrivals); spin_for(2.0)
a = arrivals[mark:]
final = a[-1][2]
peak = max(x for _, _, x, _ in a)
over = peak - final
settle_err = abs((final - off_cruise) - x_stop)
check('after a stop it settles on the true stopping point (within 1 cm)', settle_err < 0.01, f'{settle_err*100:.2f} cm off')
check('and overshoots by at most the 6 cm cap before it does', over < 0.065, f'overshoot {over*100:.1f} cm (cap = 0.3 s x 0.2 m/s = 6 cm)')

print('--- the ticks stop (ESP32 gone): the transform must go stale, not lie')
state['alive'] = False; spin_for(0.3)
mark = len(arrivals); t0 = now(); spin_for(3.0)
a = arrivals[mark:]
last_after = (a[-1][0] - t0) if a else 0.0
check('keeps going for at most ~1 s after the last tick, then stops', a and last_after <= 1.2 and not any(arr - t0 > 1.3 for arr, *_ in a), f'last TF {last_after:.2f} s into the silence, {len(a)} msgs')
age_now = now() - arrivals[-1][0]
check('and it is stale now (no new transforms)', age_now > 1.5, f'{age_now:.1f} s since the last')

os.killpg(proc.pid, signal.SIGTERM); proc.wait(timeout=10)
print('\nRESULT:', 'ALL PASSED' if not FAILS else f'{len(FAILS)} FAILED: {FAILS}')
rclpy.shutdown()
sys.exit(1 if FAILS else 0)
