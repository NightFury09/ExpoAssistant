#!/usr/bin/env python3
"""Measure how smoothly the rover drives: where does the stagger come from?

Records three topics for a while and reports rates, stalls, stop-and-go restarts,
pivoting time and how hard the commands jump, with plain-language pointers.

    python3 tools/drive_profile.py record --seconds 40 --out run1.csv
    python3 tools/drive_profile.py report run1.csv

Do the recording while the rover drives a goal (SET GOAL a few metres away on open
floor, you beside it, hand near EMERGENCY STOP). Record the SAME drive before and
after any tuning change and compare, otherwise you are comparing different rooms.

It only listens. It sends nothing and moves nothing.
"""
import argparse
import csv
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'rover_core'))
from rover_core.drive_metrics import analyze, verdicts  # noqa: E402

TOPICS = {'nav': '/cmd_vel_nav', 'cmd': '/cmd_vel', 'odom': '/odom'}


def record(a):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    rclpy.init()
    n = Node('drive_profile')
    rows = []
    t0 = [None]

    def stamp():
        t = time.monotonic()
        if t0[0] is None:
            t0[0] = t
        return t - t0[0]

    n.create_subscription(Twist, TOPICS['nav'], lambda m: rows.append(
        ('nav', stamp(), m.linear.x, m.angular.z)), 10)
    n.create_subscription(Twist, TOPICS['cmd'], lambda m: rows.append(
        ('cmd', stamp(), m.linear.x, m.angular.z)), 10)
    n.create_subscription(Odometry, TOPICS['odom'], lambda m: rows.append(
        ('odom', stamp(), m.twist.twist.linear.x, m.twist.twist.angular.z)), 10)
    print('recording %.0f s of %s ...' % (a.seconds, ', '.join(TOPICS.values())))
    end = time.time() + a.seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.destroy_node()
    rclpy.shutdown()
    with open(a.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['topic', 't', 'v', 'w'])
        w.writerows(rows)
    print('%d messages -> %s' % (len(rows), a.out))
    return a.out


def load(path):
    ev = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            t, v, w = ev.setdefault(r['topic'], ([], [], []))
            t.append(float(r['t']))
            v.append(float(r['v']))
            w.append(float(r['w']))
    return ev


def report(path):
    m = analyze(load(path))
    print('\n=== %s ===' % path)
    for k, label in (('nav', '/cmd_vel_nav (controller)'), ('cmd', '/cmd_vel (to the base)'),
                     ('odom', '/odom (wheels)')):
        r = m['rates'].get(k)
        print('%-26s %s' % (label, 'no messages' if not r else
                            '%5.1f Hz   worst gap %.2f s' % (r['rate_hz'], r['max_gap_s'])))
    if 'note' in m:
        print('\n' + m['note'])
    else:
        print('\ndriving (asked > %.2f m/s): %.1f s of %.1f s' % (0.05, m['driving_s'], m['seconds']))
        if 'speed_efficiency' in m:
            print('asked %.2f m/s, got %.2f m/s  (%.0f%%)' % (
                m['mean_speed_asked'], m['mean_speed_got'], 100 * m['speed_efficiency']))
            print('stopped while asked to move: %.1f s, %d stall(s), %d restart(s)' % (
                m['stopped_while_asked_s'], m['stalls'], m['restarts']))
        print('pivoting on the spot: %.1f s' % m['pivot_s'])
        if 'cmd_accel_p95' in m:
            print('command acceleration p95 %.2f m/s^2, biggest single step %.2f m/s' % (
                m['cmd_accel_p95'], m['cmd_max_step']))
    print('\nWhat this points at:')
    for line in verdicts(m):
        print('  - ' + line)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('record')
    r.add_argument('--seconds', type=float, default=40.0)
    r.add_argument('--out', required=True)
    p = sub.add_parser('report')
    p.add_argument('csv')
    a = ap.parse_args()
    if a.cmd == 'record':
        report(record(a))
    else:
        report(a.csv)


if __name__ == '__main__':
    main()
