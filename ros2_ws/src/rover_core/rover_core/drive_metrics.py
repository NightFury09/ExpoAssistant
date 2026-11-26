"""Put numbers on "the drive is staggered". No ROS.

tools/drive_profile.py records /cmd_vel_nav (what the controller asks for),
/cmd_vel (what reaches the base, after the velocity smoother) and /odom (what the
wheels did) during a run and hands the events here. The point is to stop guessing
which link in controller -> smoother -> ESP32 -> wheels -> odometry is the
stagger: each one leaves a different fingerprint.

    slow odometry          rate_hz of /odom is low or has long gaps
    controller starved     /cmd_vel_nav gaps well over its 100 ms period
    unsmoothed commands    large jumps between consecutive /cmd_vel messages
    stop-and-go            the rover stops while a goal is still being driven
    pivoting               time spent rotating on the spot
"""
import math

import numpy as np

MOVING = 0.03          # m/s: below this the rover is "stopped"
COMMANDED = 0.05       # m/s: the controller is asking it to move
STALL_MIN_S = 0.4      # stopped this long while asked to move = a stall
PIVOT_W = 0.2          # rad/s


def rate_stats(t):
    """Message rate and worst gap of one topic from its arrival times."""
    t = np.sort(np.asarray(t, float))
    if t.size < 2:
        return {'n': int(t.size), 'rate_hz': 0.0, 'max_gap_s': math.inf}
    dt = np.diff(t)
    return {'n': int(t.size), 'rate_hz': float((t.size - 1) / (t[-1] - t[0])),
            'max_gap_s': float(dt.max()), 'p95_gap_s': float(np.percentile(dt, 95))}


def hold(t_events, values, grid):
    """Zero-order hold of an event stream onto a time grid (value before the first
    event is that event's)."""
    t_events, values = np.asarray(t_events, float), np.asarray(values, float)
    idx = np.searchsorted(t_events, grid, side='right') - 1
    return values[np.clip(idx, 0, len(values) - 1)]


def analyze(ev, hz=50.0):
    """ev: {'nav': (t, v, w), 'cmd': (t, v, w), 'odom': (t, v, w)}, each a tuple of
    equal-length sequences; any may be missing. Returns a dict of metrics."""
    out = {'rates': {k: rate_stats(v[0]) for k, v in ev.items() if len(v[0])}}
    if 'odom' not in ev or 'nav' not in ev or len(ev['nav'][0]) < 2 or len(ev['odom'][0]) < 2:
        out['note'] = 'need /cmd_vel_nav and /odom to judge the drive'
        return out
    t0 = max(ev['nav'][0][0], ev['odom'][0][0])
    t1 = min(ev['nav'][0][-1], ev['odom'][0][-1])
    if t1 - t0 < 1.0:
        out['note'] = 'under a second of overlapping data'
        return out
    grid = np.arange(t0, t1, 1.0 / hz)
    nav_v = hold(ev['nav'][0], ev['nav'][1], grid)
    nav_w = hold(ev['nav'][0], ev['nav'][2], grid)
    odo_v = np.abs(hold(ev['odom'][0], ev['odom'][1], grid))
    odo_w = np.abs(hold(ev['odom'][0], ev['odom'][2], grid))
    asked = nav_v > COMMANDED

    out['seconds'] = float(t1 - t0)
    out['driving_s'] = float(asked.sum() / hz)
    if asked.any():
        out['mean_speed_asked'] = float(nav_v[asked].mean())
        out['mean_speed_got'] = float(odo_v[asked].mean())
        out['speed_efficiency'] = float(odo_v[asked].mean() / max(1e-6, nav_v[asked].mean()))
        out['stopped_while_asked_s'] = float(((odo_v < MOVING) & asked).sum() / hz)

    # stalls: runs of "asked to move but stopped" at least STALL_MIN_S long
    stalled = (odo_v < MOVING) & asked
    runs, start = [], None
    for i, s in enumerate(stalled):
        if s and start is None:
            start = i
        if (not s or i == len(stalled) - 1) and start is not None:
            if (i - start) / hz >= STALL_MIN_S:
                runs.append((start / hz, (i - start) / hz))
            start = None
    out['stalls'] = len(runs)
    out['stall_total_s'] = float(sum(d for _, d in runs))
    # stop-go cycles: it was stopped, then moved off again, with the goal still on
    moving = odo_v >= MOVING
    starts = int(((~moving[:-1]) & moving[1:] & asked[1:]).sum())
    out['restarts'] = starts

    pivot = (odo_w > PIVOT_W) & (odo_v < MOVING)
    out['pivot_s'] = float(pivot.sum() / hz)
    out['nav_pivot_s'] = float(((np.abs(nav_w) > PIVOT_W) & (nav_v < MOVING)).sum() / hz)

    if 'cmd' in ev and len(ev['cmd'][0]) > 3:
        t, v = np.asarray(ev['cmd'][0], float), np.asarray(ev['cmd'][1], float)
        dt = np.maximum(np.diff(t), 1e-3)
        acc = np.diff(v) / dt
        out['cmd_accel_p95'] = float(np.percentile(np.abs(acc), 95))
        out['cmd_max_step'] = float(np.abs(np.diff(v)).max())
    return out


def verdicts(m):
    """Plain-language lines: what the numbers point at."""
    lines = []
    r = m.get('rates', {})
    od = r.get('odom')
    if od and od['rate_hz'] < 10:
        lines.append('odometry arrives at %.1f Hz (worst gap %.2f s): every controller '
                     'decision uses a pose that old. Raising this is the biggest single '
                     'smoothness lever, and it is a firmware/serial job.'
                     % (od['rate_hz'], od['max_gap_s']))
    nv = r.get('nav')
    if nv and nv['max_gap_s'] > 0.25:
        lines.append('the controller went quiet for %.2f s (expected ~0.1 s): it is being '
                     'starved, by a slow costmap or a busy CPU.' % nv['max_gap_s'])
    if m.get('speed_efficiency') is not None and m['speed_efficiency'] < 0.6:
        lines.append('it moved at %.0f%% of the speed it was asked for.'
                     % (100 * m['speed_efficiency']))
    if m.get('stalls'):
        lines.append('%d stall(s), %.1f s in total, stopped while a goal was still being '
                     'driven.' % (m['stalls'], m['stall_total_s']))
    if m.get('restarts', 0) >= 3:
        lines.append('%d stop-and-go restarts in %.0f s.' % (m['restarts'], m.get('seconds', 0)))
    if m.get('pivot_s', 0) > 0.15 * m.get('seconds', 1):
        lines.append('%.0f%% of the run was spent pivoting on the spot.'
                     % (100 * m['pivot_s'] / m['seconds']))
    if m.get('cmd_max_step', 0) > 0.12:
        lines.append('commands jump by %.2f m/s between messages: the smoother is not '
                     'limiting them (is it running, and is the base reading /cmd_vel?).'
                     % m['cmd_max_step'])
    if not lines:
        lines.append('nothing stands out in this run.')
    return lines
