"""The drive-smoothness metrics, on synthetic runs. No ROS.

    python3 -m pytest ros2_ws/src/rover_core/test/test_drive_metrics.py
"""
import numpy as np

from rover_core.drive_metrics import analyze, rate_stats, verdicts


def run(speed_fn, secs=20.0, odom_hz=4.0, nav_hz=10.0, cmd_hz=20.0, nav_v=0.22):
    """A drive: the controller asks nav_v throughout; the wheels follow speed_fn(t)."""
    tn = np.arange(0, secs, 1 / nav_hz)
    tc = np.arange(0, secs, 1 / cmd_hz)
    to = np.arange(0, secs, 1 / odom_hz)
    return {'nav': (tn, np.full_like(tn, nav_v), np.zeros_like(tn)),
            'cmd': (tc, np.array([speed_fn(t) for t in tc]), np.zeros_like(tc)),
            'odom': (to, np.array([speed_fn(t) for t in to]), np.zeros_like(to))}


def smooth(t):
    return min(0.22, 0.5 * t)


def stop_and_go(t):                      # 2 s moving, 1 s stopped, repeat
    return 0.22 if (t % 3.0) < 2.0 else 0.0


def test_rates_are_measured():
    r = rate_stats(np.arange(0, 10, 0.25))
    assert abs(r['rate_hz'] - 4.0) < 1e-6 and abs(r['max_gap_s'] - 0.25) < 1e-9


def test_a_smooth_drive_has_no_stalls_and_good_efficiency():
    m = analyze(run(smooth))
    assert m['stalls'] == 0
    assert m['speed_efficiency'] > 0.9
    assert m['cmd_max_step'] < 0.05


def test_stop_and_go_shows_up_as_stalls_and_restarts():
    m = analyze(run(stop_and_go))
    assert m['stalls'] >= 5
    assert m['restarts'] >= 5
    assert m['speed_efficiency'] < 0.8
    assert any('stall' in line for line in verdicts(m))


def test_slow_odometry_is_called_out():
    m = analyze(run(smooth, odom_hz=4.0))
    assert any('4.0 Hz' in line or 'odometry arrives' in line for line in verdicts(m))
    fast = analyze(run(smooth, odom_hz=20.0))
    assert not any('odometry arrives' in line for line in verdicts(fast))


def test_unsmoothed_command_steps_are_called_out():
    m = analyze(run(stop_and_go))
    assert m['cmd_max_step'] > 0.2
    assert any('jump' in line for line in verdicts(m))


def test_a_quiet_controller_is_called_out():
    ev = run(smooth)
    t, v, w = ev['nav']
    keep = (t < 8) | (t > 9.0)                       # a one-second hole
    ev['nav'] = (t[keep], v[keep], w[keep])
    assert any('went quiet' in line for line in verdicts(analyze(ev)))


def test_too_little_data_is_reported_not_crashed():
    m = analyze({'odom': (np.array([0.0, 0.2]), np.array([0.0, 0.0]), np.array([0.0, 0.0]))})
    assert 'note' in m and verdicts(m)


def test_pivoting_is_counted():
    ev = run(lambda t: 0.0, secs=10.0)
    to, tn = ev['odom'][0], ev['nav'][0]
    ev['odom'] = (to, np.zeros_like(to), np.full_like(to, 0.5))
    ev['nav'] = (tn, np.zeros_like(tn), np.full_like(tn, 0.5))
    m = analyze(ev)
    assert m['pivot_s'] > 8.0
