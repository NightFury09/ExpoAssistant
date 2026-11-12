"""The odometry TF predictor, with no ROS and no rover.

    # needs ROS sourced: the module under test imports rclpy
    python3 -m pytest ros2_ws/src/rover_core/test/test_odometry_extrapolation.py
"""
import math

import pytest

from rover_core.rover_odometry import extrapolate


def test_standing_still_stays_put():
    assert extrapolate(1.0, 2.0, 0.5, 0.0, 0.0, 0.3) == (1.0, 2.0, 0.5)


def test_zero_horizon_returns_the_measurement():
    assert extrapolate(1.0, 2.0, 0.5, 0.2, 0.4, 0.0) == (1.0, 2.0, 0.5)


@pytest.mark.parametrize('heading', [0.0, math.pi / 2, math.pi, -2.0])
def test_straight_line_moves_v_times_h_along_the_heading(heading):
    x, y, th = extrapolate(0.0, 0.0, heading, 0.2, 0.0, 0.25)
    assert math.hypot(x, y) == pytest.approx(0.05)
    assert math.atan2(y, x) == pytest.approx(math.atan2(math.sin(heading), math.cos(heading)))
    assert th == pytest.approx(heading)


def test_turning_in_place_changes_only_the_heading():
    x, y, th = extrapolate(1.0, 1.0, 0.0, 0.0, 1.0, 0.2)
    assert (x, y) == (1.0, 1.0)
    assert th == pytest.approx(0.2)


def test_reverse_moves_backwards():
    x, y, _ = extrapolate(0.0, 0.0, 0.0, -0.2, 0.0, 0.5)
    assert x == pytest.approx(-0.1) and y == pytest.approx(0.0)


def test_steps_compose_like_one_long_step():
    # Two 0.1 s predictions along a constant arc land where one 0.2 s prediction
    # does, to within the midpoint rule's second-order error (~0.01 mm here), so
    # stamping the transform more often does not change where it points.
    v, w = 0.2, 0.5
    one = extrapolate(0.0, 0.0, 0.0, v, w, 0.2)
    a = extrapolate(0.0, 0.0, 0.0, v, w, 0.1)
    two = extrapolate(*a, v, w, 0.1)
    assert two == pytest.approx(one, abs=1e-4)
