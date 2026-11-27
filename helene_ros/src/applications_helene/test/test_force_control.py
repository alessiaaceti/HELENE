"""Offline tests for force_control (no ROS needed).

    cd helene_ros/src/applications_helene && python3 -m pytest test/
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import pytest  # noqa: E402

from applications_helene.force_control import (  # noqa: E402
    Admittance, AdmittanceParams, InsertionParams, ScrewInsertion, Tare,
    max_stable_force_gain, DONE, FAILED, SEARCH)
from applications_helene.insertion_sim import (  # noqa: E402
    StiffSurfaceWithHole, simulate, peak_force)


def test_stable_gain_formula():
    # 20 N/mm, 100 ms -> 0.15 mm/s per N with the default margin
    assert max_stable_force_gain(20000.0, 0.1) == pytest.approx(0.00015)
    # stability limit itself is pi/2
    assert max_stable_force_gain(20000.0, 0.1, margin=math.pi / 2) == pytest.approx(
        math.pi / 2 / 2000.0)


def test_tare_removes_bias():
    t = Tare(n_samples=10)
    for _ in range(10):
        t.add([0.3, -0.2, -2.5])
    assert t.done
    assert t.apply([0.3, -0.2, -0.5]) == pytest.approx([0.0, 0.0, 2.0])


def test_admittance_holds_still_and_follows_push():
    a = Admittance(AdmittanceParams(tare_samples=5, deadband=0.5, damping=400.0))
    bias = [0.1, 0.2, -3.0]
    for _ in range(5):
        assert a.step(bias, 0.01) == [0.0, 0.0, 0.0]
    # nobody touches the sensor -> no motion despite the bias
    assert a.step(bias, 0.01) == [0.0, 0.0, 0.0]
    # steady 5 N push in +x -> moves +x at (5-0.5)/400 m/s
    v = None
    for _ in range(200):
        v = a.step([bias[0] + 5.0, bias[1], bias[2]], 0.01)
    assert v[0] == pytest.approx(4.5 / 400.0, rel=1e-3)
    assert abs(v[1]) < 1e-9 and abs(v[2]) < 1e-9


def test_admittance_speed_is_clamped():
    a = Admittance(AdmittanceParams(tare_samples=1, max_speed=0.02))
    a.step([0, 0, 0], 0.01)
    for _ in range(200):
        v = a.step([0.0, 0.0, 100.0], 0.01)
    assert math.sqrt(sum(c * c for c in v)) == pytest.approx(0.02)


@pytest.mark.parametrize("k_env", [5000.0, 20000.0, 50000.0])
@pytest.mark.parametrize("angle_deg", [0, 90, 200, 315])
@pytest.mark.parametrize("offset", [0.0, 0.0008, 0.002])
def test_finds_hole_with_bounded_force(k_env, angle_deg, offset):
    a = math.radians(angle_deg)
    hole = (offset * math.cos(a), offset * math.sin(a))
    env = StiffSurfaceWithHole(k_env=k_env, hole_xy=hole)
    ctrl, trace = simulate(InsertionParams(expected_surface_z=0.0), env=env,
                           seed=angle_deg)
    assert ctrl.status.state == DONE, ctrl.status.message
    assert peak_force(trace) < 10.0
    hx, hy = ctrl.status.hole_xy
    assert math.hypot(hx - hole[0], hy - hole[1]) < env.r + 4e-4  # estimate lags by ~speed*latency


def test_force_stays_near_target_while_searching():
    # hole far away -> long search; force must stay regulated, never lose
    # contact and never spike
    env = StiffSurfaceWithHole(hole_xy=(0.0035, 0.0))
    ctrl, trace = simulate(env=env)
    search = [r[5] for r in trace if r[1] == SEARCH]
    settled = search[len(search) // 4:-150]  # drop the approach and the hole drop
    assert min(settled) > 1.0
    assert max(settled) < 3.0


def test_naive_high_gain_explodes():
    """Reproduces the 'force explodes' behaviour: gain above the stability
    limit for 20 N/mm and ~100 ms of latency. The force oscillation grows
    until the safety abort trips."""
    p = InsertionParams(force_gain=0.002, max_z_speed=0.02)
    ctrl, trace = simulate(p)
    search = [r[5] for r in trace if r[1] == SEARCH]
    assert ctrl.status.state == FAILED
    assert max(search) > 3 * p.target_force


def test_same_naive_gain_is_fine_without_latency():
    """Same gain, ~10 ms latency: stable. The latency is what kills it."""
    p = InsertionParams(force_gain=0.002, max_z_speed=0.02, expected_surface_z=0.0)
    ctrl, trace = simulate(p, delay=0.01, hw_rate=100.0)
    assert ctrl.status.state == DONE
