"""The studies reproduce the lab experiments; these tests pin their key results."""

import numpy as np
import pytest

from state_space_studio.scenarios import (
    DRIVETRAIN_CASES,
    drivetrain_study,
    fundamentals_study,
    lqr_weight_sweep,
    three_tank_study,
)


def test_fundamental_responses_settle_where_physics_says() -> None:
    s = fundamentals_study()
    assert s.rc_step.final("v3") == pytest.approx(1.0, abs=1e-3)
    assert max(abs(s.rc_free.final(n)) for n in s.rc.states) < 1e-3
    assert s.msd_step.final("x") == pytest.approx(0.25, abs=2e-4)  # u / c
    for tr in s.msd_portrait:
        assert np.hypot(tr.final("x") / 0.3, tr.final("v") / 3.0) < 1e-3
    assert s.mimo_ramp.final("x1") == pytest.approx(4 / 3 * (8.0 - 1 / 3), abs=1e-3)


def test_drivetrain_closed_loop_reaches_the_analytic_steady_state() -> None:
    s = drivetrain_study()
    angles = ["phi1", "phi2", "phi3"]
    for case in DRIVETRAIN_CASES:
        tr = s.cases[case.key]
        u = np.array([case.w, *case.Tz])
        x_ss = -np.linalg.solve(s.closed_loop.A, s.closed_loop.B @ u)
        # after 1 s the double pole at -10 has decayed to (1 + 10 t) e^(-10 t) ~ 5e-4
        np.testing.assert_allclose(tr.stack(angles)[-1], x_ss[:3], rtol=2e-3, atol=2e-3)


def test_drivetrain_reference_tracking_and_load_offsets() -> None:
    s = drivetrain_study()
    assert s.tracking.overshoot_pct < 0.1
    assert 0.55 < s.tracking.settling_time < 0.7  # double pole at -10: 5.83 / 10 s
    load_1 = s.cases["load_1"]
    assert load_1.final("phi3") == pytest.approx(1 / s.V, rel=2e-3)  # 1 N m / virtual spring V
    assert s.cases["load_3"].final("phi3") == pytest.approx(-36.55, abs=0.05)
    assert s.cases["load_23"].final("u") == pytest.approx(-1.5, abs=1e-3)


def test_open_loop_drivetrain_spins_up_under_constant_torque() -> None:
    s = drivetrain_study()
    omega_ss = 10.0 / sum((8.0e-3, 7.5e-3, 7.5e-3))
    assert s.torque_step.final("omega3") == pytest.approx(omega_ss, rel=0.02)
    assert s.torque_step.final("phi3") > 150.0


def test_three_tank_controller_comparison() -> None:
    runs = three_tank_study().runs
    assert runs["pi"].offset_after_leak == pytest.approx(0.0, abs=0.01)
    assert runs["pole_placement"].offset_after_leak == pytest.approx(46.75, abs=0.05)
    assert runs["lqr_identity"].offset_after_leak == pytest.approx(6.32, abs=0.05)
    assert runs["lqr_tuned"].offset_after_leak == pytest.approx(3.84, abs=0.05)
    assert runs["pole_placement"].min_u < 0.0  # asks the pump to run backwards
    assert runs["lqr_tuned"].peak_u > 100.0  # saturates the pump
    for run in runs.values():
        assert abs(run.offset_before_leak) < 0.05
        assert run.tracking.overshoot_pct < 0.5


def test_settling_time_rule_of_thumb_is_optimistic_for_critical_damping() -> None:
    run = three_tank_study().runs["pole_placement"]
    assert 290.0 < run.tracking.settling_time < 305.0  # rule of thumb promised 200 s


def test_open_loop_tank_profile_steady_states() -> None:
    s = three_tank_study()
    gain = s.plant.open_loop().dc_gain()[0]
    assert s.open_loop_profile.at("y", 1499.0) == pytest.approx(10 * gain[0], abs=0.1)
    assert s.open_loop_profile.final("y") == pytest.approx(25 * gain[0] - 5 * gain[1], abs=0.05)


def test_lqr_sweep_trades_speed_for_effort() -> None:
    sweep = lqr_weight_sweep(np.array([0.1, 1.0, 10.0]))
    settling = [r.tracking.settling_time for r in sweep]
    effort = [r.peak_u for r in sweep]
    droop = [r.offset_after_leak for r in sweep]
    assert settling == sorted(settling)
    assert effort == sorted(effort, reverse=True)
    assert droop == sorted(droop)
