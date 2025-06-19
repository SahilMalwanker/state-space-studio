import numpy as np
import pytest

from state_space_studio.analysis import (
    classify_stability,
    controllability_matrix,
    modes,
    observability_matrix,
    step_metrics,
    structural_report,
)
from state_space_studio.lti import LTISystem
from state_space_studio.signals import time_grid


def test_kalman_matrices_have_the_expected_blocks() -> None:
    A = np.array([[0.0, 1.0], [-2.0, -3.0]])
    B = np.array([[0.0], [1.0]])
    C = np.array([[1.0, 0.0]])
    np.testing.assert_allclose(controllability_matrix(A, B), np.hstack([B, A @ B]))
    np.testing.assert_allclose(observability_matrix(A, C), np.vstack([C, C @ A]))


def test_uncontrollable_mode_is_detected() -> None:
    A = np.diag([-1.0, -2.0])
    report = structural_report(A, [[1.0], [0.0]], [[1.0, 1.0]])
    assert report.controllability_rank == 1
    assert not report.controllable
    assert report.observable


@pytest.mark.parametrize(
    ("poles", "expected"),
    [
        ([-1.0, -2 + 1j, -2 - 1j], "asymptotically stable"),
        ([0.0, -1.0], "marginally stable"),
        ([2j, -2j, -1.0], "marginally stable"),
        ([0.0, 0.0], "undetermined (repeated imaginary-axis poles)"),
        ([2j, 2j, -2j, -2j], "undetermined (repeated imaginary-axis poles)"),
        ([0.0, 0.0, 0.1], "unstable"),
        ([0.1, -1.0], "unstable"),
    ],
)
def test_classify_stability(poles: list[complex], expected: str) -> None:
    assert classify_stability(poles) == expected


def test_modes_report_frequency_and_damping() -> None:
    (mode,) = modes([-3 + 4j, -3 - 4j])
    assert mode.natural_frequency == pytest.approx(5.0)
    assert mode.damping_ratio == pytest.approx(0.6)
    assert mode.time_constant == pytest.approx(1 / 3)


def test_step_metrics_of_a_first_order_lag() -> None:
    tau = 2.0
    t = time_grid(30.0, 0.001)
    y = 1 - np.exp(-t / tau)
    m = step_metrics(t, y, target=1.0)
    assert m.rise_time == pytest.approx(tau * np.log(9), abs=2e-3)
    assert m.settling_time == pytest.approx(tau * np.log(50), abs=5e-3)  # e^(-t/tau) = 2 %
    assert m.overshoot_pct == 0.0
    assert m.steady_state_error == pytest.approx(0.0, abs=1e-6)


def test_step_metrics_overshoot_of_a_second_order_system() -> None:
    zeta, wn = 0.5, 2.0
    system = LTISystem(
        [[0.0, 1.0], [-(wn**2), -2 * zeta * wn]],
        [[0.0], [wn**2]],
        [[1.0, 0.0]],
        [[0.0]],
        ("x", "v"),
        ("u",),
        ("y",),
    )
    t = time_grid(20.0, 0.001)
    tr = system.simulate(t, {"u": 1.0})
    m = step_metrics(t, tr["y"], target=1.0)
    assert m.overshoot_pct == pytest.approx(100 * np.exp(-np.pi * zeta / np.sqrt(1 - zeta**2)), abs=0.05)
    assert m.peak == pytest.approx(1 + m.overshoot_pct / 100, abs=1e-3)


def test_step_metrics_window_and_flat_signal() -> None:
    t = time_grid(10.0, 0.01)
    y = np.where(t >= 5.0, 1 - np.exp(-(t - 5.0)), 0.0)
    m = step_metrics(t, y, target=2.0, t_step=5.0)
    y_end = 1 - np.exp(-5.0)  # rise time is relative to the response's own final value
    assert m.rise_time == pytest.approx(np.log((1 - 0.1 * y_end) / (1 - 0.9 * y_end)), abs=0.02)
    assert m.steady_state_error == pytest.approx(2.0 - y_end)
    flat = step_metrics(t, np.ones_like(t), target=1.0)
    assert np.isnan(flat.rise_time)
    assert flat.steady_state_error == 0.0
