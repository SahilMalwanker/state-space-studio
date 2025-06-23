import numpy as np
import pytest
from scipy import signal

from state_space_studio.lti import LTISystem, discretize, simulate
from state_space_studio.signals import ramp, step, time_grid


def _random_stable_system(rng: np.random.Generator, n: int = 4, m: int = 2, p: int = 2) -> LTISystem:
    A = rng.normal(size=(n, n))
    A -= (np.max(np.linalg.eigvals(A).real) + 0.5) * np.eye(n)
    return LTISystem(
        A,
        rng.normal(size=(n, m)),
        rng.normal(size=(p, n)),
        rng.normal(size=(p, m)),
        states=tuple(f"x{i}" for i in range(n)),
        inputs=tuple(f"u{i}" for i in range(m)),
        outputs=tuple(f"y{i}" for i in range(p)),
    )


@pytest.mark.parametrize(("hold", "interp"), [("zoh", False), ("foh", True)])
def test_simulation_matches_scipy_lsim(hold: str, interp: bool) -> None:
    rng = np.random.default_rng(7)
    system = _random_stable_system(rng)
    t = time_grid(5.0, 0.01)
    u = np.column_stack([np.sin(3 * t), step(t, 1.0, 0.0, 2.0)])
    x0 = rng.normal(size=system.n)

    ours = simulate(system, t, u, x0, hold=hold)
    _, y_ref, x_ref = signal.lsim((system.A, system.B, system.C, system.D), u, t, X0=x0, interp=interp)
    np.testing.assert_allclose(ours.stack(system.states), x_ref, atol=1e-9)
    np.testing.assert_allclose(ours.stack(system.outputs), y_ref, atol=1e-9)


def test_zoh_is_exact_for_a_first_order_step() -> None:
    tau = 2.0
    system = LTISystem([[-1 / tau]], [[1 / tau]], [[1.0]], [[0.0]], ("x",), ("u",), ("y",))
    t = time_grid(10.0, 0.5)
    tr = system.simulate(t, {"u": 1.0})
    np.testing.assert_allclose(tr["y"], 1 - np.exp(-t / tau), atol=1e-12)


def test_foh_is_exact_for_a_ramp() -> None:
    system = LTISystem([[-1.0]], [[1.0]], [[1.0]], [[0.0]], ("x",), ("u",), ("y",))
    t = time_grid(4.0, 0.25)
    tr = system.simulate(t, {"u": ramp(t)}, hold="foh")
    np.testing.assert_allclose(tr["x"], t - 1 + np.exp(-t), atol=1e-12)


def test_discretize_reproduces_the_matrix_exponential() -> None:
    A = np.array([[0.0, 1.0], [-4.0, -0.4]])
    Ad, B0, B1 = discretize(A, [[0.0], [1.0]], 0.1)
    w = np.linalg.eigvals(Ad)
    np.testing.assert_allclose(np.sort(np.abs(w)), np.exp(-0.2 * 0.1) * np.ones(2), rtol=1e-12)
    assert np.all(B1 == 0)
    assert B0.shape == (2, 1)


def test_named_inputs_and_trajectory_helpers() -> None:
    system = LTISystem(
        np.diag([-1.0, -2.0]), np.eye(2), np.eye(2), np.zeros((2, 2)), ("a", "b"), ("ua", "ub"), ("ya", "yb")
    )
    t = time_grid(1.0, 0.1)
    tr = system.simulate(t, {"ub": 2.0})
    assert np.all(tr["ua"] == 0)
    assert np.all(tr["ub"] == 2.0)
    assert "a" in tr
    assert "missing" not in tr
    assert tr.stack(["a", "b"]).shape == (t.size, 2)
    assert tr.final("b") == pytest.approx(1 - np.exp(-2.0), abs=1e-12)
    assert tr.at("b", 0.05) == pytest.approx(0.5 * (tr["b"][0] + tr["b"][1]))
    with pytest.raises(KeyError, match="unknown signal"):
        tr["nope"]


def test_validation_errors() -> None:
    with pytest.raises(ValueError, match="square"):
        LTISystem(np.ones((2, 3)), np.ones((2, 1)), np.ones((1, 3)), [[0.0]], ("a", "b"), ("u",), ("y",))
    with pytest.raises(ValueError, match="unique"):
        LTISystem([[-1.0]], [[1.0]], [[1.0]], [[0.0]], ("x",), ("x",), ("y",))
    system = LTISystem([[-1.0]], [[1.0]], [[1.0]], [[0.0]], ("x",), ("u",), ("y",))
    with pytest.raises(ValueError, match="uniformly"):
        system.simulate([0.0, 0.1, 0.3])
    with pytest.raises(KeyError, match="unknown inputs"):
        system.simulate(time_grid(1.0, 0.1), {"v": 1.0})
    with pytest.raises(ValueError, match="x0"):
        system.simulate(time_grid(1.0, 0.1), x0=[1.0, 2.0])
    with pytest.raises(ValueError, match="hold"):
        system.simulate(time_grid(1.0, 0.1), hold="cubic")
