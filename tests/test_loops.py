import numpy as np
import pytest

from state_space_studio.design import ackermann, pi_state_feedback, prefilter
from state_space_studio.loops import pi_state_feedback_loop, state_feedback_loop
from state_space_studio.plants import mass_spring_damper, three_tank
from state_space_studio.signals import step, time_grid


@pytest.fixture
def tank_design():
    plant = three_tank()
    K = ackermann(plant.A, plant.B, [-0.02, -0.02, -0.1])
    return plant, K, prefilter(plant.A, plant.B, plant.C, K)


def test_state_feedback_loop_structure(tank_design) -> None:
    plant, K, V = tank_design
    loop = state_feedback_loop(plant, K, V)
    np.testing.assert_allclose(loop.A, plant.A - plant.B @ K)
    assert loop.inputs == ("w", "z")
    assert loop.outputs == ("y", "u")
    assert loop.dc_gain()[0, 0] == pytest.approx(1.0)  # w -> y


def test_state_feedback_offset_matches_the_static_formula(tank_design) -> None:
    plant, K, V = tank_design
    loop = state_feedback_loop(plant, K, V)
    t = time_grid(4000.0, 1.0)
    tr = loop.simulate(t, {"w": 50.0, "z": -5.0})
    x_ss = -np.linalg.solve(loop.A, loop.B @ [50.0, -5.0])
    np.testing.assert_allclose(tr.stack(plant.states)[-1], x_ss, rtol=1e-6)
    assert tr.final("u") == pytest.approx((-K @ x_ss + V[0, 0] * 50.0)[0], rel=1e-6)


def test_pi_loop_has_the_designed_poles_and_rejects_constant_leaks() -> None:
    plant = three_tank()
    poles = [-0.02, -0.02, -0.2, -0.2]
    loop = pi_state_feedback_loop(plant, pi_state_feedback(plant.A, plant.B, plant.C, poles))
    np.testing.assert_allclose(np.poly(loop.A), np.poly(poles), atol=1e-12)
    t = time_grid(3000.0, 1.0)
    tr = loop.simulate(t, {"w": step(t, 0.0, 0.0, 60.0), "z": step(t, 1000.0, 0.0, -5.0)})
    assert tr.final("y") == pytest.approx(60.0, abs=1e-6)
    assert tr.final("e") == pytest.approx(0.0, abs=1e-6)
    assert tr["xi"][-1] > tr.at("xi", 999.0)  # the integrator absorbed the leak


def test_pi_loop_requires_a_strictly_proper_plant() -> None:
    plant = mass_spring_damper()
    gains = pi_state_feedback(plant.A, plant.B, plant.C, [-20, -20, -20])
    proper = type(plant)(**{**plant.__dict__, "D": [[0.5]]})
    with pytest.raises(ValueError, match="strictly proper"):
        pi_state_feedback_loop(proper, gains)
