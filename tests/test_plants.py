import numpy as np
import pytest

from state_space_studio.analysis import structural_report
from state_space_studio.plants import (
    PLANTS,
    DrivetrainParams,
    coupled_mimo,
    drivetrain,
    mass_spring_damper,
    rc_ladder,
    three_tank,
)


@pytest.mark.parametrize("key", list(PLANTS))
def test_every_plant_is_controllable_and_observable(key: str) -> None:
    plant = PLANTS[key]()
    report = structural_report(plant.A, plant.B, plant.C)
    assert report.controllable
    assert report.observable
    assert plant.open_loop().inputs == (*plant.inputs, *plant.disturbances)
    for name in (*plant.states, *plant.inputs, *plant.outputs):
        assert plant.label(name)


def test_rc_ladder_matches_the_circuit_equations() -> None:
    plant = rc_ladder()
    expected = np.array([[-0.5, 0.25, 0.0], [0.25, -0.5, 0.25], [0.0, 0.25, -0.25]])
    np.testing.assert_allclose(plant.A, expected)
    assert plant.open_loop().dc_gain()[0, 0] == pytest.approx(1.0)  # capacitors charge to u


def test_coupled_system_has_the_documented_poles() -> None:
    poles = coupled_mimo().open_loop().poles()
    np.testing.assert_allclose(poles, [-3.0, -1 - 4j, -1 + 4j], atol=1e-12)


def test_mass_spring_damper_is_overdamped() -> None:
    plant = mass_spring_damper()
    poles = plant.open_loop().poles()
    np.testing.assert_allclose(poles.real, [-10 - np.sqrt(20), -10 + np.sqrt(20)], rtol=1e-12)
    assert plant.open_loop().dc_gain()[0, 0] == pytest.approx(1 / 4.0)


def test_drivetrain_matches_the_explicit_lab_matrices() -> None:
    p = DrivetrainParams()
    (J1, J2, J3), (d1, d2, d3) = p.J, p.d
    k12, k23 = p.k12, p.k23
    A = np.array(
        [
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
            [-k12 / J1, k12 / J1, 0, -d1 / J1, 0, 0],
            [k12 / J2, -k12 / J2 - k23 / J2, k23 / J2, 0, -d2 / J2, 0],
            [0, k23 / J3, -k23 / J3, 0, 0, -d3 / J3],
        ]
    )
    plant = drivetrain()
    np.testing.assert_allclose(plant.A, A)
    np.testing.assert_allclose(plant.B.ravel(), [0, 0, 0, 1 / J1, 0, 0])
    np.testing.assert_allclose(plant.E[3:], np.diag([1 / J1, 1 / J2, 1 / J3]))


def test_drivetrain_has_one_rigid_body_mode() -> None:
    plant = drivetrain()
    np.testing.assert_allclose(plant.A @ [1, 1, 1, 0, 0, 0], 0.0, atol=1e-12)
    report = structural_report(plant.A, plant.B, plant.C)
    assert report.stability == "marginally stable"
    assert np.sum(np.abs(report.poles) < 1e-9) == 1
    damped = sorted(abs(p.imag) for p in report.poles if p.imag > 0)
    np.testing.assert_allclose(damped, [46.59, 103.98], atol=0.01)


def test_three_tank_only_loses_liquid_through_the_drains() -> None:
    plant = three_tank()
    # column sums give d/dt(h1 + h2 + h3) per unit level: the valves only move liquid around
    np.testing.assert_allclose(plant.A.sum(axis=0), [-0.002, -0.004, -0.006], atol=1e-15)
    assert structural_report(plant.A, plant.B, plant.C).stability == "asymptotically stable"
    assert plant.open_loop().dc_gain()[0, 0] == pytest.approx(2.6747, abs=1e-4)
