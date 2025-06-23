import numpy as np
import pytest

from state_space_studio.design import (
    ackermann,
    augment_with_integrator,
    characteristic_polynomial,
    lqr,
    pi_state_feedback,
    prefilter,
    second_order_poles,
)
from state_space_studio.plants import drivetrain, three_tank


def _controllable(rng: np.random.Generator, n: int) -> tuple[np.ndarray, np.ndarray]:
    while True:
        A, B = rng.normal(size=(n, n)), rng.normal(size=(n, 1))
        if np.linalg.cond(np.hstack([np.linalg.matrix_power(A, k) @ B for k in range(n)])) < 1e6:
            return A, B


@pytest.mark.parametrize("seed", range(5))
def test_ackermann_places_distinct_poles(seed: int) -> None:
    rng = np.random.default_rng(seed)
    A, B = _controllable(rng, 4)
    desired = np.array([-1.0, -2.0, -3 + 1j, -3 - 1j])
    K = ackermann(A, B, desired)
    placed = np.linalg.eigvals(A - B @ K)
    np.testing.assert_allclose(np.sort_complex(placed), np.sort_complex(desired), atol=1e-7)


def test_ackermann_handles_the_repeated_drivetrain_poles() -> None:
    plant = drivetrain()
    poles = [-10, -10, -100, -100, -100, -100]
    K = ackermann(plant.A, plant.B, poles)
    achieved = np.poly(plant.A - plant.B @ K)
    np.testing.assert_allclose(achieved, np.poly(poles), rtol=1e-9)


def test_ackermann_rejects_uncontrollable_pairs() -> None:
    with pytest.raises(ValueError, match="not controllable"):
        ackermann(np.diag([-1.0, -2.0]), [[1.0], [0.0]], [-3.0, -4.0])
    with pytest.raises(ValueError, match="expected 2 poles"):
        ackermann(np.diag([-1.0, -2.0]), [[1.0], [1.0]], [-3.0])


def test_characteristic_polynomial_requires_conjugate_pairs() -> None:
    np.testing.assert_allclose(characteristic_polynomial([-1 + 2j, -1 - 2j]), [1, 2, 5])
    with pytest.raises(ValueError, match="conjugate"):
        characteristic_polynomial([-1 + 2j, -3.0])


def test_second_order_poles_follow_the_settling_time_rule() -> None:
    np.testing.assert_allclose(second_order_poles(200.0, 1.0), [-0.02, -0.02])
    poles = second_order_poles(2.0, 0.5)
    np.testing.assert_allclose(poles.real, [-2.0, -2.0])
    np.testing.assert_allclose(np.abs(poles), [4.0, 4.0])


def test_prefilter_gives_unit_dc_gain() -> None:
    plant = three_tank()
    K = ackermann(plant.A, plant.B, [-0.02, -0.02, -0.1])
    V = prefilter(plant.A, plant.B, plant.C, K)
    closed = plant.A - plant.B @ K
    dc = -(plant.C @ np.linalg.solve(closed, plant.B @ V))
    assert dc[0, 0] == pytest.approx(1.0)


def test_drivetrain_prefilter_matches_the_original_lab_value() -> None:
    plant = drivetrain()
    K = ackermann(plant.A, plant.B, [-10, -10, -100, -100, -100, -100])
    V = prefilter(plant.A, plant.B, plant.C, K)[0, 0]
    assert pytest.approx(1.0667, abs=5e-5) == V
    # rigid-body mode: the angle gains act as one virtual spring of stiffness V
    assert K[0, :3].sum() == pytest.approx(V)


def test_pi_state_feedback_places_the_augmented_poles() -> None:
    plant = three_tank()
    poles = np.array([-0.02, -0.02, -0.2, -0.2])
    gains = pi_state_feedback(plant.A, plant.B, plant.C, poles, alpha=1.0)
    A_aug, B_aug = augment_with_integrator(plant.A, plant.B, plant.C)
    np.testing.assert_allclose(np.poly(A_aug - B_aug @ gains.K_aug), np.poly(poles), atol=1e-12)
    np.testing.assert_allclose(gains.Kpi + gains.Kp * plant.C, gains.Kx)
    assert gains.Kp * plant.open_loop().dc_gain()[0, 0] == pytest.approx(1.0)
    assert gains.Ki > 0


def test_lqr_solves_the_riccati_equation() -> None:
    plant = three_tank()
    Q, R = np.diag([10.0, 1.0, 10.0]), np.array([[5.0]])
    design = lqr(plant.A, plant.B, Q, R)
    A, B, P = plant.A, plant.B, design.P
    residual = A.T @ P + P @ A - P @ B @ np.linalg.solve(R, B.T @ P) + Q
    np.testing.assert_allclose(residual, 0.0, atol=1e-9)
    np.testing.assert_allclose(P, P.T, atol=1e-12)
    assert np.all(np.linalg.eigvalsh(P) > 0)
    np.testing.assert_allclose(design.K, np.linalg.solve(R, B.T @ P))
    assert np.all(design.poles.real < 0)


def test_lqr_matches_the_scalar_closed_form() -> None:
    a, b, q, r = 1.0, 2.0, 3.0, 0.5
    design = lqr([[a]], [[b]], [[q]], [[r]])
    expected = (a + np.sqrt(a**2 + b**2 * q / r)) / b
    assert design.K[0, 0] == pytest.approx(expected)
