"""Controller synthesis: pole placement, prefilters, PI state feedback and LQR."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike
from scipy.linalg import solve_continuous_are

from state_space_studio.analysis import controllability_matrix
from state_space_studio.lti import ComplexArray, FloatArray

__all__ = [
    "LQRDesign",
    "PIStateFeedback",
    "ackermann",
    "augment_with_integrator",
    "characteristic_polynomial",
    "lqr",
    "pi_state_feedback",
    "prefilter",
    "second_order_poles",
]


def characteristic_polynomial(poles: ArrayLike) -> FloatArray:
    """Real coefficients ``[1, a_(n-1), ..., a_0]`` of ``prod(s - p_i)``."""
    coeffs = np.poly(np.asarray(poles, dtype=complex))
    if np.max(np.abs(coeffs.imag), initial=0.0) > 1e-9 * np.max(np.abs(coeffs)):
        raise ValueError("complex poles must come in conjugate pairs")
    return coeffs.real


def second_order_poles(settling_time: float, damping_ratio: float, criterion: float = 4.0) -> ComplexArray:
    """Dominant pole pair from a settling-time target.

    Uses the rule of thumb ``t_s = criterion / (zeta * omega_n)`` (``criterion = 4`` for a
    2 % band) and returns the roots of ``s^2 + 2 zeta omega_n s + omega_n^2``.
    """
    if settling_time <= 0 or damping_ratio <= 0:
        raise ValueError("settling_time and damping_ratio must be positive")
    wn = criterion / (damping_ratio * settling_time)
    sigma = damping_ratio * wn
    if damping_ratio >= 1.0:  # real (possibly repeated) roots, computed without round-off
        spread = wn * np.sqrt(damping_ratio**2 - 1.0)
        return np.array([-sigma - spread, -sigma + spread], dtype=complex)
    wd = wn * np.sqrt(1.0 - damping_ratio**2)
    return np.array([-sigma - 1j * wd, -sigma + 1j * wd])


def ackermann(A: ArrayLike, B: ArrayLike, poles: ArrayLike) -> FloatArray:
    """Single-input pole placement with Ackermann's formula.

    ``K = e_n^T Q_c^{-1} phi(A)``, where ``phi`` is the desired characteristic polynomial,
    so that ``eig(A - B K)`` equals ``poles`` (repeated poles are allowed). Returns a
    ``(1, n)`` gain.
    """
    A = np.atleast_2d(np.asarray(A, dtype=float))
    B = np.asarray(B, dtype=float).reshape(-1, 1)
    n = A.shape[0]
    if B.shape[0] != n:
        raise ValueError("B must be an n x 1 column for Ackermann's formula")
    if len(np.atleast_1d(poles)) != n:
        raise ValueError(f"expected {n} poles, got {len(np.atleast_1d(poles))}")
    Qc = controllability_matrix(A, B)
    if np.linalg.matrix_rank(Qc) < n:
        raise ValueError("(A, B) is not controllable; poles cannot be placed arbitrarily")

    phi = np.zeros_like(A)
    for coefficient in characteristic_polynomial(poles):  # Horner scheme
        phi = phi @ A + coefficient * np.eye(n)
    e_n = np.zeros(n)
    e_n[-1] = 1.0
    last_row_of_inverse = np.linalg.solve(Qc.T, e_n)
    return (last_row_of_inverse @ phi).reshape(1, n)


def prefilter(A: ArrayLike, B: ArrayLike, C: ArrayLike, K: ArrayLike, D: ArrayLike = 0.0) -> FloatArray:
    """Static prefilter ``V`` for unit DC gain of ``u = V w - K x``.

    ``V = [ (C - D K) (-(A - B K))^{-1} B + D ]^{-1}``, which reduces to the textbook
    ``V = -(C (A - B K)^{-1} B)^{-1}`` for ``D = 0``.
    """
    A, B, C, K = (np.atleast_2d(np.asarray(v, dtype=float)) for v in (A, B, C, K))
    D = np.broadcast_to(np.asarray(D, dtype=float), (C.shape[0], B.shape[1]))
    closed = A - B @ K
    dc_gain = (C - D @ K) @ np.linalg.solve(-closed, B) + D
    return np.linalg.inv(dc_gain)


def augment_with_integrator(A: ArrayLike, B: ArrayLike, C: ArrayLike) -> tuple[FloatArray, FloatArray]:
    """Plant plus the integrator state ``x_i' = w - y`` (with ``w = 0``)."""
    A, B, C = (np.atleast_2d(np.asarray(v, dtype=float)) for v in (A, B, C))
    n, m, p = A.shape[0], B.shape[1], C.shape[0]
    A_aug = np.block([[A, np.zeros((n, p))], [-C, np.zeros((p, p))]])
    B_aug = np.vstack([B, np.zeros((p, m))])
    return A_aug, B_aug


@dataclass(frozen=True)
class PIStateFeedback:
    """Gains of ``u = Kp e + Ki x_i - Kpi x`` with ``e = w - y`` and ``x_i' = e``.

    ``K_aug = [Kx, -Ki]`` places the poles of the augmented loop; ``Kp`` is a free
    feed-forward on the error (scaled by ``alpha``) and ``Kpi = Kx - Kp C`` keeps the
    closed-loop poles unchanged.
    """

    K_aug: FloatArray
    Kx: FloatArray
    Ki: float
    Kp: float
    Kpi: FloatArray
    poles: ComplexArray
    alpha: float


def pi_state_feedback(
    A: ArrayLike, B: ArrayLike, C: ArrayLike, poles: ArrayLike, alpha: float = 1.0
) -> PIStateFeedback:
    """Design a PI state controller for a single-input single-output plant."""
    A, B, C = (np.atleast_2d(np.asarray(v, dtype=float)) for v in (A, B, C))
    n = A.shape[0]
    A_aug, B_aug = augment_with_integrator(A, B, C)
    K_aug = ackermann(A_aug, B_aug, poles)
    Kx = K_aug[:, :n]
    plant_dc_gain = float(-(C @ np.linalg.solve(A, B))[0, 0])
    Kp = alpha / plant_dc_gain
    return PIStateFeedback(
        K_aug=K_aug,
        Kx=Kx,
        Ki=float(-K_aug[0, n]),
        Kp=float(Kp),
        Kpi=Kx - Kp * C,
        poles=np.asarray(poles, dtype=complex),
        alpha=float(alpha),
    )


@dataclass(frozen=True)
class LQRDesign:
    """Optimal state feedback ``u = -K x`` minimising ``int(x'Qx + u'Ru) dt``."""

    K: FloatArray
    P: FloatArray
    poles: ComplexArray
    Q: FloatArray
    R: FloatArray


def lqr(A: ArrayLike, B: ArrayLike, Q: ArrayLike, R: ArrayLike) -> LQRDesign:
    """Solve the continuous-time algebraic Riccati equation and return ``K = R^-1 B' P``."""
    A, B, Q, R = (np.atleast_2d(np.asarray(v, dtype=float)) for v in (A, B, Q, R))
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    poles = np.linalg.eigvals(A - B @ K).astype(complex)
    return LQRDesign(K=K, P=P, poles=poles[np.lexsort((poles.imag, poles.real))], Q=Q, R=R)
