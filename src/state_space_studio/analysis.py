"""Structural analysis (stability, controllability, observability) and step metrics."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from state_space_studio.lti import ComplexArray, FloatArray

__all__ = [
    "Mode",
    "StepMetrics",
    "StructuralReport",
    "classify_stability",
    "controllability_matrix",
    "modes",
    "observability_matrix",
    "step_metrics",
    "structural_report",
]


def controllability_matrix(A: ArrayLike, B: ArrayLike) -> FloatArray:
    """Kalman controllability matrix ``[B, AB, ..., A^(n-1) B]``."""
    A, B = np.atleast_2d(A).astype(float), np.atleast_2d(B).astype(float)
    blocks = [B]
    for _ in range(A.shape[0] - 1):
        blocks.append(A @ blocks[-1])
    return np.hstack(blocks)


def observability_matrix(A: ArrayLike, C: ArrayLike) -> FloatArray:
    """Kalman observability matrix ``[C; CA; ...; C A^(n-1)]``."""
    A, C = np.atleast_2d(A).astype(float), np.atleast_2d(C).astype(float)
    blocks = [C]
    for _ in range(A.shape[0] - 1):
        blocks.append(blocks[-1] @ A)
    return np.vstack(blocks)


def classify_stability(poles: ArrayLike, tol: float = 1e-9) -> str:
    """Classify continuous-time stability from the eigenvalues of ``A``.

    Simple imaginary-axis eigenvalues imply marginal stability. Repeated ones require
    a Jordan-structure check, which cannot be inferred from eigenvalues alone.
    """
    poles = np.asarray(poles, dtype=complex)
    if np.any(poles.real > tol):
        return "unstable"
    on_axis = poles[np.abs(poles.real) <= tol]
    if on_axis.size == 0:
        return "asymptotically stable"
    for pole in on_axis:
        if np.sum(np.abs(on_axis - pole) <= 1e3 * tol) > 1:
            return "undetermined (repeated imaginary-axis poles)"
    return "marginally stable"


@dataclass(frozen=True)
class StructuralReport:
    """Eigenvalues, stability class and Kalman ranks of ``(A, B, C)``."""

    poles: ComplexArray
    stability: str
    controllability_rank: int
    observability_rank: int
    n: int

    @property
    def controllable(self) -> bool:
        return self.controllability_rank == self.n

    @property
    def observable(self) -> bool:
        return self.observability_rank == self.n


def structural_report(A: ArrayLike, B: ArrayLike, C: ArrayLike) -> StructuralReport:
    """Compute the structural properties a controller design starts from."""
    A = np.atleast_2d(A).astype(float)
    poles = np.linalg.eigvals(A).astype(complex)
    poles = poles[np.lexsort((poles.imag, poles.real))]
    return StructuralReport(
        poles=poles,
        stability=classify_stability(poles),
        controllability_rank=int(np.linalg.matrix_rank(controllability_matrix(A, B))),
        observability_rank=int(np.linalg.matrix_rank(observability_matrix(A, C))),
        n=A.shape[0],
    )


@dataclass(frozen=True)
class Mode:
    """One eigenvalue described by natural frequency, damping ratio and time constant."""

    pole: complex
    natural_frequency: float
    damping_ratio: float
    time_constant: float


def modes(poles: ArrayLike) -> list[Mode]:
    """Describe each pole (conjugate pairs are listed once, positive imaginary part)."""
    result = []
    for pole in np.asarray(poles, dtype=complex):
        if pole.imag < -1e-12:
            continue
        wn = abs(pole)
        zeta = -pole.real / wn if wn > 0 else 1.0
        tau = -1 / pole.real if pole.real < 0 else np.inf
        result.append(Mode(complex(pole), float(wn), float(zeta), float(tau)))
    return result


@dataclass(frozen=True)
class StepMetrics:
    """Classic step-response figures, measured after the step instant."""

    rise_time: float
    settling_time: float
    overshoot_pct: float
    peak: float
    final_value: float
    steady_state_error: float


def _first_crossing(t: FloatArray, s: FloatArray, level: float) -> float:
    above = np.nonzero(s >= level)[0]
    if above.size == 0:
        return float("nan")
    i = int(above[0])
    if i == 0:
        return float(t[0])
    frac = (level - s[i - 1]) / (s[i] - s[i - 1])
    return float(t[i - 1] + frac * (t[i] - t[i - 1]))


def step_metrics(
    t: ArrayLike,
    y: ArrayLike,
    *,
    target: float,
    t_step: float = 0.0,
    t_end: float | None = None,
    band: float = 0.02,
) -> StepMetrics:
    """Rise time (10-90 %), settling time (``band``), overshoot and steady-state error.

    Rise, settling and overshoot follow the MATLAB ``stepinfo`` conventions and are taken
    relative to the response's own final value; the steady-state error is
    ``target - final value``. Times are measured from ``t_step``.
    """
    t, y = np.asarray(t, dtype=float), np.asarray(y, dtype=float)
    t_end = t[-1] if t_end is None else t_end
    window = (t >= t_step - 1e-9) & (t <= t_end + 1e-9)
    tw, yw = t[window] - t_step, y[window]
    y0, y_final = yw[0], yw[-1]
    delta = y_final - y0
    if abs(delta) < 1e-12:
        nan = float("nan")
        return StepMetrics(nan, nan, 0.0, float(y_final), float(y_final), target - y_final)

    s = (yw - y0) / delta
    rise = _first_crossing(tw, s, 0.9) - _first_crossing(tw, s, 0.1)
    outside = np.nonzero(np.abs(yw - y_final) > band * abs(delta))[0]
    settling = 0.0 if outside.size == 0 else float(tw[min(outside[-1] + 1, tw.size - 1)])
    i_peak = int(np.argmax(s))
    return StepMetrics(
        rise_time=float(rise),
        settling_time=settling,
        overshoot_pct=float(max(0.0, (s[i_peak] - 1.0) * 100.0)),
        peak=float(yw[i_peak]),
        final_value=float(y_final),
        steady_state_error=float(target - y_final),
    )
