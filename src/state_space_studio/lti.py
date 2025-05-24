"""Continuous-time LTI systems with named signals and exact sampled simulation.

The simulator discretises ``x' = A x + B u`` exactly with the matrix exponential, so a
trajectory is exact at every sample as long as the input follows the chosen hold:

* ``"zoh"`` - input held constant between samples (exact for steps on the grid),
* ``"foh"`` - input interpolated linearly between samples (exact for ramps).
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.linalg import expm

FloatArray = NDArray[np.float64]
ComplexArray = NDArray[np.complex128]
InputSpec = ArrayLike | Mapping[str, ArrayLike] | None

__all__ = ["LTISystem", "Trajectory", "discretize", "simulate"]


def _matrix(value: ArrayLike, name: str) -> FloatArray:
    matrix = np.asarray(value, dtype=float)
    if matrix.ndim == 0:
        matrix = matrix.reshape(1, 1)
    elif matrix.ndim == 1:
        matrix = matrix.reshape(1, -1)
    if matrix.ndim != 2:
        raise ValueError(f"{name} must be 2-D, got shape {matrix.shape}")
    return matrix


@dataclass(frozen=True)
class LTISystem:
    """State-space model ``x' = A x + B u``, ``y = C x + D u`` with named signals."""

    A: FloatArray
    B: FloatArray
    C: FloatArray
    D: FloatArray
    states: tuple[str, ...]
    inputs: tuple[str, ...]
    outputs: tuple[str, ...]
    name: str = ""

    def __post_init__(self) -> None:
        A, B, C, D = (_matrix(getattr(self, k), k) for k in "ABCD")
        n = A.shape[0]
        if A.shape != (n, n):
            raise ValueError(f"A must be square, got {A.shape}")
        if B.shape[0] != n:
            raise ValueError(f"B must have {n} rows, got {B.shape}")
        if C.shape[1] != n:
            raise ValueError(f"C must have {n} columns, got {C.shape}")
        if D.shape != (C.shape[0], B.shape[1]):
            raise ValueError(f"D must be {(C.shape[0], B.shape[1])}, got {D.shape}")
        for label, names, size in (
            ("states", self.states, n),
            ("inputs", self.inputs, B.shape[1]),
            ("outputs", self.outputs, C.shape[0]),
        ):
            if len(names) != size:
                raise ValueError(f"expected {size} {label} names, got {len(names)}")
        everything = (*self.states, *self.inputs, *self.outputs)
        if len(set(everything)) != len(everything):
            raise ValueError(f"signal names must be unique: {everything}")
        for key, value in zip("ABCD", (A, B, C, D), strict=True):
            object.__setattr__(self, key, value)
        for key in ("states", "inputs", "outputs"):
            object.__setattr__(self, key, tuple(getattr(self, key)))

    @property
    def n(self) -> int:
        """Number of states."""
        return self.A.shape[0]

    @property
    def m(self) -> int:
        """Number of inputs."""
        return self.B.shape[1]

    @property
    def p(self) -> int:
        """Number of outputs."""
        return self.C.shape[0]

    def poles(self) -> ComplexArray:
        """Eigenvalues of ``A`` sorted by real part, then imaginary part."""
        poles = np.linalg.eigvals(self.A).astype(complex)
        return poles[np.lexsort((poles.imag, poles.real))]

    def dc_gain(self) -> FloatArray:
        """Static gain ``D - C A^-1 B`` (requires a non-singular ``A``)."""
        return self.D - self.C @ np.linalg.solve(self.A, self.B)

    def simulate(
        self,
        t: ArrayLike,
        u: InputSpec = None,
        x0: ArrayLike | None = None,
        *,
        hold: str = "zoh",
    ) -> Trajectory:
        """Simulate on a uniform time grid, see :func:`simulate`."""
        return simulate(self, t, u, x0, hold=hold)


@dataclass(frozen=True)
class Trajectory:
    """Sampled response with every state, input and output addressable by name."""

    t: FloatArray
    signals: Mapping[str, FloatArray]
    states: tuple[str, ...]
    inputs: tuple[str, ...]
    outputs: tuple[str, ...]

    def __getitem__(self, name: str) -> FloatArray:
        try:
            return self.signals[name]
        except KeyError:
            raise KeyError(f"unknown signal {name!r}; available: {list(self.signals)}") from None

    def __contains__(self, name: object) -> bool:
        return name in self.signals

    def stack(self, names: Sequence[str]) -> FloatArray:
        """Column-stack several signals into an ``(N, len(names))`` array."""
        return np.column_stack([self[name] for name in names])

    def at(self, name: str, time: float) -> float:
        """Signal value at ``time`` (linear interpolation between samples)."""
        return float(np.interp(time, self.t, self[name]))

    def final(self, name: str) -> float:
        """Last sample of a signal."""
        return float(self[name][-1])


def discretize(
    A: ArrayLike, B: ArrayLike, dt: float, hold: str = "zoh"
) -> tuple[FloatArray, FloatArray, FloatArray]:
    """Exact discretisation ``x[k+1] = Ad x[k] + B0 u[k] + B1 u[k+1]``.

    ``B1`` is zero for a zero-order hold. Both variants come from one matrix exponential
    of an augmented matrix (Van Loan's method), so no numerical integration is involved.
    """
    A = _matrix(A, "A")
    B = _matrix(B, "B")
    n, m = B.shape
    if dt <= 0:
        raise ValueError("dt must be positive")
    if hold == "zoh":
        M = np.zeros((n + m, n + m))
        M[:n, :n] = A * dt
        M[:n, n:] = B * dt
        E = expm(M)
        return E[:n, :n], E[:n, n:], np.zeros((n, m))
    if hold == "foh":
        M = np.zeros((n + 2 * m, n + 2 * m))
        M[:n, :n] = A * dt
        M[:n, n : n + m] = B * dt
        M[n : n + m, n + m :] = np.eye(m)
        E = expm(M)
        B1 = E[:n, n + m :]
        return E[:n, :n], E[:n, n : n + m] - B1, B1
    raise ValueError(f"hold must be 'zoh' or 'foh', got {hold!r}")


def _input_matrix(system: LTISystem, u: InputSpec, t: FloatArray) -> FloatArray:
    N = t.size
    if u is None:
        return np.zeros((N, system.m))
    if isinstance(u, Mapping):
        unknown = set(u) - set(system.inputs)
        if unknown:
            raise KeyError(f"unknown inputs {sorted(unknown)}; system has {system.inputs}")
        U = np.zeros((N, system.m))
        for j, name in enumerate(system.inputs):
            if name in u:
                U[:, j] = np.broadcast_to(np.asarray(u[name], dtype=float), (N,))
        return U
    U = np.asarray(u, dtype=float)
    if U.ndim <= 1 and system.m == 1:
        return np.broadcast_to(U, (N,)).reshape(N, 1).copy()
    if U.ndim == 1 and U.shape == (system.m,):
        return np.tile(U, (N, 1))
    if U.shape != (N, system.m):
        raise ValueError(f"u must have shape {(N, system.m)}, got {U.shape}")
    return U


def simulate(
    system: LTISystem,
    t: ArrayLike,
    u: InputSpec = None,
    x0: ArrayLike | None = None,
    *,
    hold: str = "zoh",
) -> Trajectory:
    """Simulate ``system`` on the uniform grid ``t``.

    ``u`` may be ``None`` (free response), an ``(N, m)`` array, a constant vector, or a
    mapping from input name to a scalar or an ``(N,)`` array (missing inputs are zero).
    """
    t = np.asarray(t, dtype=float).ravel()
    if t.size < 2:
        raise ValueError("t needs at least two samples")
    steps = np.diff(t)
    dt = float(steps[0])
    if dt <= 0 or not np.allclose(steps, dt, rtol=1e-6, atol=1e-12):
        raise ValueError("t must be strictly increasing and uniformly spaced")

    U = _input_matrix(system, u, t)
    x = np.zeros(system.n) if x0 is None else np.asarray(x0, dtype=float).ravel()
    if x.shape != (system.n,):
        raise ValueError(f"x0 must have {system.n} entries, got {x.size}")

    Ad, B0, B1 = discretize(system.A, system.B, dt, hold)
    drive = U[:-1] @ B0.T + U[1:] @ B1.T
    X = np.empty((t.size, system.n))
    X[0] = x
    for k in range(t.size - 1):
        X[k + 1] = Ad @ X[k] + drive[k]
    Y = X @ system.C.T + U @ system.D.T

    signals: dict[str, FloatArray] = {}
    for names, data in ((system.states, X), (system.inputs, U), (system.outputs, Y)):
        for j, name in enumerate(names):
            signals[name] = data[:, j].copy()
    return Trajectory(t, signals, system.states, system.inputs, system.outputs)
