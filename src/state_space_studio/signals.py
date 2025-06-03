"""Time grids and test signals."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike

from state_space_studio.lti import FloatArray

__all__ = ["ramp", "step", "time_grid"]


def time_grid(stop: float, dt: float, start: float = 0.0) -> FloatArray:
    """Uniform grid from ``start`` to ``stop`` (inclusive) with spacing ``dt``."""
    count = round((stop - start) / dt)
    if count < 1 or not np.isclose(start + count * dt, stop, rtol=1e-9, atol=1e-12):
        raise ValueError("(stop - start) must be a positive integer multiple of dt")
    return np.linspace(start, stop, count + 1)


def step(t: ArrayLike, at: float = 0.0, before: float = 0.0, after: float = 1.0) -> FloatArray:
    """Step from ``before`` to ``after`` at time ``at`` (the new value applies from ``at``)."""
    t = np.asarray(t, dtype=float)
    return np.where(t >= at - 1e-9 * max(1.0, abs(at)), after, before).astype(float)


def ramp(t: ArrayLike, start: float = 0.0, slope: float = 1.0) -> FloatArray:
    """Ramp of the given slope that starts at time ``start``."""
    t = np.asarray(t, dtype=float)
    return slope * np.maximum(t - start, 0.0)
