"""Markdown summary of every study (``python -m state_space_studio report``)."""

from __future__ import annotations

from collections.abc import Iterable

import numpy as np

from state_space_studio.analysis import structural_report
from state_space_studio.plants import PLANTS
from state_space_studio.scenarios import (
    DRIVETRAIN_CASES,
    drivetrain_study,
    three_tank_study,
)

__all__ = ["format_complex", "markdown_report"]


def format_complex(z: complex, digits: int = 3, tol: float = 1e-6) -> str:
    """Format a pole as ``-9.2 + 46.6j`` (imaginary parts below ``tol`` are dropped)."""
    z = complex(z)
    scale = max(1.0, abs(z))
    real = f"{z.real:.{digits}g}" if abs(z.real) > tol * scale else "0"
    if abs(z.imag) <= tol * scale:
        return real
    sign = "+" if z.imag > 0 else "-"
    return f"{real} {sign} {abs(z.imag):.{digits}g}j"


def _poles(poles: Iterable[complex], digits: int = 3, tol: float = 1e-6) -> str:
    """List poles once each: conjugate pairs as ``a ± bj``, repeats as ``(×k)``."""
    poles = [complex(p) for p in poles]
    parts: list[str] = []
    seen: list[complex] = []
    for p in poles:
        close = [q for q in poles if abs(p - q) <= tol * max(1.0, abs(p))]
        if p.imag < -tol * max(1.0, abs(p)) or any(abs(p - q) <= tol for q in seen):
            continue
        seen.append(p)
        text = format_complex(p, digits, tol).replace(" + ", " ± ")
        parts.append(f"{text} (×{len(close)})" if len(close) > 1 else text)
    return ", ".join(parts)


def _row(*cells: object) -> str:
    return "| " + " | ".join(str(c) for c in cells) + " |"


def _vector(values: np.ndarray, digits: int = 4) -> str:
    return "[" + ", ".join(f"{v:.{digits}g}" for v in np.ravel(values)) + "]"


def markdown_report() -> str:
    lines = ["## Structural properties", ""]
    lines += [
        _row("Plant", "n", "Open-loop poles", "Stability", "rank Qc", "rank Qo"),
        _row("---", ":-:", "---", "---", ":-:", ":-:"),
    ]
    for factory in PLANTS.values():
        plant = factory()
        rep = structural_report(plant.A, plant.B, plant.C)
        lines.append(
            _row(
                plant.title,
                rep.n,
                _poles(rep.poles),
                rep.stability,
                rep.controllability_rank,
                rep.observability_rank,
            )
        )

    dt = drivetrain_study()
    lines += ["", "## Drivetrain: pole placement + prefilter", ""]
    lines += [
        f"- Desired poles: {_poles(dt.poles)}",
        f"- K = {_vector(dt.K)}",
        f"- V = {dt.V:.4f}",
        f"- Reference step: rise time {dt.tracking.rise_time:.3f} s, "
        f"2 % settling time {dt.tracking.settling_time:.3f} s, "
        f"overshoot {dt.tracking.overshoot_pct:.1f} %",
        "",
        _row("Case", "φ1(1 s) [rad]", "φ2(1 s) [rad]", "φ3(1 s) [rad]", "u(1 s) [N m]", r"max \|u\| [N m]"),
        _row("---", "--:", "--:", "--:", "--:", "--:"),
    ]
    for case in DRIVETRAIN_CASES:
        tr = dt.cases[case.key]
        lines.append(
            _row(
                case.title,
                f"{tr.final('phi1'):.3f}",
                f"{tr.final('phi2'):.3f}",
                f"{tr.final('phi3'):.3f}",
                f"{tr.final('u'):.3f}",
                f"{np.max(np.abs(tr['u'])):.2f}",
            )
        )

    tank = three_tank_study()
    timing = tank.timing
    lines += ["", "## Three-tank process: four controllers, one test profile", ""]
    lines += [
        f"Reference {timing.reference_before:g} → {timing.reference_after:g} % at "
        f"t = {timing.reference_time:g} s; leak z = {timing.leak:g} at t = {timing.leak_time:g} s.",
        "",
        _row(
            "Controller",
            "Closed-loop poles",
            "Rise [s]",
            "Settling 2 % [s]",
            "Error before leak [%]",
            "Error after leak [%]",
            "u range [%]",
            "IAE [% s]",
        ),
        _row("---", "---", "--:", "--:", "--:", "--:", "--:", "--:"),
    ]
    for run in tank.runs.values():
        assert run.tracking is not None
        lines.append(
            _row(
                run.title,
                _poles(run.poles),
                f"{run.tracking.rise_time:.0f}",
                f"{run.tracking.settling_time:.0f}",
                f"{run.offset_before_leak:.2f}",
                f"{run.offset_after_leak:.2f}",
                f"{run.min_u:.1f} … {run.peak_u:.1f}",
                f"{run.iae:.0f}",
            )
        )
    gains = tank.pi_gains
    pp = tank.runs["pole_placement"].gains
    lines += [
        "",
        f"- Pole placement: K = {_vector(pp['K'])}, V = {pp['V']:.4f}",
        f"- PI state feedback: Kx = {_vector(gains.Kx)}, Ki = {gains.Ki:.4f}, "
        f"Kp = {gains.Kp:.4f}, Kpi = {_vector(gains.Kpi)}",
    ]
    for key in ("lqr_identity", "lqr_tuned"):
        g = tank.runs[key].gains
        lines.append(f"- {tank.runs[key].title}: K = {_vector(g['K'])}, V = {g['V']:.4f}")
    return "\n".join(lines) + "\n"
