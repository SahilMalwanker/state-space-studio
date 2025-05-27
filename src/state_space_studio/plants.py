"""Physical plant models used throughout the project.

Every factory returns a :class:`Plant` holding the state-space matrices, an optional
disturbance-input matrix ``E`` and human-readable signal labels (with units) for plots.

=====================  ======  =======================================================
plant                  states  physics
=====================  ======  =======================================================
``rc_ladder``          3       three cascaded RC low-pass stages
``coupled_mimo``       3       two-input third-order benchmark with a complex mode
``mass_spring_damper`` 2       single translational oscillator
``drivetrain``         6       three inertias coupled by torsional springs
``three_tank``         3       three coupled liquid tanks with a pump
=====================  ======  =======================================================
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field

import numpy as np

from state_space_studio.lti import FloatArray, LTISystem

__all__ = [
    "PLANTS",
    "DrivetrainParams",
    "MassSpringDamperParams",
    "Plant",
    "RCLadderParams",
    "ThreeTankParams",
    "coupled_mimo",
    "drivetrain",
    "mass_spring_damper",
    "rc_ladder",
    "three_tank",
]


@dataclass(frozen=True)
class Plant:
    """Open-loop plant ``x' = A x + B u + E z``, ``y = C x + D u``."""

    key: str
    title: str
    A: FloatArray
    B: FloatArray
    C: FloatArray
    D: FloatArray
    E: FloatArray
    states: tuple[str, ...]
    inputs: tuple[str, ...]
    outputs: tuple[str, ...]
    disturbances: tuple[str, ...] = ()
    labels: Mapping[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for key in "ABCDE":
            object.__setattr__(self, key, np.atleast_2d(np.asarray(getattr(self, key), float)))
        if self.E.shape != (self.n, len(self.disturbances)):
            raise ValueError(f"E must be {(self.n, len(self.disturbances))}, got {self.E.shape}")
        self.open_loop()  # validates the remaining shapes and names

    @property
    def n(self) -> int:
        return self.A.shape[0]

    def label(self, name: str) -> str:
        """Plot label for a signal name, falling back to the name itself."""
        return self.labels.get(name, name)

    def open_loop(self) -> LTISystem:
        """Plant as an :class:`LTISystem` with inputs ``(u..., z...)``."""
        p, q = self.C.shape[0], self.E.shape[1]
        return LTISystem(
            self.A,
            np.hstack([self.B, self.E]),
            self.C,
            np.hstack([self.D, np.zeros((p, q))]),
            self.states,
            (*self.inputs, *self.disturbances),
            self.outputs,
            name=self.title,
        )


def _no_disturbance(n: int) -> FloatArray:
    return np.zeros((n, 0))


# --------------------------------------------------------------------------- RC ladder
@dataclass(frozen=True)
class RCLadderParams:
    """Resistances in ohm and capacitances in farad of the three ladder stages."""

    R: tuple[float, float, float] = (4.0, 4.0, 4.0)
    C: tuple[float, float, float] = (1.0, 1.0, 1.0)


def rc_ladder(params: RCLadderParams | None = None) -> Plant:
    """Three cascaded RC stages; the input is the source voltage, the output ``v3``."""
    p = params or RCLadderParams()
    (R1, R2, R3), (C1, C2, C3) = p.R, p.C
    A = [
        [-(1 / R1 + 1 / R2) / C1, 1 / (R2 * C1), 0.0],
        [1 / (R2 * C2), -(1 / R2 + 1 / R3) / C2, 1 / (R3 * C2)],
        [0.0, 1 / (R3 * C3), -1 / (R3 * C3)],
    ]
    return Plant(
        key="rc_ladder",
        title="RC ladder network",
        A=A,
        B=[[1 / (R1 * C1)], [0.0], [0.0]],
        C=[[0.0, 0.0, 1.0]],
        D=[[0.0]],
        E=_no_disturbance(3),
        states=("v1", "v2", "v3"),
        inputs=("u",),
        outputs=("y",),
        labels={
            "v1": "capacitor voltage $v_1$",
            "v2": "capacitor voltage $v_2$",
            "v3": "capacitor voltage $v_3$",
            "u": "source voltage $u$",
            "y": "output $y = v_3$",
        },
    )


# ------------------------------------------------------------------------ coupled MIMO
def coupled_mimo() -> Plant:
    """Two-input benchmark: a decoupled real mode at -3 drives a lightly damped pair."""
    return Plant(
        key="coupled_mimo",
        title="Coupled two-input system",
        A=[[-3.0, 0.0, 0.0], [0.0, -2.0, 1.0], [10.0, -17.0, 0.0]],
        B=[[4.0, 0.0], [0.0, 1.0], [0.0, 0.0]],
        C=[[0.0, 0.0, 1.0]],
        D=[[0.0, 0.0]],
        E=_no_disturbance(3),
        states=("x1", "x2", "x3"),
        inputs=("u1", "u2"),
        outputs=("y",),
        labels={
            "x1": "$x_1$",
            "x2": "$x_2$",
            "x3": "$x_3$",
            "u1": "input $u_1$",
            "u2": "input $u_2$",
            "y": "output $y = x_3$",
        },
    )


# ------------------------------------------------------------------ mass-spring-damper
@dataclass(frozen=True)
class MassSpringDamperParams:
    """Mass in kg, viscous damping in N s/m and spring stiffness in N/m."""

    m: float = 0.05
    b: float = 1.0
    c: float = 4.0


def mass_spring_damper(params: MassSpringDamperParams | None = None) -> Plant:
    """``m x'' + b x' + c x = u`` with position output."""
    p = params or MassSpringDamperParams()
    return Plant(
        key="mass_spring_damper",
        title="Mass-spring-damper",
        A=[[0.0, 1.0], [-p.c / p.m, -p.b / p.m]],
        B=[[0.0], [1 / p.m]],
        C=[[1.0, 0.0]],
        D=[[0.0]],
        E=_no_disturbance(2),
        states=("x", "v"),
        inputs=("u",),
        outputs=("y",),
        labels={
            "x": "position $x$ [m]",
            "v": "velocity $\\dot{x}$ [m/s]",
            "u": "force $u$ [N]",
            "y": "position $y$ [m]",
        },
    )


# ------------------------------------------------------------------------- drivetrain
@dataclass(frozen=True)
class DrivetrainParams:
    """Inertias in kg m^2, viscous damping in N m s/rad, shaft stiffness in N m/rad."""

    J: tuple[float, float, float] = (2.0e-3, 0.4e-3, 0.3e-3)
    d: tuple[float, float, float] = (8.0e-3, 7.5e-3, 7.5e-3)
    k12: float = 1.5
    k23: float = 1.5


def drivetrain(params: DrivetrainParams | None = None) -> Plant:
    """Three rotating inertias in series, driven by a motor torque on the first one.

    The model is assembled from mass, damping and stiffness matrices,
    ``M phi'' + D_v phi' + K phi = b T1 + Tz``, with the state
    ``x = (phi1, phi2, phi3, omega1, omega2, omega3)`` and output ``phi3``.
    """
    p = params or DrivetrainParams()
    M_inv = np.diag(1 / np.asarray(p.J))
    damping = np.diag(p.d)
    stiffness = np.array(
        [
            [p.k12, -p.k12, 0.0],
            [-p.k12, p.k12 + p.k23, -p.k23],
            [0.0, -p.k23, p.k23],
        ]
    )
    zeros, eye = np.zeros((3, 3)), np.eye(3)
    A = np.block([[zeros, eye], [-M_inv @ stiffness, -M_inv @ damping]])
    E = np.vstack([zeros, M_inv])
    B = E[:, [0]]
    return Plant(
        key="drivetrain",
        title="Three-mass torsional drivetrain",
        A=A,
        B=B,
        C=[[0.0, 0.0, 1.0, 0.0, 0.0, 0.0]],
        D=[[0.0]],
        E=E,
        states=("phi1", "phi2", "phi3", "omega1", "omega2", "omega3"),
        inputs=("u",),
        outputs=("y",),
        disturbances=("Tz1", "Tz2", "Tz3"),
        labels={
            "phi1": "$\\varphi_1$ (motor side)",
            "phi2": "$\\varphi_2$ (middle)",
            "phi3": "$\\varphi_3$ (load)",
            "omega1": "$\\dot{\\varphi}_1$",
            "omega2": "$\\dot{\\varphi}_2$",
            "omega3": "$\\dot{\\varphi}_3$",
            "u": "motor torque $u = T_1$ [N m]",
            "y": "load angle $\\varphi_3$ [rad]",
            "Tz1": "load torque $T_{z1}$",
            "Tz2": "load torque $T_{z2}$",
            "Tz3": "load torque $T_{z3}$",
        },
    )


# ------------------------------------------------------------------------- three tank
@dataclass(frozen=True)
class ThreeTankParams:
    """Linearised flow coefficients in 1/s (levels and pump command in percent)."""

    k1: float = 0.002
    k2: float = 0.004
    k3: float = 0.006
    k12: float = 0.077
    k23: float = 0.081
    kin: float = 0.034
    kz: float = 0.06


def three_tank(params: ThreeTankParams | None = None) -> Plant:
    """Three tanks in series: pump into tank 1, valves 1-2 and 2-3, drains on every tank.

    The disturbance ``z`` enters tank 1 (a negative value models an extra leak).
    """
    p = params or ThreeTankParams()
    A = [
        [-(p.k1 + p.k12), p.k12, 0.0],
        [p.k12, -(p.k12 + p.k23 + p.k2), p.k23],
        [0.0, p.k23, -(p.k23 + p.k3)],
    ]
    return Plant(
        key="three_tank",
        title="Three-tank liquid-level process",
        A=A,
        B=[[p.kin], [0.0], [0.0]],
        C=[[0.0, 0.0, 1.0]],
        D=[[0.0]],
        E=[[p.kz], [0.0], [0.0]],
        states=("h1", "h2", "h3"),
        inputs=("u",),
        outputs=("y",),
        disturbances=("z",),
        labels={
            "h1": "level $h_1$",
            "h2": "level $h_2$",
            "h3": "level $h_3$",
            "u": "pump command $u$ [%]",
            "y": "controlled level $h_3$ [%]",
            "z": "disturbance $z$",
        },
    )


PLANTS: Mapping[str, Callable[[], Plant]] = {
    "rc_ladder": rc_ladder,
    "coupled_mimo": coupled_mimo,
    "mass_spring_damper": mass_spring_damper,
    "drivetrain": drivetrain,
    "three_tank": three_tank,
}
