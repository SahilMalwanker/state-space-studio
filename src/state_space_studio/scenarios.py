"""Reproducible experiments for every plant.

Each study function runs one complete workflow - model, analyse, design, simulate,
evaluate - and returns plain data. Plots, animations, the Markdown report and the test
suite all consume these results, so every number quoted in the docs comes from here.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import cache

import numpy as np

from state_space_studio.analysis import StepMetrics, StructuralReport, step_metrics, structural_report
from state_space_studio.design import (
    PIStateFeedback,
    ackermann,
    lqr,
    pi_state_feedback,
    prefilter,
    second_order_poles,
)
from state_space_studio.loops import pi_state_feedback_loop, state_feedback_loop
from state_space_studio.lti import ComplexArray, FloatArray, LTISystem, Trajectory
from state_space_studio.plants import (
    Plant,
    coupled_mimo,
    drivetrain,
    mass_spring_damper,
    rc_ladder,
    three_tank,
)
from state_space_studio.signals import step, time_grid

__all__ = [
    "DRIVETRAIN_CASES",
    "DRIVETRAIN_POLES",
    "ControllerRun",
    "DrivetrainCase",
    "DrivetrainStudy",
    "FundamentalsStudy",
    "TankTiming",
    "ThreeTankStudy",
    "drivetrain_study",
    "fundamentals_study",
    "lqr_weight_sweep",
    "three_tank_study",
]


# =============================================================================== basics
@dataclass(frozen=True)
class FundamentalsStudy:
    """Free and forced responses of the three introductory plants."""

    rc: Plant
    rc_free: Trajectory
    rc_step: Trajectory
    mimo: Plant
    mimo_free: dict[str, Trajectory]
    mimo_step: Trajectory
    mimo_ramp: Trajectory
    msd: Plant
    msd_step: Trajectory
    msd_portrait: list[Trajectory]


@cache
def fundamentals_study() -> FundamentalsStudy:
    rc = rc_ladder()
    t_rc = time_grid(150.0, 0.25)
    rc_free = rc.open_loop().simulate(t_rc, x0=[0.5, 0.0, 0.0])
    rc_step = rc.open_loop().simulate(t_rc, {"u": 1.0})

    mimo = coupled_mimo()
    t_m = time_grid(8.0, 0.01)
    sys_m = mimo.open_loop()
    mimo_free = {
        "x0 = (0.2, 0, 0)": sys_m.simulate(t_m, x0=[0.2, 0.0, 0.0]),
        "x0 = (0, 0.1, 0)": sys_m.simulate(t_m, x0=[0.0, 0.1, 0.0]),
    }
    mimo_step = sys_m.simulate(t_m, {"u1": 3.0, "u2": 1.0})
    mimo_ramp = sys_m.simulate(t_m, {"u1": t_m}, hold="foh")

    msd = mass_spring_damper()
    t_s = time_grid(1.5, 0.002)
    msd_step = msd.open_loop().simulate(t_s, {"u": 1.0})
    angles = np.linspace(0.0, 2.0 * np.pi, 12, endpoint=False)
    portrait = [msd.open_loop().simulate(t_s, x0=[0.3 * np.cos(a), 3.0 * np.sin(a)]) for a in angles]
    return FundamentalsStudy(
        rc, rc_free, rc_step, mimo, mimo_free, mimo_step, mimo_ramp, msd, msd_step, portrait
    )


# =========================================================================== drivetrain
@dataclass(frozen=True)
class DrivetrainCase:
    """Reference and constant load torques applied from ``t = 0``."""

    key: str
    title: str
    w: float = 0.0
    Tz: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def inputs(self) -> dict[str, float]:
        return {"w": self.w, "Tz1": self.Tz[0], "Tz2": self.Tz[1], "Tz3": self.Tz[2]}


DRIVETRAIN_POLES: tuple[float, ...] = (-10.0, -10.0, -100.0, -100.0, -100.0, -100.0)
DRIVETRAIN_CASES: tuple[DrivetrainCase, ...] = (
    DrivetrainCase("reference", "Reference step $w$ = 50 rad", w=50.0),
    DrivetrainCase("load_1", "Load torque $T_{z1}$ = 1 N m", Tz=(1.0, 0.0, 0.0)),
    DrivetrainCase("load_2", "Load torque $T_{z2}$ = 1 N m", Tz=(0.0, 1.0, 0.0)),
    DrivetrainCase("load_3", "Load torque $T_{z3}$ = 1 N m", Tz=(0.0, 0.0, 1.0)),
    DrivetrainCase("load_23", "$T_{z2}$ = 0.5 N m and $T_{z3}$ = 1 N m", Tz=(0.0, 0.5, 1.0)),
)


@dataclass(frozen=True)
class DrivetrainStudy:
    plant: Plant
    report: StructuralReport
    initial: dict[str, Trajectory]
    torque_step: Trajectory
    open_loop_disturbance: dict[str, Trajectory]
    poles: FloatArray
    K: FloatArray
    V: float
    closed_loop: LTISystem
    cases: dict[str, Trajectory]
    tracking: StepMetrics


@cache
def drivetrain_study() -> DrivetrainStudy:
    plant = drivetrain()
    report = structural_report(plant.A, plant.B, plant.C)
    sys_ol = plant.open_loop()

    t_ic = time_grid(1.0, 5e-4)
    initial = {}
    for i, name in enumerate(("phi1", "phi2", "phi3")):
        x0 = np.zeros(plant.n)
        x0[i] = 0.05
        initial[name] = sys_ol.simulate(t_ic, x0=x0)
    torque_step = sys_ol.simulate(time_grid(0.5, 2.5e-4), {"u": 10.0})
    open_loop_disturbance = {
        "Tz2": sys_ol.simulate(t_ic, {"Tz2": 1.0}),
        "Tz3": sys_ol.simulate(t_ic, {"Tz3": 1.0}),
    }

    poles = np.array(DRIVETRAIN_POLES)
    K = ackermann(plant.A, plant.B, poles)
    V = float(prefilter(plant.A, plant.B, plant.C, K)[0, 0])
    loop = state_feedback_loop(plant, K, V, name="Drivetrain - pole placement")
    cases = {case.key: loop.simulate(t_ic, case.inputs()) for case in DRIVETRAIN_CASES}
    tracking = step_metrics(t_ic, cases["reference"]["y"], target=DRIVETRAIN_CASES[0].w)
    return DrivetrainStudy(
        plant,
        report,
        initial,
        torque_step,
        open_loop_disturbance,
        poles,
        K,
        V,
        loop,
        cases,
        tracking,
    )


# =========================================================================== three tank
@dataclass(frozen=True)
class TankTiming:
    """Test profile used for every three-tank controller."""

    horizon: float = 1500.0
    dt: float = 0.5
    reference_time: float = 500.0
    reference_before: float = 30.0
    reference_after: float = 70.0
    leak_time: float = 1000.0
    leak: float = -5.0

    def grid(self) -> FloatArray:
        return time_grid(self.horizon, self.dt)

    def inputs(self, t: FloatArray) -> dict[str, FloatArray]:
        return {
            "w": step(t, self.reference_time, self.reference_before, self.reference_after),
            "z": step(t, self.leak_time, 0.0, self.leak),
        }


@dataclass(frozen=True)
class ControllerRun:
    """One controller evaluated on the common three-tank test profile."""

    key: str
    title: str
    closed_loop: LTISystem
    trajectory: Trajectory
    poles: ComplexArray
    gains: dict[str, object] = field(default_factory=dict)
    tracking: StepMetrics | None = None
    offset_before_leak: float = float("nan")
    offset_after_leak: float = float("nan")
    peak_u: float = float("nan")
    min_u: float = float("nan")
    iae: float = float("nan")


def _evaluate(
    key: str,
    title: str,
    loop: LTISystem,
    gains: dict[str, object],
    timing: TankTiming,
) -> ControllerRun:
    t = timing.grid()
    traj = loop.simulate(t, timing.inputs(t))
    before = timing.leak_time - timing.dt
    error = traj["w"] - traj["y"]
    after_step = t >= timing.reference_time
    return ControllerRun(
        key=key,
        title=title,
        closed_loop=loop,
        trajectory=traj,
        poles=loop.poles(),
        gains=gains,
        tracking=step_metrics(
            t,
            traj["y"],
            target=timing.reference_after,
            t_step=timing.reference_time,
            t_end=before,
        ),
        offset_before_leak=float(timing.reference_after - traj.at("y", before)),
        offset_after_leak=float(error[-1]),
        peak_u=float(traj["u"].max()),
        min_u=float(traj["u"].min()),
        iae=float(np.sum(np.abs(error[after_step])) * timing.dt),
    )


@dataclass(frozen=True)
class ThreeTankStudy:
    plant: Plant
    report: StructuralReport
    initial: Trajectory
    open_loop_profile: Trajectory
    dominant_poles: ComplexArray
    pi_gains: PIStateFeedback
    runs: dict[str, ControllerRun]
    timing: TankTiming


TANK_LQR_WEIGHTS = {
    "lqr_identity": (np.eye(3), 2.0),
    "lqr_tuned": (np.diag([10.0, 1.0, 10.0]), 5.0),
}


@cache
def three_tank_study() -> ThreeTankStudy:
    plant = three_tank()
    report = structural_report(plant.A, plant.B, plant.C)
    sys_ol = plant.open_loop()
    initial = sys_ol.simulate(time_grid(1200.0, 1.0), x0=[60.0, 50.0, 40.0])
    t_ol = time_grid(5000.0, 1.0)
    profile = sys_ol.simulate(t_ol, {"u": step(t_ol, 1500.0, 10.0, 25.0), "z": step(t_ol, 3000.0, 0.0, -5.0)})

    timing = TankTiming()
    dominant = second_order_poles(settling_time=200.0, damping_ratio=1.0)
    runs: dict[str, ControllerRun] = {}

    placed = np.array([dominant[0], dominant[1], 5.0 * dominant[0]]).real
    K = ackermann(plant.A, plant.B, placed)
    V = prefilter(plant.A, plant.B, plant.C, K)
    runs["pole_placement"] = _evaluate(
        "pole_placement",
        "Pole placement + prefilter",
        state_feedback_loop(plant, K, V),
        {"K": K, "V": float(V[0, 0])},
        timing,
    )

    pi_poles = np.array([dominant[0], dominant[1], 10.0 * dominant[0], 10.0 * dominant[0]]).real
    gains = pi_state_feedback(plant.A, plant.B, plant.C, pi_poles, alpha=1.0)
    runs["pi"] = _evaluate(
        "pi",
        "PI state feedback",
        pi_state_feedback_loop(plant, gains),
        {"Kx": gains.Kx, "Ki": gains.Ki, "Kp": gains.Kp, "Kpi": gains.Kpi},
        timing,
    )

    titles = {"lqr_identity": "LQR  Q = I, R = 2", "lqr_tuned": "LQR  Q = diag(10, 1, 10), R = 5"}
    for key, (Q, R) in TANK_LQR_WEIGHTS.items():
        design = lqr(plant.A, plant.B, Q, R)
        V = prefilter(plant.A, plant.B, plant.C, design.K)
        runs[key] = _evaluate(
            key,
            titles[key],
            state_feedback_loop(plant, design.K, V),
            {"K": design.K, "V": float(V[0, 0]), "P": design.P},
            timing,
        )
    return ThreeTankStudy(plant, report, initial, profile, dominant, gains, runs, timing)


def lqr_weight_sweep(R_values: np.ndarray | None = None, Q: np.ndarray | None = None) -> list[ControllerRun]:
    """Three-tank LQR + prefilter for a range of input weights ``R`` (``Q = I`` by default)."""
    plant = three_tank()
    Q = np.eye(3) if Q is None else Q
    R_values = np.logspace(-2, 2, 17) if R_values is None else R_values
    timing = TankTiming()
    runs = []
    for R in R_values:
        design = lqr(plant.A, plant.B, Q, float(R))
        V = prefilter(plant.A, plant.B, plant.C, design.K)
        runs.append(
            _evaluate(
                f"lqr_R_{R:g}",
                f"LQR R = {R:g}",
                state_feedback_loop(plant, design.K, V),
                {"K": design.K, "V": float(V[0, 0]), "R": float(R)},
                timing,
            )
        )
    return runs
