"""State-Space Studio - state-space modelling, analysis and controller design.

Typical workflow::

    from state_space_studio import analysis, design, loops, plants, signals

    plant = plants.three_tank()
    report = analysis.structural_report(plant.A, plant.B, plant.C)
    K = design.ackermann(plant.A, plant.B, [-0.02, -0.02, -0.1])
    V = design.prefilter(plant.A, plant.B, plant.C, K)
    t = signals.time_grid(1500.0, 0.5)
    trajectory = loops.state_feedback_loop(plant, K, V).simulate(t, {"w": 50.0})
    print(trajectory.final("y"))  # -> 50.0 (zero steady-state error)
"""

from state_space_studio.analysis import StepMetrics, StructuralReport, step_metrics, structural_report
from state_space_studio.design import ackermann, lqr, pi_state_feedback, prefilter, second_order_poles
from state_space_studio.loops import pi_state_feedback_loop, state_feedback_loop
from state_space_studio.lti import LTISystem, Trajectory, simulate
from state_space_studio.plants import (
    PLANTS,
    Plant,
    coupled_mimo,
    drivetrain,
    mass_spring_damper,
    rc_ladder,
    three_tank,
)
from state_space_studio.signals import ramp, step, time_grid

__version__ = "1.0.0"

__all__ = [
    "PLANTS",
    "LTISystem",
    "Plant",
    "StepMetrics",
    "StructuralReport",
    "Trajectory",
    "ackermann",
    "coupled_mimo",
    "drivetrain",
    "lqr",
    "mass_spring_damper",
    "pi_state_feedback",
    "pi_state_feedback_loop",
    "prefilter",
    "ramp",
    "rc_ladder",
    "second_order_poles",
    "simulate",
    "state_feedback_loop",
    "step",
    "step_metrics",
    "structural_report",
    "three_tank",
    "time_grid",
]
