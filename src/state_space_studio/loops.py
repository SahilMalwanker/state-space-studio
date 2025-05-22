"""Closed-loop interconnections, mirroring the block diagrams in ``docs/``.

Every loop keeps the plant's disturbance inputs, so one simulation call can apply a
reference step and a disturbance step together.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike

from state_space_studio.design import PIStateFeedback
from state_space_studio.lti import LTISystem
from state_space_studio.plants import Plant

__all__ = ["pi_state_feedback_loop", "state_feedback_loop"]


def state_feedback_loop(plant: Plant, K: ArrayLike, V: ArrayLike, name: str = "") -> LTISystem:
    """Plant with ``u = V w - K x``.

    Inputs ``(w, z...)``; outputs ``(y, u)``::

        x' = (A - B K) x + B V w + E z
        y  = (C - D K) x + D V w
        u  =      -K x +   V w
    """
    K = np.atleast_2d(np.asarray(K, dtype=float))
    V = np.atleast_2d(np.asarray(V, dtype=float))
    A, B, C, D, E = plant.A, plant.B, plant.C, plant.D, plant.E
    q = E.shape[1]
    return LTISystem(
        A=A - B @ K,
        B=np.hstack([B @ V, E]),
        C=np.vstack([C - D @ K, -K]),
        D=np.block([[D @ V, np.zeros((C.shape[0], q))], [V, np.zeros((K.shape[0], q))]]),
        states=plant.states,
        inputs=("w", *plant.disturbances),
        outputs=("y", "u"),
        name=name or f"{plant.title} - state feedback",
    )


def pi_state_feedback_loop(plant: Plant, gains: PIStateFeedback, name: str = "") -> LTISystem:
    """Plant with ``u = Kp e + Ki x_i - Kpi x``, ``e = w - y`` and ``x_i' = e``.

    The integrator state is called ``xi``. Inputs ``(w, z...)``; outputs ``(y, u, e)``.
    """
    if np.any(plant.D):
        raise ValueError("PI state feedback assumes a strictly proper plant (D = 0)")
    A, B, C, E = plant.A, plant.B, plant.C, plant.E
    q = E.shape[1]
    Kx = gains.Kpi + gains.Kp * C  # equals the state part of the augmented gain
    A_cl = np.block([[A - B @ Kx, gains.Ki * B], [-C, np.zeros((1, 1))]])
    B_cl = np.block([[gains.Kp * B, E], [np.ones((1, 1)), np.zeros((1, q))]])
    C_cl = np.vstack(
        [
            np.hstack([C, np.zeros((1, 1))]),
            np.hstack([-Kx, [[gains.Ki]]]),
            np.hstack([-C, np.zeros((1, 1))]),
        ]
    )
    D_cl = np.vstack([np.zeros((1, 1 + q)), np.hstack([[[gains.Kp]], np.zeros((1, q))]), np.eye(1, 1 + q)])
    return LTISystem(
        A=A_cl,
        B=B_cl,
        C=C_cl,
        D=D_cl,
        states=(*plant.states, "xi"),
        inputs=("w", *plant.disturbances),
        outputs=("y", "u", "e"),
        name=name or f"{plant.title} - PI state feedback",
    )
