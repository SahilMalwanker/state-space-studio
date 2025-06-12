"""Block diagrams and plant schematics, drawn with Matplotlib patches (exported as SVG)."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle
from matplotlib.path import Path as MplPath

from state_space_studio.viz.style import BLUE, INK, MUTED, ROSE, TEAL, VIOLET, WATER, styled

__all__ = ["DIAGRAMS", "draw_tanks", "save_diagram"]

PLANT_FILL, PLANT_EDGE = "#EFF6FF", "#93C5FD"
CTRL_FILL, CTRL_EDGE = "#FFF7ED", "#FDBA74"
INT_FILL, INT_EDGE = "#F5F3FF", "#C4B5FD"


@dataclass(frozen=True)
class Box:
    cx: float
    cy: float
    w: float
    h: float

    @property
    def left(self) -> tuple[float, float]:
        return (self.cx - self.w / 2, self.cy)

    @property
    def right(self) -> tuple[float, float]:
        return (self.cx + self.w / 2, self.cy)

    @property
    def top(self) -> tuple[float, float]:
        return (self.cx, self.cy + self.h / 2)

    @property
    def bottom(self) -> tuple[float, float]:
        return (self.cx, self.cy - self.h / 2)


@dataclass(frozen=True)
class Junction:
    cx: float
    cy: float
    r: float = 2.3

    @property
    def left(self) -> tuple[float, float]:
        return (self.cx - self.r, self.cy)

    @property
    def right(self) -> tuple[float, float]:
        return (self.cx + self.r, self.cy)

    @property
    def top(self) -> tuple[float, float]:
        return (self.cx, self.cy + self.r)

    @property
    def bottom(self) -> tuple[float, float]:
        return (self.cx, self.cy - self.r)


class Sketch:
    """Tiny block-diagram kit on an equal-aspect canvas measured in abstract units."""

    def __init__(self, width: float, height: float, scale: float = 0.1) -> None:
        self.fig = Figure(figsize=(width * scale, height * scale))
        self.ax: Axes = self.fig.add_axes((0, 0, 1, 1))
        self.ax.set_xlim(0, width)
        self.ax.set_ylim(0, height)
        self.ax.set_aspect("equal")
        self.ax.axis("off")

    def block(
        self,
        cx: float,
        cy: float,
        w: float,
        h: float,
        text: str,
        kind: str = "plant",
        size: float = 12,
    ) -> Box:
        fill, edge = {"plant": (PLANT_FILL, PLANT_EDGE), "ctrl": (CTRL_FILL, CTRL_EDGE)}.get(
            kind, (INT_FILL, INT_EDGE)
        )
        self.ax.add_patch(
            FancyBboxPatch(
                (cx - w / 2, cy - h / 2),
                w,
                h,
                boxstyle="round,pad=0,rounding_size=1.2",
                fc=fill,
                ec=edge,
                lw=1.4,
            )
        )
        self.ax.text(cx, cy, text, ha="center", va="center", fontsize=size, color=INK)
        return Box(cx, cy, w, h)

    def junction(self, cx: float, cy: float, signs: dict[str, str]) -> Junction:
        j = Junction(cx, cy)
        self.ax.add_patch(Circle((cx, cy), j.r, fc="white", ec=INK, lw=1.3))
        offsets = {"left": (-3.4, 1.8), "top": (1.9, 3.3), "bottom": (1.9, -3.4), "right": (3.4, 1.8)}
        for side, sign in signs.items():
            dx, dy = offsets[side]
            self.ax.text(cx + dx, cy + dy, sign, ha="center", va="center", fontsize=11, color=INK)
        return j

    def wire(self, *points: tuple[float, float], arrow: bool = True, color: str = INK) -> None:
        path = MplPath(np.asarray(points, dtype=float))
        style = "-|>,head_length=0.55,head_width=0.28" if arrow else "-"
        self.ax.add_patch(
            FancyArrowPatch(path=path, arrowstyle=style, mutation_scale=14, lw=1.4, color=color)
        )

    def dot(self, x: float, y: float) -> None:
        self.ax.add_patch(Circle((x, y), 0.55, color=INK))

    def label(self, x: float, y: float, text: str, size: float = 12, **kw) -> None:
        kw.setdefault("color", INK)
        self.ax.text(x, y, text, ha=kw.pop("ha", "center"), va=kw.pop("va", "center"), fontsize=size, **kw)


# ================================================================== block diagrams
def plant_diagram() -> Figure:
    """Open-loop plant built from integrators and gain blocks (the Simulink structure)."""
    s = Sketch(110, 48)
    s.label(3, 27, "$u$")
    s.label(3, 41, "$z$")
    B = s.block(19.5, 27, 11, 7, "$B$")
    E = s.block(19.5, 41, 11, 7, "$E$")
    D = s.block(60, 7, 11, 7, "$D$")
    sum_x = s.junction(34, 27, {"left": "+", "top": "+", "bottom": "+"})
    integ = s.block(46, 27, 9, 8.5, r"$\int$", kind="int", size=15)
    A = s.block(46, 16, 11, 7, "$A$")
    C = s.block(74, 27, 11, 7, "$C$")
    sum_y = s.junction(90, 27, {"left": "+", "bottom": "+"})

    s.wire((5, 27), B.left)
    s.dot(9, 27)
    s.wire((9, 27), (9, 7), D.left)
    s.wire((5, 41), E.left)
    s.wire(E.right, (sum_x.cx, 41), sum_x.top)
    s.wire(B.right, sum_x.left)
    s.wire(sum_x.right, integ.left)
    s.label(38.9, 29.2, r"$\dot{x}$", size=11)
    s.wire(integ.right, C.left)
    s.dot(60, 27)
    s.label(57.5, 29.2, "$x$", size=11)
    s.wire((60, 27), (60, 16), A.right)
    s.wire(A.left, (sum_x.cx, 16), sum_x.bottom)
    s.wire(C.right, sum_y.left)
    s.wire(D.right, (sum_y.cx, 7), sum_y.bottom)
    s.wire(sum_y.right, (103, 27))
    s.label(105.5, 27, "$y$")
    s.label(55, 46, r"plant:  $\dot{x} = Ax + Bu + Ez$,   $y = Cx + Du$", size=10.5, color=MUTED)
    return s.fig


def state_feedback_diagram() -> Figure:
    """``u = V w - K x`` around the plant (pole placement and LQR share this structure)."""
    s = Sketch(100, 42)
    s.label(3, 24, "$w$")
    V = s.block(14, 24, 9, 7, "$V$", kind="ctrl")
    sum_u = s.junction(28, 24, {"left": "+", "bottom": "−"})
    plant = s.block(49, 24, 20, 11, "plant\n" + r"$\dot{x} = Ax + Bu + Ez$", size=10.5)
    C = s.block(76, 24, 9, 7, "$C$")
    K = s.block(49, 8, 9, 7, "$K$", kind="ctrl")

    s.wire((5, 24), V.left)
    s.wire(V.right, sum_u.left)
    s.wire(sum_u.right, plant.left)
    s.label(34.6, 26.2, "$u$", size=11)
    s.label(49, 38.6, "$z$")
    s.wire((49, 36.8), plant.top)
    s.wire(plant.right, C.left)
    s.dot(65, 24)
    s.label(65, 26.4, "$x$", size=11)
    s.wire(C.right, (91, 24))
    s.label(93.5, 24, "$y$")
    s.wire((65, 24), (65, 8), K.right)
    s.wire(K.left, (sum_u.cx, 8), sum_u.bottom)
    s.label(28, 3.2, "controller: prefilter V + state feedback K", size=9, color="#C2410C")
    return s.fig


def pi_state_feedback_diagram() -> Figure:
    """``u = Kp e + Ki x_i - Kpi x`` with ``e = w - y`` and ``x_i' = e``."""
    s = Sketch(120, 46)
    s.label(3, 24, "$w$")
    sum_e = s.junction(11, 24, {"left": "+", "bottom": "−"})
    integ = s.block(26, 24, 8, 8, r"$\int$", kind="int", size=15)
    Ki = s.block(38.5, 24, 9, 7, "$K_i$", kind="ctrl")
    Kp = s.block(33, 35, 9, 7, "$K_p$", kind="ctrl")
    sum_a = s.junction(50, 24, {"left": "+", "top": "+"})
    sum_b = s.junction(59, 24, {"left": "+", "bottom": "−"})
    plant = s.block(78, 24, 20, 11, "plant\n" + r"$\dot{x} = Ax + Bu + Ez$", size=10.5)
    C = s.block(101, 24, 8, 7, "$C$")
    Kpi = s.block(75, 12, 10, 7, "$K_{pi}$", kind="ctrl")

    s.wire((5, 24), sum_e.left)
    s.wire(sum_e.right, integ.left)
    s.dot(17.5, 24)
    s.label(15.6, 26.3, "$e$", size=11)
    s.wire((17.5, 24), (17.5, 35), Kp.left)
    s.wire(Kp.right, (sum_a.cx, 35), sum_a.top)
    s.wire(integ.right, Ki.left)
    s.label(32, 26.3, "$x_i$", size=10)
    s.wire(Ki.right, sum_a.left)
    s.wire(sum_a.right, sum_b.left)
    s.wire(sum_b.right, plant.left)
    s.label(64.4, 26.3, "$u$", size=11)
    s.label(78, 40, "$z$")
    s.wire((78, 38.2), plant.top)
    s.wire(plant.right, C.left)
    s.dot(92.5, 24)
    s.label(92.5, 26.4, "$x$", size=11)
    s.wire((92.5, 24), (92.5, 12), Kpi.right)
    s.wire(Kpi.left, (sum_b.cx, 12), sum_b.bottom)
    s.wire(C.right, (114, 24))
    s.label(116.5, 24, "$y$")
    s.dot(109, 24)
    s.wire((109, 24), (109, 4), (sum_e.cx, 4), sum_e.bottom)
    s.label(
        36,
        44,
        "integral action removes the steady-state error caused by constant disturbances",
        size=9,
        color="#C2410C",
    )
    return s.fig


# ======================================================================= schematics
def _spring(ax: Axes, x0: float, x1: float, y: float, turns: int = 7, amp: float = 2.2) -> None:
    n = 2 * turns
    xs = np.linspace(x0 + 2, x1 - 2, n + 1)
    ys = np.full(n + 1, y)
    ys[1:-1] = y + amp * np.where(np.arange(1, n) % 2 == 1, 1.0, -1.0)
    ax.plot([x0, *xs, x1], [y, *ys, y], color=INK, lw=1.4, solid_joinstyle="miter")


def _ground(ax: Axes, x: float, y: float, width: float = 9) -> None:
    ax.plot([x - width / 2, x + width / 2], [y, y], color=INK, lw=1.4)
    for xi in np.linspace(x - width / 2, x + width / 2 - 1, 6):
        ax.plot([xi, xi + 1.4], [y, y - 1.6], color=INK, lw=0.9)


def _damper(ax: Axes, x: float, y_top: float, y_bottom: float, label: str) -> None:
    mid = (y_top + y_bottom) / 2
    ax.plot([x, x], [y_top, mid + 1.0], color=INK, lw=1.3)
    ax.plot([x - 1.8, x + 1.8], [mid + 1.0, mid + 1.0], color=INK, lw=1.3)
    ax.plot(
        [x - 2.6, x - 2.6, x + 2.6, x + 2.6], [mid + 2.4, mid - 1.6, mid - 1.6, mid + 2.4], color=INK, lw=1.3
    )
    ax.plot([x, x], [mid - 1.6, y_bottom], color=INK, lw=1.3)
    ax.text(x + 3.6, mid, label, fontsize=11, va="center", color=INK)
    _ground(ax, x, y_bottom)


def _turn_arrow(ax: Axes, x: float, y: float, half_width: float, color: str) -> None:
    ax.add_patch(
        FancyArrowPatch(
            (x - half_width, y),
            (x + half_width, y),
            connectionstyle="arc3,rad=-0.45",
            arrowstyle="-|>,head_length=0.45,head_width=0.22",
            mutation_scale=12,
            color=color,
            lw=1.3,
        )
    )


def drivetrain_schematic() -> Figure:
    s = Sketch(112, 56, scale=0.085)
    ax = s.ax
    y = 30.0
    discs = ((22.0, 7.0, 30.0), (60.0, 5.0, 21.0), (92.0, 5.0, 18.0))
    fills = ("#BFDBFE", "#BAE6FD", "#A7F3D0")
    ax.add_patch(Rectangle((2, y - 1.3), 16.5, 2.6, fc="#E2E8F0", ec=INK, lw=1.2))
    ax.add_patch(Rectangle((2, y - 6), 7, 12, fc="#CBD5E1", ec=INK, lw=1.2))
    s.label(5.5, y, "M", size=10, weight="bold")
    for i, ((x, w, h), fill) in enumerate(zip(discs, fills, strict=True)):
        ax.add_patch(
            FancyBboxPatch(
                (x - w / 2, y - h / 2),
                w,
                h,
                boxstyle="round,pad=0,rounding_size=1.0",
                fc=fill,
                ec=INK,
                lw=1.4,
            )
        )
        _turn_arrow(ax, x, y + h / 2 + 1.6, 4.0, MUTED)
        s.label(x + 5.6, y + h / 2 + 3.6, rf"$\varphi_{i + 1}$", size=11, color=MUTED, ha="left")
        s.label(x - 5.0, y + h / 2 + 3.6, f"$J_{i + 1}$", size=12, ha="right")
        _damper(ax, x, y - h / 2, 4.0, f"$d_{i + 1}$")
    _spring(ax, 22 + 3.5, 60 - 2.5, y)
    _spring(ax, 60 + 2.5, 92 - 2.5, y)
    s.label(41, y + 5.2, "$k_{12}$", size=12)
    s.label(76, y + 5.2, "$k_{23}$", size=12)
    ax.add_patch(Rectangle((94.5, y - 1.0), 7.4, 2.0, fc="#E2E8F0", ec=INK, lw=1.2))
    ax.add_patch(Circle((104.5, y), 2.6, fc="white", ec=TEAL, lw=1.6))
    s.label(104.5, y, "θ", size=10, color=TEAL)
    s.label(104.5, y - 5.6, r"$y = \varphi_3$", size=10.5, color=TEAL)
    ax.add_patch(
        FancyArrowPatch(
            (13.8, y + 6.5),
            (13.8, y - 6.5),
            connectionstyle="arc3,rad=0.35",
            arrowstyle="-|>,head_length=0.5,head_width=0.25",
            mutation_scale=14,
            color=VIOLET,
            lw=1.8,
        )
    )
    s.label(12.5, y + 9.6, "$u = T_1$", size=11, color=VIOLET)
    s.label(
        110,
        53.5,
        r"load torques $T_{z1}, T_{z2}, T_{z3}$ act on $J_1, J_2, J_3$",
        size=9.5,
        color=ROSE,
        ha="right",
    )
    return s.fig


def draw_tanks(
    ax: Axes,
    levels: tuple[float, float, float],
    *,
    x0: float = 0.0,
    pump: float = 50.0,
    leak: bool = True,
    setpoint: float | None = None,
) -> None:
    """Draw the three-tank process with levels and pump command in percent.

    The geometry spans roughly ``x0 .. x0 + 100`` by ``0 .. 52`` canvas units; the hero
    animation reuses this function with live values.
    """
    base, height, width, gap = 10.0, 30.0, 17.0, 13.0
    lefts = [x0 + 9 + i * (width + gap) for i in range(3)]
    pipe_y = base + 1.6
    levels = tuple(float(np.clip(level, 0.0, 100.0)) for level in levels)
    for i, left in enumerate(lefts):
        water = height * levels[i] / 100
        ax.add_patch(Rectangle((left, base), width, water, fc=WATER, ec="none", alpha=0.85))
        ax.plot(
            [left, left, left + width, left + width],
            [base + height + 2, base, base, base + height + 2],
            color=INK,
            lw=1.8,
            solid_joinstyle="miter",
        )
        drain_x = left + width / 2
        ax.plot([drain_x, drain_x], [base, base - 4.5], color=INK, lw=1.4)
        ax.add_patch(
            Polygon(
                [[drain_x - 1.3, base - 2.6], [drain_x + 1.3, base - 2.6], [drain_x, base - 1.4]],
                closed=True,
                fc="white",
                ec=INK,
                lw=0.9,
            )
        )
        ax.text(drain_x + 2.0, base - 4.2, f"$k_{i + 1}$", fontsize=9, color=MUTED, va="center")
    for i, label in enumerate(("$k_{12}$", "$k_{23}$")):
        x_left, x_right = lefts[i] + width, lefts[i + 1]
        ax.add_patch(Rectangle((x_left, pipe_y - 0.9), gap, 1.8, fc=WATER, ec=INK, lw=1.0))
        vx = (x_left + x_right) / 2
        ax.add_patch(
            Polygon(
                [
                    [vx - 1.6, pipe_y + 1.8],
                    [vx + 1.6, pipe_y - 1.8],
                    [vx + 1.6, pipe_y + 1.8],
                    [vx - 1.6, pipe_y - 1.8],
                ],
                closed=True,
                fc="white",
                ec=INK,
                lw=1.0,
            )
        )
        ax.text(vx, pipe_y + 3.6, label, ha="center", fontsize=9.5, color=INK)

    inlet_x, pipe_top = lefts[0] + 4, base + height + 6
    ax.plot([x0 + 1.5, inlet_x, inlet_x], [pipe_top, pipe_top, base + height - 1], color=INK, lw=1.6)
    ax.add_patch(Circle((x0 + 4.5, pipe_top), 2.6, fc="white", ec=VIOLET, lw=1.6, zorder=3))
    ax.text(
        x0 + 4.5, pipe_top, "P", ha="center", va="center", fontsize=8.5, color=VIOLET, weight="bold", zorder=4
    )
    flow = float(np.clip(pump / 100.0, 0.0, 1.0))
    surface = base + height * levels[0] / 100
    stream = base + height - 1 - surface
    if flow > 0.005 and stream > 0:
        ax.add_patch(
            Rectangle(
                (inlet_x - 0.4 - 1.1 * flow, surface),
                0.8 + 2.2 * flow,
                stream,
                fc=WATER,
                ec="none",
                alpha=0.75,
            )
        )
    for i, left in enumerate(lefts):
        ax.text(
            left + width / 2,
            base + max(3.6, height * levels[i] / 200),
            f"$h_{i + 1}$ = {levels[i]:.0f} %",
            ha="center",
            va="center",
            fontsize=8.6,
            color=INK,
            weight="semibold",
        )
    feasible = 0.0 <= pump <= 100.0
    ax.text(
        x0 + 8.0,
        pipe_top + 1.2,
        f"pump u = {pump:.0f} %".replace("-", "\N{MINUS SIGN}") + ("" if feasible else "  (infeasible)"),
        ha="left",
        va="bottom",
        fontsize=9,
        color=VIOLET if feasible else ROSE,
        weight="normal" if feasible else "semibold",
    )
    if leak:
        leak_y = base + 5
        ax.add_patch(
            FancyArrowPatch(
                (lefts[0], leak_y),
                (lefts[0] - 6.5, leak_y - 3.5),
                arrowstyle="-|>,head_length=0.5,head_width=0.25",
                mutation_scale=12,
                color=ROSE,
                lw=1.8,
                connectionstyle="arc3,rad=0.3",
            )
        )
        ax.text(lefts[0] - 7.0, leak_y + 1.4, "leak z", ha="right", fontsize=9, color=ROSE, weight="semibold")
    if setpoint is not None:
        y_sp = base + height * setpoint / 100
        ax.plot([lefts[2] - 1.5, lefts[2] + width + 1.5], [y_sp, y_sp], color=INK, lw=1.4, ls=(0, (3, 2)))
        ax.text(lefts[2] + width + 2.2, y_sp, f"w = {setpoint:.0f} %", va="center", fontsize=9, color=INK)


def three_tank_schematic() -> Figure:
    s = Sketch(112, 56, scale=0.085)
    levels = (62.0, 54.0, 48.0)
    draw_tanks(s.ax, levels, x0=4.0, pump=35.0, leak=True)
    sensor_x = 4.0 + 9 + 2 * 30 + 13.5
    surface = 10 + 30 * levels[2] / 100
    s.ax.add_patch(Circle((sensor_x, 50.0), 2.4, fc="white", ec=BLUE, lw=1.4))
    s.label(sensor_x, 50.0, "LT", size=7.5, color=BLUE, weight="bold")
    s.ax.plot([sensor_x, sensor_x], [47.6, surface], color=BLUE, lw=1.0, ls=(0, (2, 2)))
    s.label(sensor_x + 3.6, 50.0, "$y = h_3$", size=11, color=BLUE, ha="left")
    return s.fig


DIAGRAMS: dict[str, Callable[[], Figure]] = {
    "block_plant.svg": plant_diagram,
    "block_state_feedback.svg": state_feedback_diagram,
    "block_pi_state_feedback.svg": pi_state_feedback_diagram,
    "schematic_drivetrain.svg": drivetrain_schematic,
    "schematic_three_tank.svg": three_tank_schematic,
}


def save_diagram(name: str, path: str | Path) -> None:
    """Render one diagram in the project style and write it to ``path``."""
    with styled():
        fig = DIAGRAMS[name]()
        fig.savefig(path, bbox_inches="tight", pad_inches=0.08, metadata={"Date": None})
