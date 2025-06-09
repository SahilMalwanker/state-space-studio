"""Animated GIFs for the README.

Frames are rendered off-screen with the Agg canvas and encoded with Pillow using one
shared palette, which keeps the files small and free of dithering noise.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import numpy as np
from matplotlib.axes import Axes
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.patches import Circle, FancyArrowPatch, Wedge
from matplotlib.ticker import MaxNLocator
from PIL import Image

from state_space_studio.plants import DrivetrainParams
from state_space_studio.scenarios import drivetrain_study, fundamentals_study, three_tank_study
from state_space_studio.viz.diagrams import draw_tanks
from state_space_studio.viz.style import (
    BLUE,
    CONTROLLER_COLORS,
    INK,
    MUTED,
    ROSE,
    STATE_COLORS,
    VIOLET,
    num,
    styled,
)

__all__ = ["ANIMATIONS", "save_gif"]


def _snapshot(canvas: FigureCanvasAgg) -> Image.Image:
    canvas.draw()
    return Image.fromarray(np.asarray(canvas.buffer_rgba())[..., :3].copy())


def save_gif(frames: list[Image.Image], path: str | Path, fps: float, hold_last_s: float = 2.0) -> None:
    """Encode frames with a single shared median-cut palette and no dithering."""
    step = max(1, len(frames) // 10)
    sample = frames[::step]
    strip = Image.new("RGB", (sample[0].width, sample[0].height * len(sample)), "white")
    for i, frame in enumerate(sample):
        strip.paste(frame, (0, i * frame.height))
    palette = strip.quantize(colors=255, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
    colors = palette.getpalette()
    for i in range(0, len(colors), 3):  # median-cut averages tint pure white slightly grey
        if min(colors[i : i + 3]) >= 244:
            colors[i : i + 3] = [255, 255, 255]
    palette.putpalette(colors)
    indexed = [frame.quantize(palette=palette, dither=Image.Dither.NONE) for frame in frames]
    boundaries = np.rint(np.arange(len(indexed) + 1) * 100 / fps).astype(int)
    durations = (np.diff(boundaries) * 10).tolist()
    durations[-1] += round(100 * hold_last_s) * 10
    indexed[0].save(
        path,
        save_all=True,
        append_images=indexed[1:],
        duration=durations,
        loop=0,
        optimize=True,
        disposal=1,
    )


def _canvas(width_px: int, height_px: int, dpi: int = 100) -> tuple[Figure, FigureCanvasAgg]:
    fig = Figure(figsize=(width_px / dpi, height_px / dpi), dpi=dpi)
    return fig, FigureCanvasAgg(fig)


def _blank(ax: Axes, width: float, height: float) -> None:
    ax.cla()
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    ax.set_aspect("equal")
    ax.axis("off")


# ======================================================================= hero: tanks
def three_tank_duel(path: str | Path) -> None:
    """Pole placement vs PI state feedback on the same leak - the README hero."""
    study = three_tank_study()
    timing = study.timing
    runs = (study.runs["pole_placement"], study.runs["pi"])
    t = runs[0].trajectory.t
    frame_times = np.arange(0.0, timing.horizon + 1e-9, 10.0)

    with styled():
        fig, canvas = _canvas(960, 560)
        fig.text(
            0.03,
            0.955,
            "STATE-SPACE STUDIO / Disturbance recovery",
            fontsize=15,
            weight="bold",
            color=INK,
            va="center",
        )
        fig.text(
            0.03,
            0.912,
            "LINEAR SIMULATION  ·  reference step at 500 s  ·  leak at 1000 s  ·  no actuator limits",
            fontsize=10,
            color=MUTED,
            va="center",
        )
        clock = fig.text(
            0.97, 0.955, "", fontsize=12, color=INK, ha="right", va="center", family="DejaVu Sans Mono"
        )
        tank_axes = [fig.add_axes((0.005, 0.36, 0.49, 0.52)), fig.add_axes((0.505, 0.36, 0.49, 0.52))]
        ax = fig.add_axes((0.075, 0.085, 0.9, 0.25))
        ax.plot(t, runs[0].trajectory["w"], color=INK, lw=1.3, ls="--", label="reference w")
        lines = []
        for run in runs:
            (line,) = ax.plot([], [], color=CONTROLLER_COLORS[run.key], lw=2.2, label=run.title)
            lines.append(line)
        cursor = ax.axvline(0, color=MUTED, lw=1)
        leak_patch = ax.axvspan(timing.leak_time, timing.horizon, color=ROSE, alpha=0.0, lw=0)
        ax.set(xlim=(0, timing.horizon), ylim=(0, 96), xlabel="time [s]", ylabel="$h_3$ [%]")
        ax.legend(loc="upper left", ncols=3, fontsize=8.5)

        frames = []
        for time in frame_times:
            k = int(time / timing.dt + 0.5)
            leak = time >= timing.leak_time
            for tank_ax, run in zip(tank_axes, runs, strict=True):
                tr = run.trajectory
                _blank(tank_ax, 118, 61)
                draw_tanks(
                    tank_ax,
                    (tr["h1"][k], tr["h2"][k], tr["h3"][k]),
                    x0=9.0,
                    pump=float(tr["u"][k]),
                    leak=leak,
                    setpoint=float(tr["w"][k]),
                )
                error = tr["w"][k] - tr["y"][k]
                tank_ax.text(
                    60,
                    59.0,
                    run.title,
                    ha="center",
                    va="center",
                    fontsize=11.5,
                    weight="bold",
                    color=CONTROLLER_COLORS[run.key],
                )
                tank_ax.text(
                    60,
                    54.6,
                    f"error  w − h₃ = {num(error, '.1f')} %",
                    ha="center",
                    va="center",
                    fontsize=9.5,
                    color=INK,
                )
            for line, run in zip(lines, runs, strict=True):
                line.set_data(t[: k + 1], run.trajectory["y"][: k + 1])
            cursor.set_xdata([time, time])
            leak_patch.set_alpha(0.08 if leak else 0.0)
            clock.set_text(f"t = {time:4.0f} s")
            frames.append(_snapshot(canvas))
    save_gif(frames, path, fps=15, hold_last_s=2.5)


# ================================================================== drivetrain discs
def _disc(ax: Axes, cx: float, cy: float, radius: float, angle: float, color: str) -> None:
    ax.add_patch(Circle((cx, cy), radius, fc="white", ec=INK, lw=1.6))
    ax.add_patch(Circle((cx, cy), radius * 0.92, fc=color, ec="none", alpha=0.18))
    direction = np.pi / 2 + angle
    ax.add_patch(
        Wedge(
            (cx, cy),
            radius * 0.92,
            np.degrees(direction) - 14,
            np.degrees(direction) + 14,
            fc=color,
            ec="none",
            alpha=0.55,
        )
    )
    tip = (cx + 0.92 * radius * np.cos(direction), cy + 0.92 * radius * np.sin(direction))
    ax.plot([cx, tip[0]], [cy, tip[1]], color=INK, lw=2.4, solid_capstyle="round")
    ax.add_patch(Circle((cx, cy), radius * 0.12, fc=INK, ec="none"))


def _coil(ax: Axes, x0: float, x1: float, y: float, twist: float) -> None:
    strain = float(np.clip(abs(twist) / 2.5, 0.0, 1.0))
    color = tuple(np.array([0.58, 0.64, 0.72]) * (1 - strain) + np.array([0.88, 0.11, 0.28]) * strain)
    xs = np.linspace(x0, x1, 15)
    ys = y + np.where(np.arange(15) % 2 == 0, 0.0, 1.0) * np.where(np.arange(15) % 4 == 1, 1.6, -1.6)
    ys[0] = ys[-1] = y
    ax.plot(xs, ys, color=color, lw=2.0)
    ax.text((x0 + x1) / 2, y - 4.2, f"twist {num(twist)} rad", ha="center", fontsize=8.5, color=color)


def drivetrain_tracking(path: str | Path) -> None:
    """Three discs under pole-placement state feedback tracking a 50 rad reference."""
    study = drivetrain_study()
    tr = study.cases["reference"]
    t = tr.t
    frame_times = np.linspace(0.0, 1.0, 201)
    inertia = np.asarray(DrivetrainParams().J)
    radii = 12.0 * (inertia / inertia[0]) ** 0.25  # a uniform disc has J ~ r^4
    centers = (32.0, 88.0, 138.0)
    cy = 26.0
    names = ("$J_1$ · motor side", "$J_2$", "$J_3$ · load")

    with styled():
        fig, canvas = _canvas(960, 560)
        fig.text(
            0.03,
            0.955,
            "STATE-SPACE STUDIO / Drivetrain servo",
            fontsize=15,
            weight="bold",
            color=INK,
            va="center",
        )
        fig.text(
            0.03,
            0.912,
            f"reference step w = 50 rad (≈ 8 turns)  ·  poles −10 ×2, −100 ×4  ·  "
            f"prefilter V = {study.V:.3f}",
            fontsize=10,
            color=MUTED,
            va="center",
        )
        clock = fig.text(
            0.97, 0.955, "", fontsize=12, color=INK, ha="right", va="center", family="DejaVu Sans Mono"
        )
        stage = fig.add_axes((0.02, 0.47, 0.96, 0.42))
        ax_phi = fig.add_axes((0.075, 0.09, 0.5, 0.33))
        ax_u = fig.add_axes((0.66, 0.09, 0.31, 0.33))
        ax_phi.plot(t, tr["w"], color=INK, lw=1.3, ls="--", label="reference w")
        phi_lines = [
            ax_phi.plot([], [], color=c, lw=2.0, label=study.plant.label(n))[0]
            for c, n in zip(STATE_COLORS, ("phi1", "phi2", "phi3"), strict=True)
        ]
        (u_line,) = ax_u.plot([], [], color=VIOLET, lw=2.0)
        cursors = [ax_phi.axvline(0, color=MUTED, lw=1), ax_u.axvline(0, color=MUTED, lw=1)]
        ax_phi.set(xlim=(0, 1), ylim=(-3, 56), xlabel="time [s]", ylabel="angle [rad]")
        ax_phi.legend(loc="lower right", fontsize=8.2)
        ax_u.set(xlim=(0, 1), ylim=(-6, 56), xlabel="time [s]", ylabel="motor torque u [N m]")

        frames = []
        for time in frame_times:
            k = int(time / (t[1] - t[0]) + 0.5)
            phis = (tr["phi1"][k], tr["phi2"][k], tr["phi3"][k])
            _blank(stage, 172, 46)
            for cx, radius, phi, color, name in zip(centers, radii, phis, STATE_COLORS, names, strict=True):
                _disc(stage, cx, cy, radius, phi, color)
                stage.text(cx, cy - radius - 3.4, name, ha="center", va="center", fontsize=10.5, color=INK)
                stage.text(
                    cx,
                    cy - radius - 8.0,
                    f"φ = {num(phi, '6.2f')} rad",
                    ha="center",
                    va="center",
                    fontsize=10,
                    color=color,
                    family="DejaVu Sans Mono",
                    weight="bold",
                )
            _coil(stage, centers[0] + radii[0] + 2, centers[1] - radii[1] - 2, cy, phis[0] - phis[1])
            _coil(stage, centers[1] + radii[1] + 2, centers[2] - radii[2] - 2, cy, phis[1] - phis[2])
            u = tr["u"][k]
            arrow_x = centers[0] - radii[0] - 3.5
            if abs(u) > 0.2:
                size = float(np.clip(abs(u) / 53.0, 0.2, 1.0))
                sign = 1.0 if u > 0 else -1.0
                start = (arrow_x, cy - sign * 8 * size)
                end = (arrow_x, cy + sign * 8 * size)
                stage.add_patch(
                    FancyArrowPatch(
                        start,
                        end,
                        connectionstyle=f"arc3,rad={0.6 * sign}",
                        arrowstyle="-|>,head_length=0.5,head_width=0.3",
                        mutation_scale=14,
                        color=VIOLET,
                        lw=1.5 + 2.0 * size,
                    )
                )
            stage.text(
                arrow_x,
                cy + radii[0] + 3.0,
                f"u = {num(u, '.1f')} N m",
                ha="center",
                fontsize=9.5,
                color=VIOLET,
            )
            stage.text(170, 44, "target  φ₃ = 50 rad", ha="right", va="top", fontsize=9.5, color=MUTED)
            for line, name in zip(phi_lines, ("phi1", "phi2", "phi3"), strict=True):
                line.set_data(t[: k + 1], tr[name][: k + 1])
            u_line.set_data(t[: k + 1], tr["u"][: k + 1])
            for cursor in cursors:
                cursor.set_xdata([time, time])
            clock.set_text(f"t = {time:5.3f} s")
            frames.append(_snapshot(canvas))
    save_gif(frames, path, fps=25, hold_last_s=2.0)


# ===================================================================== phase space
def phase_space(path: str | Path) -> None:
    """Two free responses of the coupled system spiralling into the origin (3-D)."""
    study = fundamentals_study()
    trajectories = list(study.mimo_free.items())
    count = 90
    reveal = np.minimum(1.0, np.linspace(0.0, 1.3, count))
    azimuth = np.linspace(-65.0, 35.0, count)

    with styled():
        fig, canvas = _canvas(640, 520)
        fig.text(
            0.04, 0.955, "Free response in state space", fontsize=14, weight="bold", color=INK, va="center"
        )
        fig.text(
            0.04,
            0.912,
            "LINEAR SIMULATION  ·  poles −3, −1 ± 4j  ·  two initial states",
            fontsize=9.5,
            color=MUTED,
            va="center",
        )
        ax = fig.add_axes((0.0, 0.0, 1.0, 0.9), projection="3d")
        ax.grid(False)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.set_pane_color((0.96, 0.97, 0.99, 1.0))
            axis.set_major_locator(MaxNLocator(3))
        all_x = np.concatenate([tr.stack(("x1", "x2", "x3")) for _, tr in trajectories])
        lows, highs = all_x.min(axis=0), all_x.max(axis=0)
        colors = (BLUE, ROSE)
        lines, heads = [], []
        for (label, tr), color in zip(trajectories, colors, strict=True):
            (line,) = ax.plot([], [], [], color=color, lw=2.2, label=label)
            (head,) = ax.plot([], [], [], "o", color=color, ms=6)
            ax.plot([tr["x1"][0]], [tr["x2"][0]], [tr["x3"][0]], "o", color=color, ms=4, alpha=0.6)
            lines.append(line)
            heads.append(head)
        ax.plot([0], [0], [0], "*", color=INK, ms=11, label="equilibrium")
        pad = 0.08 * (highs - lows)
        ax.set(
            xlim=(lows[0] - pad[0], highs[0] + pad[0]),
            ylim=(lows[1] - pad[1], highs[1] + pad[1]),
            zlim=(lows[2] - pad[2], highs[2] + pad[2]),
            xlabel="$x_1$",
            ylabel="$x_2$",
            zlabel="$x_3$",
        )
        ax.legend(loc="upper right", fontsize=8.5)

        frames = []
        for fraction, az in zip(reveal, azimuth, strict=True):
            for (_, tr), line, head in zip(trajectories, lines, heads, strict=True):
                k = max(1, int(fraction * (tr.t.size - 1)))
                line.set_data_3d(tr["x1"][: k + 1], tr["x2"][: k + 1], tr["x3"][: k + 1])
                head.set_data_3d([tr["x1"][k]], [tr["x2"][k]], [tr["x3"][k]])
            ax.view_init(elev=22, azim=az)
            frames.append(_snapshot(canvas))
    save_gif(frames, path, fps=20, hold_last_s=1.5)


ANIMATIONS: dict[str, Callable[[str | Path], None]] = {
    "hero_three_tank.gif": three_tank_duel,
    "drivetrain_servo.gif": drivetrain_tracking,
    "phase_space.gif": phase_space,
}
