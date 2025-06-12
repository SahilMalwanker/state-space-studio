"""Static result figures.

Every function builds a stand-alone :class:`~matplotlib.figure.Figure` (no pyplot state),
so the module works headless in CI and inside tests.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import numpy as np
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.ticker import MaxNLocator

from state_space_studio.lti import Trajectory
from state_space_studio.plants import Plant
from state_space_studio.scenarios import (
    DRIVETRAIN_CASES,
    drivetrain_study,
    fundamentals_study,
    lqr_weight_sweep,
    three_tank_study,
)
from state_space_studio.viz.style import (
    AMBER,
    BLUE,
    CONTROLLER_COLORS,
    EDGE,
    GREEN,
    INK,
    MUTED,
    PANEL,
    ROSE,
    SKY,
    STATE_COLORS,
    TEAL,
    VIOLET,
    num,
    styled,
)

__all__ = ["FIGURES", "save_figure"]


def _states(ax: Axes, tr: Trajectory, plant: Plant, names: tuple[str, ...], **kw) -> None:
    for color, name in zip(STATE_COLORS, names, strict=False):
        ax.plot(tr.t, tr[name], color=color, label=plant.label(name), **kw)


def _note(ax: Axes, text: str, loc: str = "lower right", xy: tuple[float, float] | None = None) -> None:
    x, ha = (0.98, "right") if "right" in loc else (0.02, "left")
    y, va = (0.04, "bottom") if "lower" in loc else (0.96, "top")
    if xy is not None:
        x, y = xy
    ax.text(
        x,
        y,
        text,
        transform=ax.transAxes,
        ha=ha,
        va=va,
        fontsize=8.3,
        color=INK,
        bbox={"boxstyle": "round,pad=0.45", "fc": PANEL, "ec": EDGE, "lw": 0.8},
    )


# ========================================================================= fundamentals
def benchmark_figure() -> Figure:
    study = three_tank_study()
    runs = list(study.runs.values())
    colors = [CONTROLLER_COLORS[run.key] for run in runs]
    positions = np.arange(len(runs))[::-1]
    figure = Figure(figsize=(10, 6.8))
    figure.text(0.04, 0.955, "STATE-SPACE STUDIO / CONTROL BENCHMARK", color=TEAL, fontsize=11, weight="bold")
    figure.text(0.04, 0.887, "Tracking. Recovery. Actuator demand.", fontsize=22, weight="bold")
    figure.text(
        0.04,
        0.835,
        "Four controllers on the same three-tank model and disturbance profile.",
        fontsize=11,
        color=MUTED,
    )
    axes = figure.subplots(1, 3, sharey=True)
    figure.subplots_adjust(left=0.21, right=0.97, bottom=0.23, top=0.72, wspace=0.35)
    axes[0].set_yticks(positions, ["Pole placement", "PI feedback", "LQR / identity", "LQR / weighted"])
    columns = (
        ("Leak error", "percentage points at 1500 s", [run.offset_after_leak for run in runs], 59, ".2f"),
        ("Settling time", "seconds / 2 % band", [run.tracking.settling_time for run in runs], 390, ".0f"),
    )
    for axis, (title, label, values, upper, spec) in zip(axes[:2], columns, strict=True):
        axis.barh(positions, values, height=0.25, color=colors)
        axis.scatter(values, positions, color=colors, s=38, zorder=3)
        for position, value in zip(positions, values, strict=True):
            axis.annotate(
                format(value, spec),
                (value, position),
                xytext=(7, 0),
                textcoords="offset points",
                va="center",
                fontsize=12,
                weight="bold",
            )
        axis.set(xlim=(0, upper), xlabel=label)
        axis.set_title(title, fontsize=13, pad=18)
    command_axis = axes[2]
    command_axis.axvspan(0, 100, color=TEAL, alpha=0.07)
    command_axis.axvline(100, color=MUTED, linestyle=":", linewidth=1)
    for position, run, color in zip(positions, runs, colors, strict=True):
        command_axis.plot([run.min_u, run.peak_u], [position, position], color=color, linewidth=4)
        command_axis.scatter([run.min_u, run.peak_u], [position, position], color=color, s=35, zorder=3)
        command_axis.text(
            50,
            position + 0.2,
            f"{num(run.min_u, '.1f')} to {run.peak_u:.1f}",
            ha="center",
            fontsize=11,
            color=color,
            weight="bold",
        )
    command_axis.set(xlim=(-15, 120), xticks=[0, 50, 100], xlabel="pump command [%]")
    command_axis.set_title("Command range", fontsize=13, pad=18)
    for axis in axes:
        axis.set_ylim(-0.5, 3.55)
        axis.tick_params(axis="y", length=0, labelsize=11)
        axis.grid(axis="y", visible=False)
        axis.spines["left"].set_visible(False)
    figure.text(
        0.04,
        0.125,
        "PI removes the leak offset; weighted LQR tracks faster but requests >100 %.",
        fontsize=12,
        weight="bold",
    )
    figure.text(
        0.04,
        0.065,
        "Reference: 30 to 70 % at 500 s. Leak: z = -5 at 1000 s. "
        "Zero initial state.\nShading marks the assumed 0-100 % command range; saturation is not simulated.",
        fontsize=10,
        color=MUTED,
        linespacing=1.6,
    )
    return figure


def fundamentals_figure() -> Figure:
    s = fundamentals_study()
    fig = Figure(figsize=(12.5, 8.6))
    gs = fig.add_gridspec(2, 2, hspace=0.38, wspace=0.22)

    ax = fig.add_subplot(gs[0, 0])
    _states(ax, s.rc_step, s.rc, s.rc.states)
    for color, name in zip(STATE_COLORS, s.rc.states, strict=True):
        ax.plot(s.rc_free.t, s.rc_free[name], color=color, ls=(0, (2, 2)), lw=1.6)
    ax.axhline(1.0, color=MUTED, lw=1, ls="--")
    ax.set(title="RC ladder · step and free response", xlabel="time [s]", ylabel="voltage [V]")
    handles, labels = ax.get_legend_handles_labels()
    handles += [Line2D([], [], color=INK, lw=2), Line2D([], [], color=INK, lw=1.6, ls=(0, (2, 2)))]
    labels += ["unit step, x(0) = 0", "free, $v_1(0)$ = 0.5 V"]
    ax.legend(handles, labels, loc="center right", ncols=1)
    _note(ax, "poles −0.81, −0.39, −0.050 s$^{-1}$\nslowest time constant 20.2 s", "upper right", (0.98, 0.8))

    ax = fig.add_subplot(gs[0, 1])
    for (label, tr), ls in zip(s.mimo_free.items(), ("-", (0, (4, 2))), strict=True):
        for color, name in zip(STATE_COLORS, s.mimo.states, strict=True):
            ax.plot(tr.t, tr[name], color=color, ls=ls, label=f"{s.mimo.label(name)}, {label}")
    ax.set(title="Coupled system · free response", xlabel="time [s]", ylabel="state")
    ax.legend(loc="upper right", ncols=2, fontsize=7.6)
    _note(ax, "poles −3 and −1 ± 4j:\n$x_1$ decays, then excites the\nlightly damped ($x_2$, $x_3$) pair")

    ax3 = fig.add_subplot(gs[1, 0], projection="3d")
    for (label, tr), color in zip(s.mimo_free.items(), (BLUE, ROSE), strict=True):
        x, y, z = tr["x1"], tr["x2"], tr["x3"]
        ax3.plot(x, y, z, color=color, lw=2, label=label)
        ax3.scatter([x[0]], [y[0]], [z[0]], color=color, s=28, depthshade=False)
    ax3.scatter([0], [0], [0], color=INK, s=34, marker="*", depthshade=False, label="equilibrium")
    ax3.set_xlabel("$x_1$", labelpad=2)
    ax3.set_ylabel("$x_2$", labelpad=2)
    ax3.set_zlabel("$x_3$", labelpad=2)
    ax3.set_title("Coupled system · trajectories in state space", loc="left")
    ax3.view_init(elev=24, azim=-58)
    ax3.legend(loc="upper right", fontsize=8)
    for axis in (ax3.xaxis, ax3.yaxis, ax3.zaxis):
        axis.set_pane_color((0.97, 0.98, 0.99, 1.0))
        axis.set_major_locator(MaxNLocator(4))

    ax = fig.add_subplot(gs[1, 1])
    plant = s.msd
    xs, vs = np.linspace(-0.36, 0.36, 41), np.linspace(-3.6, 3.6, 41)
    X, Vv = np.meshgrid(xs, vs)
    dX = plant.A[0, 0] * X + plant.A[0, 1] * Vv
    dV = plant.A[1, 0] * X + plant.A[1, 1] * Vv
    ax.streamplot(X, Vv, dX, dV, color="#CBD5E1", density=1.1, linewidth=0.8, arrowsize=0.8)
    colors = [BLUE, SKY, GREEN, AMBER, ROSE, VIOLET] * 2
    for color, tr in zip(colors, s.msd_portrait, strict=True):
        ax.plot(tr["x"], tr["v"], color=color, lw=1.6, alpha=0.9)
        ax.plot(tr["x"][0], tr["v"][0], "o", color=color, ms=4)
    eigvals, eigvecs = np.linalg.eig(plant.A)
    for lam, vec in zip(eigvals.real, eigvecs.T.real, strict=True):
        direction = vec / np.linalg.norm(vec / [0.36, 3.6])
        ax.plot(*np.outer([-1.2, 1.2], direction).T, color=INK, lw=1, ls="--")
        ax.annotate(
            f"λ = {num(lam)}",
            xy=0.72 * direction,
            fontsize=8,
            color=INK,
            ha="center",
            bbox={"fc": "white", "ec": "none", "pad": 1},
        )
    ax.plot(s.msd_step["x"], s.msd_step["v"], color=INK, lw=2.4, label="unit step u = 1 N")
    ax.plot([0.25], [0.0], "*", color=INK, ms=11)
    ax.set(
        xlim=(-0.36, 0.36),
        ylim=(-3.6, 3.6),
        title="Mass-spring-damper · phase portrait",
        xlabel=plant.label("x"),
        ylabel=plant.label("v"),
    )
    ax.legend(loc="upper right")
    fig.suptitle("Fundamentals: free and forced responses", x=0.07, ha="left", y=0.97)
    return fig


def coupled_inputs_figure() -> Figure:
    s = fundamentals_study()
    fig = Figure(figsize=(12.5, 4.0))
    axes = fig.subplots(1, 2)
    cases = (
        (s.mimo_step, "step input u = (3, 1)", ("u1", "u2")),
        (s.mimo_ramp, "ramp input u = (t, 0)", ("u1", "u2")),
    )
    for ax, (tr, title, inputs) in zip(axes, cases, strict=True):
        _states(ax, tr, s.mimo, s.mimo.states)
        for color, name in zip((VIOLET, MUTED), inputs, strict=True):
            ax.plot(tr.t, tr[name], color=color, lw=1.4, ls="--", label=s.mimo.label(name))
        ax.set(title=f"Coupled system · {title}", xlabel="time [s]", ylabel="amplitude")
        ax.legend(loc="upper left", ncols=2)
    return fig


# =========================================================================== drivetrain
def _drivetrain_column(axes: tuple[Axes, Axes], tr: Trajectory, plant: Plant, title: str) -> None:
    ax_phi, ax_omega = axes
    _states(ax_phi, tr, plant, ("phi1", "phi2", "phi3"))
    _states(ax_omega, tr, plant, ("omega1", "omega2", "omega3"), lw=1.6)
    ax_phi.set_title(title)
    ax_phi.set_ylabel("angle [rad]")
    ax_omega.set_ylabel("angular velocity [rad/s]")
    ax_omega.set_xlabel("time [s]")


def drivetrain_initial_figure() -> Figure:
    s = drivetrain_study()
    fig = Figure(figsize=(13.5, 6.4))
    axes = fig.subplots(2, 3, sharex=True)
    for col, (name, tr) in enumerate(s.initial.items()):
        label = s.plant.label(name).split(" ")[0]
        _drivetrain_column((axes[0, col], axes[1, col]), tr, s.plant, f"{label}(0) = 0.05 rad")
    axes[0, 0].legend(loc="upper right")
    axes[1, 0].legend(loc="upper right")
    fig.suptitle(
        "Drivetrain · open-loop free response (two torsional modes at 7.4 Hz and 16.6 Hz)",
        x=0.07,
        ha="left",
    )
    fig.subplots_adjust(hspace=0.18, wspace=0.25)
    return fig


def drivetrain_open_loop_inputs_figure() -> Figure:
    s = drivetrain_study()
    fig = Figure(figsize=(13.5, 6.4))
    axes = fig.subplots(2, 3)
    columns = (
        (s.torque_step, "motor torque u = 10 N m"),
        (s.open_loop_disturbance["Tz2"], "load torque $T_{z2}$ = 1 N m"),
        (s.open_loop_disturbance["Tz3"], "load torque $T_{z3}$ = 1 N m"),
    )
    for col, (tr, title) in enumerate(columns):
        _drivetrain_column((axes[0, col], axes[1, col]), tr, s.plant, title)
    axes[0, 0].legend(loc="upper left")
    axes[1, 0].legend(loc="lower right")
    _note(axes[0, 0], "rigid-body mode (λ = 0):\nthe shaft keeps turning", "lower right")
    fig.suptitle(
        "Drivetrain · open-loop forced response (no feedback, angles drift without bound)",
        x=0.07,
        ha="left",
    )
    fig.subplots_adjust(hspace=0.28, wspace=0.25)
    return fig


def drivetrain_cases_figure() -> Figure:
    s = drivetrain_study()
    fig = Figure(figsize=(18.0, 9.2))
    axes = fig.subplots(3, len(DRIVETRAIN_CASES), sharex=True)
    for col, case in enumerate(DRIVETRAIN_CASES):
        tr = s.cases[case.key]
        _drivetrain_column((axes[0, col], axes[1, col]), tr, s.plant, case.title)
        axes[1, col].set_xlabel("")
        ax_u = axes[2, col]
        ax_u.plot(tr.t, tr["u"], color=VIOLET)
        ax_u.axhline(tr.final("u"), color=MUTED, lw=1, ls="--")
        ax_u.set_xlabel("time [s]")
        low, high = ax_u.get_ylim()
        upper = (tr.final("u") - low) / (high - low) < 0.55
        ax_u.text(
            0.97,
            0.9 if upper else 0.08,
            f"u(∞) = {num(round(tr.final('u'), 2))} N m\nφ3(1 s) = {num(tr.final('phi3'))} rad",
            transform=ax_u.transAxes,
            ha="right",
            va="top" if upper else "bottom",
            fontsize=8,
            bbox={"boxstyle": "round,pad=0.35", "fc": PANEL, "ec": EDGE, "lw": 0.8},
        )
        if col:
            for row in range(3):
                axes[row, col].set_ylabel("")
    axes[2, 0].set_ylabel("motor torque u [N m]")
    axes[0, 0].legend(loc="lower right")
    axes[1, 0].legend(loc="upper right")
    fig.suptitle(
        f"Drivetrain · state feedback with prefilter  (poles −10 ×2, −100 ×4;  V = {s.V:.4f})",
        x=0.06,
        ha="left",
    )
    fig.subplots_adjust(hspace=0.2, wspace=0.28)
    return fig


# ============================================================================ pole maps
def pole_map_figure() -> Figure:
    dt, tank = drivetrain_study(), three_tank_study()
    fig = Figure(figsize=(13.5, 5.2))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.25], wspace=0.34)

    ax = fig.add_subplot(gs[0, 0])
    ol = dt.report.poles
    cl = np.linalg.eigvals(dt.closed_loop.A)
    ax.axvline(0, color=INK, lw=1)
    ax.axvspan(0, 12, color=ROSE, alpha=0.06, lw=0)
    ax.text(6, 112, "unstable", color=ROSE, fontsize=8, ha="center", rotation=90, va="top")
    ax.scatter(ol.real, ol.imag, marker="x", s=70, lw=2.2, color=ROSE, label="open loop", zorder=3)
    ax.scatter(cl.real, cl.imag, s=60, color=BLUE, ec="white", lw=1, label="closed loop", zorder=3)
    for p in ol:
        if p.imag > 1e-6:
            ax.annotate(
                f"ζ = {-p.real / abs(p):.2f}",
                (p.real, p.imag),
                xytext=(-52, 4),
                textcoords="offset points",
                fontsize=8,
                color=ROSE,
            )
    ax.annotate(
        "rigid-body\nmode λ = 0",
        (0, 0),
        xytext=(-62, 34),
        textcoords="offset points",
        fontsize=8,
        color=ROSE,
        arrowprops={"arrowstyle": "-", "color": ROSE, "lw": 0.8},
    )
    ax.annotate("−10 (×2)", (-10, 0), xytext=(-14, -24), textcoords="offset points", fontsize=8, color=BLUE)
    ax.annotate("−100 (×4)", (-100, 0), xytext=(-6, 12), textcoords="offset points", fontsize=8, color=BLUE)
    ax.set(
        xlim=(-115, 12),
        ylim=(-125, 125),
        title="Drivetrain · s-plane",
        xlabel="Re(s) [1/s]",
        ylabel="Im(s) [rad/s]",
    )
    ax.legend(loc="lower left")

    ax = fig.add_subplot(gs[0, 1])
    rows = [("open loop", tank.report.poles, ROSE, "x")]
    markers = {"pole_placement": "o", "pi": "D", "lqr_identity": "^", "lqr_tuned": "s"}
    short = {
        "pole_placement": "pole placement",
        "pi": "PI (with integrator)",
        "lqr_identity": "LQR  Q = I",
        "lqr_tuned": "LQR  Q = diag(10,1,10)",
    }
    for key, run in tank.runs.items():
        rows.append((short[key], run.poles, CONTROLLER_COLORS[key], markers[key]))
    for i, (_label, poles, color, marker) in enumerate(rows):
        y = len(rows) - 1 - i
        values, counts = np.unique(np.round(poles.real, 6), return_counts=True)
        ax.scatter(values, np.full(values.size, y), marker=marker, s=70, color=color, lw=2, zorder=3)
        for value, count in zip(values, counts, strict=True):
            if count > 1:
                ax.annotate(f"×{count}", (value, y), xytext=(6, 6), textcoords="offset points", fontsize=8)
    ax.set_yticks(range(len(rows))[::-1], [row[0] for row in rows])
    for tick, (_, _, color, _) in zip(ax.get_yticklabels(), rows, strict=True):
        tick.set_color(color)
        tick.set_fontweight("semibold")
    ax.tick_params(axis="y", length=0, labelsize=8.8)
    ax.axvline(tank.dominant_poles[0].real, color=MUTED, lw=1, ls="--")
    ax.text(
        tank.dominant_poles[0].real - 0.003,
        -0.55,
        "design target −0.02 s$^{-1}$ (t$_s$ = 200 s)",
        fontsize=8,
        color=MUTED,
        ha="right",
    )
    ax.set(
        xlim=(-0.26, 0.005),
        ylim=(-0.75, len(rows) - 0.5),
        title="Three-tank · real poles chosen by each design",
        xlabel="Re(s) [1/s]",
    )
    ax.grid(axis="y", visible=False)
    ax.spines["left"].set_visible(False)
    fig.suptitle("Where the poles go", x=0.07, ha="left", y=1.0)
    return fig


# ============================================================================ three tank
def three_tank_open_loop_figure() -> Figure:
    s = three_tank_study()
    fig = Figure(figsize=(13.5, 5.0))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.0, 1.35], height_ratios=[2.2, 1.0], hspace=0.12, wspace=0.18)

    ax = fig.add_subplot(gs[:, 0])
    _states(ax, s.initial, s.plant, s.plant.states)
    ax.set(title="Free response from (60, 50, 40) %", xlabel="time [s]", ylabel="level [%]")
    ax.legend(loc="upper right")
    _note(ax, "poles −0.241, −0.083, −0.004 s$^{-1}$:\nfast equalising, slow draining", "right", (0.98, 0.55))

    ax = fig.add_subplot(gs[0, 1])
    tr = s.open_loop_profile
    _states(ax, tr, s.plant, s.plant.states)
    ax.axvspan(3000, 5000, color=ROSE, alpha=0.06, lw=0)
    ax.set(title="Open-loop profile: pump step at 1500 s, leak at 3000 s", ylabel="level [%]", ylim=(-4, 98))
    ax.tick_params(labelbottom=False)
    ax.legend(loc="upper left", ncols=3)

    ax_u = fig.add_subplot(gs[1, 1], sharex=ax)
    ax_u.plot(tr.t, tr["u"], color=VIOLET, label=s.plant.label("u"))
    ax_u.plot(tr.t, tr["z"], color=ROSE, label=s.plant.label("z"))
    ax_u.axvspan(3000, 5000, color=ROSE, alpha=0.06, lw=0)
    ax_u.set(xlabel="time [s]", ylabel="input")
    ax_u.legend(loc="center right", ncols=2)
    fig.suptitle("Three-tank process · open loop", x=0.07, ha="left", y=1.0)
    return fig


def three_tank_controllers_figure() -> Figure:
    s = three_tank_study()
    timing = s.timing
    fig = Figure(figsize=(15.0, 7.4))
    gs = fig.add_gridspec(2, 2, width_ratios=[2.5, 1.0], height_ratios=[1.6, 1.0], wspace=0.2, hspace=0.12)
    ax_y = fig.add_subplot(gs[0, 0])
    ax_u = fig.add_subplot(gs[1, 0], sharex=ax_y)
    first = next(iter(s.runs.values())).trajectory
    ax_y.plot(first.t, first["w"], color=INK, lw=1.4, ls="--", label="reference w")
    for key, run in s.runs.items():
        tr = run.trajectory
        ax_y.plot(tr.t, tr["y"], color=CONTROLLER_COLORS[key], label=run.title)
        ax_u.plot(tr.t, tr["u"], color=CONTROLLER_COLORS[key], lw=1.6)
    for ax in (ax_y, ax_u):
        ax.axvspan(timing.leak_time, timing.horizon, color=ROSE, alpha=0.06, lw=0)
    ax_y.text(
        timing.leak_time + 12,
        4,
        f"leak opens (z = {num(timing.leak, 'g')})",
        color=ROSE,
        fontsize=8.5,
        weight="semibold",
    )
    ax_y.set(ylabel="tank-3 level y [%]", ylim=(0, 80), title="Controlled level $h_3$")
    ax_y.tick_params(labelbottom=False)
    ax_y.legend(loc="upper left", ncols=1, fontsize=8.3)
    ax_u.axhspan(-20, 0, color=MUTED, alpha=0.12, lw=0)
    ax_u.axhspan(100, 130, color=MUTED, alpha=0.12, lw=0)
    ax_u.text(15, 103, "outside assumed pump range", fontsize=7.8, color=MUTED, va="bottom")
    ax_u.text(15, -3, "infeasible below 0 %", fontsize=7.8, color=MUTED, va="top")
    ax_u.set(xlabel="time [s]", ylabel="pump command u [%]", ylim=(-12, 112), xlim=(0, timing.horizon))

    sub = gs[:, 1].subgridspec(3, 1, hspace=0.55)
    names = [run.title.replace("  ", " ") for run in s.runs.values()]
    colors = [CONTROLLER_COLORS[key] for key in s.runs]
    ypos = np.arange(len(names))[::-1]
    metrics = (
        ("error after leak [%]", [run.offset_after_leak for run in s.runs.values()], "{:.2f}"),
        ("2 % settling time [s]", [run.tracking.settling_time for run in s.runs.values()], "{:.0f}"),
        ("IAE after the step [% s]", [run.iae for run in s.runs.values()], "{:,.0f}"),
    )
    for i, (title, values, fmt) in enumerate(metrics):
        ax = fig.add_subplot(sub[i])
        ax.barh(ypos, values, color=colors, height=0.62)
        for y, v in zip(ypos, values, strict=True):
            ax.text(v, y, "  " + fmt.format(v), va="center", fontsize=8, color=INK)
        ax.set_yticks(ypos, ["PP + prefilter", "PI", "LQR Q=I", "LQR tuned"], fontsize=8.3)
        ax.set_xlim(0, max(values) * 1.35)
        ax.set_title(title, fontsize=9.6)
        ax.grid(axis="y", visible=False)
    fig.suptitle(
        "Three-tank process · four controllers, one test (reference step at 500 s, leak at 1000 s)",
        x=0.06,
        ha="left",
        y=0.99,
    )
    return fig


def lqr_tradeoff_figure() -> Figure:
    sweep = lqr_weight_sweep()
    s = three_tank_study()
    R = np.array([run.gains["R"] for run in sweep])
    pi = s.runs["pi"]
    fig = Figure(figsize=(13.5, 3.9))
    axes = fig.subplots(1, 3)
    series = (
        ("2 % settling time [s]", [r.tracking.settling_time for r in sweep], pi.tracking.settling_time),
        ("error after leak [%]", [r.offset_after_leak for r in sweep], pi.offset_after_leak),
        ("peak pump command [%]", [r.peak_u for r in sweep], pi.peak_u),
    )
    for ax, (title, values, pi_value) in zip(axes, series, strict=True):
        ax.semilogx(R, values, "-o", color=AMBER, ms=4, label="LQR, Q = I")
        ax.axhline(pi_value, color=BLUE, lw=1.4, ls="--", label="PI state feedback")
        ax.axvline(2.0, color=MUTED, lw=1, ls=":")
        ax.set(title=title, xlabel="input weight R")
    axes[0].text(
        2.3, 0.97, "R = 2", transform=axes[0].get_xaxis_transform(), fontsize=8, color=MUTED, va="top"
    )
    axes[2].set_yscale("log")
    axes[2].axhspan(100, 2000, color=MUTED, alpha=0.12, lw=0)
    axes[2].set_ylim(15, 1500)
    axes[2].text(0.012, 115, "outside assumed range (> 100 %)", fontsize=8, color=MUTED)
    axes[0].legend(loc="lower right")
    fig.suptitle(
        "LQR trade-off (Q = I) · lower R improves tracking but increases the requested pump command",
        x=0.07,
        ha="left",
        y=1.04,
    )
    fig.subplots_adjust(wspace=0.28)
    return fig


FIGURES: dict[str, Callable[[], Figure]] = {
    "benchmark.png": benchmark_figure,
    "fundamentals.png": fundamentals_figure,
    "coupled_inputs.png": coupled_inputs_figure,
    "drivetrain_initial.png": drivetrain_initial_figure,
    "drivetrain_open_loop.png": drivetrain_open_loop_inputs_figure,
    "drivetrain_cases.png": drivetrain_cases_figure,
    "pole_maps.png": pole_map_figure,
    "three_tank_open_loop.png": three_tank_open_loop_figure,
    "three_tank_controllers.png": three_tank_controllers_figure,
    "lqr_tradeoff.png": lqr_tradeoff_figure,
}


def save_figure(name: str, path: str | Path) -> None:
    """Build one figure in the project style and write it to ``path``."""
    with styled():
        FIGURES[name]().savefig(path)
