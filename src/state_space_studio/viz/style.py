"""Shared colours and Matplotlib settings for every figure and animation."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager

import matplotlib as mpl

INK = "#0F172A"
MUTED = "#64748B"
GRID = "#E2E8F0"
PANEL = "#F8FAFC"
EDGE = "#CBD5E1"

BLUE = "#2563EB"
AMBER = "#F59E0B"
GREEN = "#10B981"
ROSE = "#E11D48"
VIOLET = "#7C3AED"
TEAL = "#0D9488"
SKY = "#0EA5E9"
WATER = "#38BDF8"

STATE_COLORS = (BLUE, AMBER, GREEN)
CONTROLLER_COLORS = {
    "pole_placement": ROSE,
    "pi": BLUE,
    "lqr_identity": AMBER,
    "lqr_tuned": VIOLET,
}

RC_PARAMS = {
    "figure.facecolor": "white",
    "figure.dpi": 100,
    "savefig.dpi": 160,
    "savefig.facecolor": "white",
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.12,
    "font.family": "DejaVu Sans",
    "font.size": 10,
    "mathtext.fontset": "dejavusans",
    "text.color": INK,
    "axes.labelcolor": INK,
    "axes.edgecolor": EDGE,
    "axes.linewidth": 0.9,
    "axes.titlesize": 11,
    "axes.titleweight": "semibold",
    "axes.titlelocation": "left",
    "axes.titlepad": 8,
    "axes.labelsize": 9.5,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.grid": True,
    "axes.axisbelow": True,
    "axes.prop_cycle": mpl.cycler(color=[BLUE, AMBER, GREEN, ROSE, VIOLET, TEAL]),
    "grid.color": GRID,
    "grid.linewidth": 0.8,
    "xtick.color": MUTED,
    "ytick.color": MUTED,
    "xtick.labelsize": 8.5,
    "ytick.labelsize": 8.5,
    "lines.linewidth": 2.0,
    "lines.solid_capstyle": "round",
    "legend.frameon": False,
    "legend.fontsize": 8.5,
    "figure.titlesize": 14,
    "figure.titleweight": "bold",
    "svg.fonttype": "path",
}


@contextmanager
def styled() -> Iterator[None]:
    """Apply the project style for the duration of a ``with`` block."""
    with mpl.rc_context(RC_PARAMS):
        yield


def num(value: float, spec: str = ".2f") -> str:
    """Format a number with a typographic minus sign and without a negative zero."""
    return format(float(value) + 0.0, spec).replace("-", "\N{MINUS SIGN}")
