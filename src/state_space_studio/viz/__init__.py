"""Figures, block diagrams and animations for the README and docs."""

from __future__ import annotations

from pathlib import Path

__all__ = ["render_all"]


def render_all(out_dir: str | Path, *, gifs: bool = True) -> list[Path]:
    """Write every static figure, diagram and (optionally) animation to ``out_dir``."""
    from state_space_studio.viz.animations import ANIMATIONS
    from state_space_studio.viz.diagrams import DIAGRAMS, save_diagram
    from state_space_studio.viz.figures import FIGURES, save_figure

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    for name in FIGURES:
        save_figure(name, out / name)
        written.append(out / name)
    for name in DIAGRAMS:
        save_diagram(name, out / name)
        written.append(out / name)
    if gifs:
        for name, build in ANIMATIONS.items():
            build(out / name)
            written.append(out / name)
    return written
