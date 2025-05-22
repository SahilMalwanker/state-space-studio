"""Command-line entry point: ``state-space-studio report`` and ``state-space-studio figures``."""

from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence
from pathlib import Path

__all__ = ["main"]


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="state-space-studio",
        description="State-space modelling, analysis and control of five physical plants.",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("report", help="print the structural analysis and controller results")

    figures = sub.add_parser("figures", help="render every figure, diagram and animation")
    figures.add_argument("--out", type=Path, default=Path("docs/assets"), help="output folder")
    figures.add_argument("--no-gifs", action="store_true", help="skip the animated GIFs")
    return parser


def _write_text(text: str) -> None:
    """Write to stdout, falling back to UTF-8 bytes on legacy code pages (e.g. cp1252)."""
    try:
        sys.stdout.write(text)
    except UnicodeEncodeError:
        sys.stdout.flush()
        sys.stdout.buffer.write(text.encode("utf-8"))
        sys.stdout.buffer.flush()


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.command == "report":
        from state_space_studio.report import markdown_report

        _write_text(markdown_report())
        return 0

    from state_space_studio.viz import render_all

    for path in render_all(args.out, gifs=not args.no_gifs):
        print(f"wrote {path.as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
