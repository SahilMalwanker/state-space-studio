import io
import sys
from pathlib import Path

import pytest
from PIL import Image

from state_space_studio.cli import main
from state_space_studio.report import format_complex, markdown_report
from state_space_studio.viz.animations import save_gif


def test_report_contains_every_section_and_key_numbers() -> None:
    text = markdown_report()
    for heading in ("## Structural properties", "## Drivetrain", "## Three-tank"):
        assert heading in text
    assert "V = 1.0667" in text
    assert "marginally stable" in text
    assert "| PI state feedback |" in text


def test_format_complex() -> None:
    assert format_complex(-9.2 + 46.6j) == "-9.2 + 46.6j"
    assert format_complex(-3 + 1e-12j) == "-3"
    assert format_complex(0.0) == "0"


def test_cli_report(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["report"]) == 0
    assert "Structural properties" in capsys.readouterr().out


def test_cli_report_survives_a_legacy_console_encoding(monkeypatch: pytest.MonkeyPatch) -> None:
    raw = io.BytesIO()
    monkeypatch.setattr(sys, "stdout", io.TextIOWrapper(raw, encoding="cp1252"))
    assert main(["report"]) == 0
    assert "φ1(1 s)" in raw.getvalue().decode("utf-8")


def test_cli_figures_without_gifs(tmp_path: Path) -> None:
    assert main(["figures", "--out", str(tmp_path), "--no-gifs"]) == 0
    written = {p.name for p in tmp_path.iterdir()}
    assert {"three_tank_controllers.png", "block_pi_state_feedback.svg", "pole_maps.png"} <= written
    svg = (tmp_path / "block_state_feedback.svg").read_text(encoding="utf-8")
    assert "<dc:date>" not in svg  # reproducible output, no timestamps


def test_gif_writer_keeps_white_white(tmp_path: Path) -> None:
    frames = []
    for shade in (0, 80, 160):
        frame = Image.new("RGB", (40, 30), "white")
        frame.paste((shade, 90, 200), (5, 5, 20, 20))
        frames.append(frame)
    path = tmp_path / "test.gif"
    save_gif(frames, path, fps=10, hold_last_s=0.5)
    with Image.open(path) as gif:
        assert gif.n_frames == 3
        assert gif.convert("RGB").getpixel((39, 29)) == (255, 255, 255)


def test_gif_timing_matches_requested_rate(tmp_path: Path) -> None:
    frames = [Image.new("RGB", (16, 16), (index * 8, 40, 80)) for index in range(30)]
    destination = tmp_path / "timing.gif"
    save_gif(frames, destination, fps=15, hold_last_s=0.5)
    with Image.open(destination) as animation:
        duration = 0
        for index in range(animation.n_frames):
            animation.seek(index)
            duration += animation.info["duration"]
    assert duration == 2500
