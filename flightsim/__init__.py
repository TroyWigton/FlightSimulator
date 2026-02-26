"""Launcher shim package for src-layout flightsim code."""

from __future__ import annotations

from pathlib import Path

_src_pkg = Path(__file__).resolve().parent.parent / "src" / "flightsim"
if _src_pkg.is_dir():
    __path__.append(str(_src_pkg))
