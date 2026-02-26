"""Workspace-local Python startup customization.

This file is auto-imported by Python's site module when running from this
repository root. It ensures the `src` layout package is importable with:

    python -m flightsim
"""

from __future__ import annotations

import os
import sys


def _ensure_src_on_path() -> None:
    repo_root = os.path.dirname(__file__)
    src_path = os.path.join(repo_root, "src")
    if os.path.isdir(src_path) and src_path not in sys.path:
        sys.path.insert(0, src_path)


_ensure_src_on_path()
