"""MuJoCo simulation backend for 008.

This package is the **only** place that touches MuJoCo, MNN or the physical
assets. It is split so the robot interface (:mod:`.robot_runtime`,
:mod:`.walking_policy`, :mod:`.camera`, :mod:`.vision`, :mod:`.tactile`,
:mod:`.stance`) is separable from the run driver (:mod:`.backend`) and the
truth-only evaluator (:mod:`.truth_evaluator`).

``ROOT`` is the 008 project root; the simulation modules live one level deeper
than the 007 baseline's modules, so the path anchor is ``parents[3]``.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

__all__ = ["ROOT"]
