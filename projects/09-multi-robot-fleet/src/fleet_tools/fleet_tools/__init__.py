"""fleet_tools: operator-facing tools. Not part of the control path.

Kept separate from fleet_ros because nothing here may be needed for the fleet to run:
if this package is missing the fleet still works, which is the test of whether
something is really a tool rather than a component.

**This file imports nothing on purpose.** `display.py` is pure Python so the P6
honesty rules can be tested with no simulator, and an eager `from .cli import main`
here would drag rclpy into that test and make it unrunnable in a clean shell --
which is exactly what happened the first time this module was written. Each entry
point imports its own module; a package's convenience re-export must not decide
whether an unrelated module can be imported without ROS.
"""

__all__ = ["cli", "dashboard", "display"]
