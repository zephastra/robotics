"""Shared path resolution for the 009 launch files.

Both launch files need the same answers to the same three questions, and getting them
independently wrong is how a launch file ends up working from one directory and not
another:

  * where is the world SDF?
  * where are the rendered per-robot models?
  * what is the world's name (needed for the /world/<name>/create service)?

This module is imported by sibling launch files via an explicit sys.path insert of
their own directory, because `ros2 launch` does not promise to put the launch file's
directory on sys.path.

Order of resolution, and the reason for it:

  world      installed package share first (that is what `ros2 launch` is for), then
             the source tree as a fallback so a raw checkout still works
  models     FLEET_MODELS_DIR, then FLEET_RUNTIME_DIR/models, then <repo>/runtime/models
             Rendered models are GENERATED, so they cannot live in an install prefix
  repo root  FLEET_REPO_ROOT, else walk up looking for assets/worlds/warehouse.sdf

Nothing here guesses from the current working directory. A launch file that behaves
differently depending on where it was started is a launch file that fails in CI.
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from pathlib import Path

WORLD_RELATIVE = Path("assets") / "worlds" / "warehouse.sdf"
MAP_RELATIVE = Path("assets") / "maps" / "warehouse.yaml"


class LaunchSetupError(RuntimeError):
    """Raised for a condition the operator can act on, with the action named."""


def repo_root() -> Path | None:
    """Source tree root, or None when this really is an installed-only workspace."""
    env = os.environ.get("FLEET_REPO_ROOT")
    if env and Path(env).is_dir():
        return Path(env)
    for parent in Path(__file__).resolve().parents:
        if (parent / WORLD_RELATIVE).is_file():
            return parent
    return None


def share_dir(package: str = "fleet_bringup") -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(package))
    except Exception:
        return None


def default_world() -> str:
    share = share_dir()
    if share is not None:
        candidate = share / "worlds" / WORLD_RELATIVE.name
        if candidate.is_file():
            return str(candidate)
    root = repo_root()
    if root is not None and (root / WORLD_RELATIVE).is_file():
        return str(root / WORLD_RELATIVE)
    print(
        "009 launch: could not find warehouse.sdf in the install space or the source "
        "tree. Pass world:=<path>.",
        flush=True,
    )
    return ""


def default_map() -> str:
    share = share_dir()
    if share is not None:
        candidate = share / "maps" / MAP_RELATIVE.name
        if candidate.is_file():
            return str(candidate)
    root = repo_root()
    if root is not None and (root / MAP_RELATIVE).is_file():
        return str(root / MAP_RELATIVE)
    return ""


def default_models_dir() -> str:
    env = os.environ.get("FLEET_MODELS_DIR")
    if env:
        return env
    runtime = os.environ.get("FLEET_RUNTIME_DIR")
    if runtime:
        return str(Path(runtime) / "models")
    root = repo_root()
    if root is not None:
        return str(root / "runtime" / "models")
    return ""


def runtime_dir() -> Path:
    env = os.environ.get("FLEET_RUNTIME_DIR")
    if env:
        return Path(env)
    root = repo_root()
    if root is None:
        raise LaunchSetupError(
            "cannot locate the runtime directory. Set FLEET_RUNTIME_DIR (or "
            "FLEET_REPO_ROOT so it can be derived). Generated models and generated "
            "params are written there, so guessing is not an option."
        )
    return root / "runtime"


def world_name(world_path: str) -> str:
    """Read the world name out of the SDF rather than hardcoding it.

    The spawn service is /world/<name>/create. A hardcoded name that no longer matches
    the file produces "service not available", which reads like a broken Gazebo
    install rather than a stale constant.
    """
    path = Path(world_path)
    if not path.is_file():
        raise LaunchSetupError(
            f"world file not found: {world_path!r}. Pass world:=<path to an .sdf>."
        )
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        raise LaunchSetupError(f"{path}: not well formed XML: {exc}") from exc
    world = root.find("world")
    if world is None or not world.get("name"):
        raise LaunchSetupError(f"{path}: no <world name=...> to spawn into")
    return world.get("name")


# --------------------------------------------------------------------------- #
# spawn poses
# --------------------------------------------------------------------------- #

SPAWNS_RELATIVE = Path("config") / "spawns.yaml"


def spawns_path() -> Path | None:
    env = os.environ.get("FLEET009_CONFIG")
    if env:
        candidate = Path(env) / "spawns.yaml"
        if candidate.is_file():
            return candidate
    root = repo_root()
    if root is not None:
        candidate = root / SPAWNS_RELATIVE
        if candidate.is_file():
            return candidate
    return None


def _parse_spawns(text: str) -> dict[str, dict[str, float]]:
    """Parse the small, regular spawns.yaml with no dependency.

    Deliberately hand written rather than importing yaml: a launch file that fails at
    import time because PyYAML is missing fails with a traceback about yaml, not about
    the thing that is actually wrong. The file's shape is fixed (two levels, four float
    keys), so a strict 30-line parser is both safer and easier to read than a general
    one. Anything unexpected raises rather than being silently skipped, because a
    silently skipped spawn pose becomes a robot in the wrong place.
    """
    sp: dict[str, dict[str, float]] = {}
    current: str | None = None
    in_spawns = False
    for lineno, raw in enumerate(text.splitlines(), 1):
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip())
        key, _, value = line.strip().partition(":")
        value = value.strip()
        if indent == 0:
            in_spawns = key == "spawns"
            current = None
            continue
        if not in_spawns:
            continue
        if indent == 2:
            current = key
            sp[current] = {}
            continue
        if indent == 4 and current is not None:
            if key not in ("x", "y", "z", "yaw"):
                raise LaunchSetupError(
                    f"spawns.yaml line {lineno}: unexpected key {key!r} under "
                    f"{current!r}. Known keys: x, y, z, yaw."
                )
            try:
                sp[current][key] = float(value)
            except ValueError as exc:
                raise LaunchSetupError(
                    f"spawns.yaml line {lineno}: {key}: {value!r} is not a number"
                ) from exc
            continue
        raise LaunchSetupError(f"spawns.yaml line {lineno}: unexpected indentation")
    if not sp:
        raise LaunchSetupError("spawns.yaml parsed to nothing; is it still a `spawns:` map?")
    return sp


def as_float_text(value: float) -> str:
    """Format a float so that it STAYS a float once it lands in YAML.

    Do not "tidy" these with %g. `f"{-6.0:.8g}"` is `-6`, YAML reads `-6` as an integer,
    and a node that declared the parameter as a double then refuses to configure:

        parameter 'initial_pose.x' has invalid type: ... is of type {double},
        setting it to {integer} is not allowed

    AMCL dies on that during configure, the lifecycle manager aborts, and the visible
    symptom is "no navigate_to_pose action server" -- which points at Nav2, not at a
    format string in a launch file. repr() of a float always carries a decimal point or
    an exponent, which is exactly the property needed.
    """
    text = repr(float(value))
    assert "." in text or "e" in text or "E" in text, f"{value!r} formatted as {text!r}"
    return text


def spawn_pose(robot: str) -> dict[str, float]:
    """Where `robot` starts, from config/spawns.yaml.

    This is the single source of truth: scripts/validate_assets.py proves each pose is
    clear of every pad, wall, charger and the protected corridor, and robot.launch.py
    both spawns there and seeds AMCL's initial pose from it. Passing the pose on the
    command line as well means the two can disagree, and a disagreement here shows up as
    a robot that localises perfectly in the wrong place.
    """
    path = spawns_path()
    if path is None:
        raise LaunchSetupError(
            "cannot find config/spawns.yaml. Set FLEET009_CONFIG or pass x:= y:= yaw:= "
            "explicitly."
        )
    table = _parse_spawns(path.read_text(encoding="utf-8"))
    if robot not in table:
        raise LaunchSetupError(
            f"{path}: no spawn pose for {robot!r}. Defined: {sorted(table)}"
        )
    pose = {"x": 0.0, "y": 0.0, "z": 0.05, "yaw": 0.0}
    pose.update(table[robot])
    return pose
