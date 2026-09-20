"""The launch robot-set guard must discriminate, not just pass.

The defect it exists for was silent: a caller could ask the launch for two
robots while the case declared three, and every odometry-derived check still
looked healthy. A guard that only ever agrees with the tree cannot catch that,
so these tests feed it a tree that is WRONG and require it to say so.
"""
from __future__ import annotations

import pathlib
import subprocess
import sys
import textwrap

import pytest
import yaml

ROOT = pathlib.Path(__file__).resolve().parent.parent
GUARD = ROOT / "scripts/check_launch_robots.py"


def _run(root: pathlib.Path) -> subprocess.CompletedProcess:
    """Run the guard against a synthetic tree.

    FLEET009_ROOT is what points the guard at `root`. Without it the guard reads
    the repository it lives in, and a test could only ever confirm that the real
    tree passes -- it could never require the guard to object to a wrong tree.
    """
    import os

    env = dict(os.environ, FLEET009_ROOT=str(root))
    return subprocess.run(
        [sys.executable, str(GUARD)],
        cwd=root, capture_output=True, text=True, env=env,
    )


def _tree(tmp_path: pathlib.Path, *, runners: list[str], known: list[str],
          batch_src: str | None = None) -> pathlib.Path:
    (tmp_path / "scripts").mkdir(parents=True, exist_ok=True)
    (tmp_path / "config").mkdir(parents=True, exist_ok=True)

    (tmp_path / "config/fleet.yaml").write_text(
        yaml.safe_dump({"robots": {r: {} for r in known}}), encoding="utf-8")
    (tmp_path / "config/scenarios").mkdir(parents=True, exist_ok=True)
    (tmp_path / "config/scenarios/regression_v1.yaml").write_text(
        yaml.safe_dump({"cases": {"X1": {"runners": runners}}}), encoding="utf-8")

    default = textwrap.dedent('''
        def run(body):
            runs = list(body.get("runners") or ["r01"])
            return runs

        class Fleet:
            def __init__(self, runs):
                self.runs = runs

            def _launch_args(self):
                return ["ros2", "launch", "fleet_bringup", "fleet.launch.py",
                        f"robots:={','.join(self.runs)}"]
    ''')
    (tmp_path / "scripts/batch.py").write_text(
        batch_src if batch_src is not None else default, encoding="utf-8")
    return tmp_path


def test_real_tree_passes():
    """The guard must pass on the actual repository."""
    import os

    env = dict(os.environ)
    env.pop("FLEET009_ROOT", None)
    res = subprocess.run([sys.executable, str(GUARD)], cwd=ROOT,
                         capture_output=True, text=True, env=env)
    assert res.returncode == 0, res.stdout + res.stderr


def test_unknown_runner_is_rejected(tmp_path):
    """A case naming a robot the config does not define cannot run."""
    root = _tree(tmp_path, runners=["r01", "r09"], known=["r01", "r02"])
    res = _run(root)
    assert res.returncode == 1
    assert "r09" in res.stdout
    assert "does not define" in res.stdout


def test_duplicate_runner_is_rejected(tmp_path):
    root = _tree(tmp_path, runners=["r01", "r01"], known=["r01"])
    res = _run(root)
    assert res.returncode == 1
    assert "duplicate runner" in res.stdout


def test_batch_not_deriving_from_runners_is_rejected(tmp_path):
    """If batch stops reading `runners`, the launch arg can disagree with the case."""
    src = textwrap.dedent('''
        def run(body):
            runs = ["r01", "r02"]

        class Fleet:
            def __init__(self, runs):
                self.runs = runs

            def _launch_args(self):
                return ["ros2", "launch", "fleet_bringup", "fleet.launch.py",
                        f"robots:={','.join(self.runs)}"]
    ''')
    root = _tree(tmp_path, runners=["r01", "r02", "r03"], known=["r01", "r02", "r03"],
                 batch_src=src)
    res = _run(root)
    assert res.returncode == 1
    assert "runners" in res.stdout


def test_batch_dropping_robots_arg_is_rejected(tmp_path):
    """Without robots:= the launch silently uses its r01,r02 default."""
    src = textwrap.dedent('''
        def run(body):
            runs = list(body.get("runners") or ["r01"])
            return runs

        class Fleet:
            def __init__(self, runs):
                self.runs = runs

            def _launch_args(self):
                return ["ros2", "launch", "fleet_bringup", "fleet.launch.py"]
    ''')
    root = _tree(tmp_path, runners=["r01"], known=["r01"], batch_src=src)
    res = _run(root)
    assert res.returncode == 1
    assert "robots:=" in res.stdout


def test_hand_written_launcher_without_robots_is_rejected(tmp_path):
    """The exact round-17 mistake: a script that spawns the launch with no robots:=.

    It must also be a REAL launcher (an argv containing "ros2") -- a file that
    merely mentions fleet.launch.py is not an offender. Both halves are checked.
    """
    root = _tree(tmp_path, runners=["r01"], known=["r01"])
    (root / "scripts/handrun.py").write_text(textwrap.dedent('''
        import subprocess
        subprocess.run(["ros2", "launch", "fleet_bringup", "fleet.launch.py"])
    '''), encoding="utf-8")
    res = _run(root)
    assert res.returncode == 1
    assert "handrun.py" in res.stdout


def test_mentioning_the_launch_file_is_not_an_offence(tmp_path):
    """A static checker that only reads the launch file must not be flagged.

    This is the distinction the guard has to get right: a name-based search for
    "fleet.launch.py" flags the wrong files.
    """
    root = _tree(tmp_path, runners=["r01"], known=["r01"])
    (root / "scripts/reader.py").write_text(textwrap.dedent('''
        import pathlib
        P = pathlib.Path("src/fleet_bringup/launch/fleet.launch.py")
        text = P.read_text()
    '''), encoding="utf-8")
    res = _run(root)
    assert res.returncode == 0, res.stdout


def test_launcher_with_robots_passes(tmp_path):
    root = _tree(tmp_path, runners=["r01"], known=["r01"])
    (root / "scripts/goodrun.py").write_text(textwrap.dedent('''
        import subprocess
        subprocess.run(["ros2", "launch", "fleet_bringup", "fleet.launch.py",
                        "robots:=r01"])
    '''), encoding="utf-8")
    res = _run(root)
    assert res.returncode == 0, res.stdout
