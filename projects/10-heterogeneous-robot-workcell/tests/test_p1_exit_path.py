"""The exit path is a measured object, so its shape is testable without running physics.

reports/p1-h-seq-07 measured the old behaviour (command the default posture immediately)
moving the V1 tray 12.0 mm, over the 5 mm limit. reports/p1-h-exitpath-line-04 measured a
Cartesian line alone keeping the tray still but leaving the arm 0.284 rad off the posture.
reports/p1-h-exitpath-line_then_default-04 measured the two-stage path passing both halves,
and reports/p1-h-exitpath-out_y_then_default-04 measured the more elaborate alternative.

These tests pin the three properties that make the chosen path work: the Cartesian stage
walks the straight line to the default hand pose, the joint stage is a ramp rather than a
step, and the path object the probe drives is the same one the comparison was run on.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from humanoid007 import runtime as rt          # noqa: E402

ANCHORS = {side: np.array([.23, sign * .30, .85]) for side, sign in
           [('left', 1), ('right', -1)]}
WORLD = 'assets/world_tray_v1.xml'


class StubRuntime:
    """Enough of Runtime for ExitPath, with the IK recorded instead of solved.

    The solver itself is exercised elsewhere; here the question is what the path *asks* for,
    so recording the requests is more informative than returning real joints.
    """

    def __init__(self, real):
        self.m, self.d = real.m, real.d
        self.qa, self.arm_ids = real.qa, real.arm_ids
        self.ik_targets = []
        self.ik_result = real.policy.default[real.arm_ids].copy() + 0.40

    def snapshot(self):
        joints = self.d.qpos[self.qa[self.arm_ids]]
        return {'hands': {s: list(v) for s, v in rt.hand_sites_at(
            self.m, self.d, self.qa, self.arm_ids, joints).items()}}

    def arm_ik(self, targets):
        self.ik_targets.append({s: np.asarray(v).copy() for s, v in targets.items()})
        return self.ik_result.copy()


@pytest.fixture(scope='module')
def real():
    return rt.Runtime(ROOT / WORLD)


def build(real, variant=rt.ExitPath.DEFAULT_VARIANT):
    """Put the model at the grasp pose, then build the path from there."""
    real.d.qpos[real.qa[real.arm_ids]] = real.arm_ik(ANCHORS)
    stub = StubRuntime(real)
    start = {s: np.asarray(v, dtype=float) for s, v in stub.snapshot()['hands'].items()}
    path = rt.ExitPath(stub, ANCHORS, real.policy.default[real.arm_ids], variant)
    return stub, path, start


def test_the_default_is_the_two_stage_path():
    assert rt.ExitPath.DEFAULT_VARIANT == 'line_then_default'
    assert set(rt.ExitPath.VARIANTS) == {'straight', 'line', 'line_then_default',
                                         'out_y_then_default'}


def test_an_unknown_variant_is_refused(real):
    with pytest.raises(ValueError, match='unknown exit path variant'):
        rt.ExitPath(StubRuntime(real), ANCHORS, real.policy.default[real.arm_ids],
                    'around_the_back')


def test_the_cartesian_stage_walks_the_line_to_the_default_hand_pose(real):
    stub, path, start = build(real)
    line_seconds = rt.EXIT_LINE_SECONDS
    poses = {}
    for elapsed, key in ((0.0, 'first'), (line_seconds / 2.0, 'middle'),
                         (line_seconds, 'end')):
        path.command(stub, elapsed)
        poses[key] = {s: np.asarray(v) for s, v in stub.ik_targets[-1].items()}
    for side in ('left', 'right'):
        direction = path.default_hands[side] - start[side]
        length_sq = float(np.dot(direction, direction))
        for key, expected in (('first', 0.0), ('middle', 0.5), ('end', 1.0)):
            offset = poses[key][side] - start[side]
            fraction = float(np.dot(offset, direction)) / length_sq
            assert fraction == pytest.approx(expected, abs=1e-6), (
                f'{side} at {key}: the target is {fraction:.3f} of the way along the line')
            off_line = offset - fraction * direction
            assert np.linalg.norm(off_line) < 1e-9, (
                f'{side} at {key}: the target left the straight line by '
                f'{np.linalg.norm(off_line):.3e} m')


def test_the_joint_stage_is_a_ramp_not_a_step(real):
    stub, path, _ = build(real)
    default = np.asarray(real.policy.default[real.arm_ids], dtype=float)
    path.command(stub, rt.EXIT_LINE_SECONDS)
    ramp_start = path.joints_at_ramp_start.copy()
    half = path.command(stub, rt.EXIT_LINE_SECONDS + rt.EXIT_RETURN_SECONDS / 2.0)
    home = path.command(stub, rt.EXIT_LINE_SECONDS + rt.EXIT_RETURN_SECONDS)
    assert np.allclose(home, default, atol=1e-9), 'the ramp must finish on the posture'
    assert np.allclose(half, ramp_start + 0.5 * (default - ramp_start), atol=1e-9), (
        'half way through the ramp the command should be half way home')
    rate = float(np.max(np.abs(default - ramp_start))) / rt.EXIT_RETURN_SECONDS
    assert rate <= 1.0, (f'the return moves at {rate:.3f} rad/s; a step is the old '
                        f'behaviour that moved the tray 12.0 mm')


def test_a_residual_hand_pose_does_not_end_the_path_early(real):
    """The stub's IK holds the joints at one fixed configuration on purpose.

    A Cartesian arrival leaves the arm off the posture, so the joint stage must run
    regardless of what the IK returned -- that is the whole finding of
    reports/p1-h-exitpath-line-04.
    """
    stub, path, _ = build(real)
    default = np.asarray(real.policy.default[real.arm_ids], dtype=float)
    off_posture = float(np.max(np.abs(stub.ik_result - default)))
    assert off_posture > 0.15, 'the stub is meant to sit off the posture'
    home = path.command(stub, rt.EXIT_LINE_SECONDS + rt.EXIT_RETURN_SECONDS)
    assert np.allclose(home, default, atol=1e-9)


def test_the_straight_variant_still_reproduces_the_old_behaviour(real):
    stub, _, _ = build(real)
    path = rt.ExitPath(stub, ANCHORS, real.policy.default[real.arm_ids], 'straight')
    command = path.command(stub, 0.0)
    assert np.allclose(command, real.policy.default[real.arm_ids], atol=1e-9)
    assert stub.ik_targets == [], 'the baseline must not go through IK at all'


def test_an_omitted_variant_resolves_to_the_measured_default():
    assert rt.ExitPath.resolve_variant(None) == rt.ExitPath.DEFAULT_VARIANT


def test_an_unknown_variant_names_the_ones_that_exist():
    with pytest.raises(ValueError) as excinfo:
        rt.ExitPath.resolve_variant('around_the_back')
    for variant in rt.ExitPath.VARIANTS:
        assert variant in str(excinfo.value), (
            f'{variant} is not named in the refusal, so the caller cannot tell what is legal')


def test_the_probe_help_lists_the_exit_path_option():
    """The option was once registered after the runtime import, which hid it from --help.

    `--help` is answered by argparse before the heavy import, so this stays a fast subprocess.
    """
    import subprocess
    proc = subprocess.run([sys.executable, str(ROOT / 'experiments/probe_h.py'), '--help'],
                          capture_output=True, text=True, timeout=120)
    assert proc.returncode == 0, proc.stderr
    assert '--exit-path' in proc.stdout


def test_the_probe_and_the_diagnostic_drive_the_same_class():
    """One implementation, so the comparison cannot drift away from what ships."""
    for name in ('experiments/probe_h.py', 'experiments/diag_exit_path.py'):
        text = (ROOT / name).read_text(encoding='utf-8')
        assert 'ExitPath' in text, f'{name} does not use the shared exit path'
        assert 'withdrawal_waypoints' not in text, (
            f'{name} still refers to the deleted helper; its history belongs in '
            f'docs/DECISIONS.md')
