"""P5 tests: the MuJoCo skill decomposition (section 17.3 step 3 gate).

The fast tests check the registry and world bridge. The full transport run is
marked ``slow`` (~40 s, exercises the whole 8-skill chain + independent
evaluation) and is skipped unless you opt in with ``-m slow``.
"""

from __future__ import annotations

import pytest

pytest.importorskip("mujoco", reason="simulation profile not installed (bash scripts/setup.sh --profile sim)")
pytest.importorskip("MNN", reason="simulation profile not installed")

from humanoid008.contracts import SkillName  # noqa: E402
from humanoid008.simulation.mujoco_world import MujocoWorld  # noqa: E402
from humanoid008.skills.mujoco import build_mujoco_registry  # noqa: E402

ALL_SKILLS = [
    SkillName.OBSERVE_OBJECT,
    SkillName.OBSERVE_TARGET,
    SkillName.APPROACH_OBJECT,
    SkillName.GRASP_OBJECT,
    SkillName.LIFT_OBJECT,
    SkillName.CARRY_TO_TARGET,
    SkillName.PLACE_OBJECT,
    SkillName.VERIFY_RESULT,
]


def test_mujoco_registry_has_all_eight_skills():
    world = MujocoWorld()
    try:
        registry = build_mujoco_registry(world)
        assert set(registry.names()) == set(ALL_SKILLS)
    finally:
        world.close()


def test_mujoco_world_bridge_produces_a_valid_worldstate():
    world = MujocoWorld()
    try:
        view = world.view()
        assert view.entity("box_01") is not None
        assert view.station("station_a").status.value == "FREE"
        assert view.station("station_b") is not None
        # the box starts unobserved, so it must not be visible
        assert view.entity("box_01").visible is False
    finally:
        world.close()


# --------------------------------------------------------------------------- #
# review finding 2: honest observation source + timestamps
# --------------------------------------------------------------------------- #


def test_held_box_is_labelled_proprioception_not_rgbd():
    from humanoid008.contracts import EvidenceSource

    world = MujocoWorld()
    try:
        world.holding = "box_01"
        view = world.view()
        box = view.entity("box_01")
        assert box.source is EvidenceSource.PROPRIOCEPTION
    finally:
        world.close()


def test_destination_timestamp_is_real_observation_not_faked_now():
    import numpy as np

    from humanoid008.simulation.vision import Detection

    world = MujocoWorld()
    try:
        # Simulate a destination seen at sim time 1.0, while the world clock is
        # still at its initial value. The reported observation time must be the
        # real observation time, never refreshed to "now".
        world.destination = Detection(
            center=np.array([1.23, 0.0, 0.85]),
            normal=np.array([0.0, 0.0, 1.0]),
            long_axis=np.array([0.0, 1.0, 0.0]),
            pixels=100,
            time=1.0,
            pixel_center=np.array([100.0, 100.0]),
        )
        view = world.view()
        station_b = view.station("station_b")
        assert station_b.observed_at_s == 1.0
    finally:
        world.close()


# --------------------------------------------------------------------------- #
# review findings 3 & 4: split lost protections, phase name, hold anchor
# --------------------------------------------------------------------------- #


def test_approach_phase_name_matches_locomotion_control():
    """Finding 4a: ApproachObject must use phase APPROACH (not WALK) so the
    locomotion layer's brake/precise branch actually matches its phase."""
    from humanoid008.skills.mujoco import ApproachObject

    world = MujocoWorld()
    try:
        skill = ApproachObject(world)
        assert skill._first_phase() == "APPROACH"
    finally:
        world.close()


def test_carry_records_origin_on_start():
    """Finding 3b: CARRY must record its origin so the corridor-recovery branch
    (carry_origin is not None) can fire. The split path left it None."""
    from humanoid008.skills.mujoco import CarryToTarget

    world = MujocoWorld()
    try:
        world.carry_origin = None
        skill = CarryToTarget(world)
        skill.start(world.view(), {"target_id": "station_b"})
        assert world.carry_origin is not None
    finally:
        world.close()


def test_guard_detects_stationary_base_drift():
    """Finding 3a: a hold anchor must trip BASE_DRIFT when the base drifts away."""
    import numpy as np

    from humanoid008.contracts import ReasonCode, SkillStatus
    from humanoid008.skills.mujoco import ObserveTarget

    world = MujocoWorld()
    try:
        world.hold_anchor = world.runtime.d.qpos[:2].copy() + np.array([0.5, 0.0])
        skill = ObserveTarget(world)
        assert skill._guard() is False
        assert skill.status is SkillStatus.FAILED
        assert skill.reason_code is ReasonCode.BASE_DRIFT
    finally:
        world.close()


def test_guard_detects_bilateral_contact_loss():
    """Finding 3a: carrying with lost contact must trip CONTACT_LOST after the
    sustained-loss window (0.6 s), matching the unsplit baseline. Release/retract
    phases deliberately let go, so they must NOT trip it."""
    from humanoid008.contracts import ReasonCode
    from humanoid008.skills.mujoco import CarryToTarget

    world = MujocoWorld()
    try:
        world.touch = {"left": 0.0, "right": 0.0, "source": False, "destination": False}
        skill = CarryToTarget(world)
        skill.start(world.view(), {"target_id": "station_b"})  # enters CARRY phase
        assert skill._guard() is True  # first call starts the loss timer
        world.runtime.d.time += 0.7
        assert skill._guard() is False
        assert skill.reason_code is ReasonCode.CONTACT_LOST
    finally:
        world.close()


# --------------------------------------------------------------------------- #
# review finding 4: per-station SIGNED marker installation offset
# --------------------------------------------------------------------------- #


def test_stance_marker_plate_follows_per_station_offset():
    """The marker plate/post must sit at table_centre + the per-station config
    offset, not at a hardcoded +0.75. Both a +y (B) and a -y (C) offset must be
    honoured so scene geometry, vision conversion and the planning collision
    model stay consistent (review finding 4)."""
    import json

    import numpy as np

    from humanoid008.simulation import ROOT
    from humanoid008.simulation.stance import StancePlanner

    offsets = json.loads((ROOT / "config" / "workcell.json").read_text())["marker_offset_y"]
    cases = [
        ("station_b", "destination_marker_plate", np.array([1.23, 0.0, 0.803])),
        ("station_c", "destination_c_marker_plate", np.array([1.6, -0.5, 0.803])),
    ]
    world = MujocoWorld()
    try:
        for station, plate, dest in cases:
            planner = StancePlanner(world.runtime, dest, station=station)
            got = float(planner.r.m.geom(plate).pos[1])
            want = float(dest[1] + offsets[station])
            assert abs(got - want) < 1e-6, (station, got, want)
    finally:
        world.close()


@pytest.mark.slow
def test_inspect_task_is_not_rejudged_as_transport():
    """Finding 5: an inspect task that succeeds must NOT be re-judged as a failed
    transport -- its physical_outcome stays NOT_APPLICABLE."""
    import argparse
    import shutil
    from datetime import datetime, timezone

    from humanoid008.simulation import ROOT
    from humanoid008.simulation.backend import run_skill_task

    args = argparse.Namespace(
        box_offset=[0.0, 0.0],
        target_offset=[0.0, 0.0],
        box_yaw=0.0,
        stance="auto",
        instruction="先看看箱子。",
        split=True,
    )
    run_dir = ROOT / "reports" / datetime.now(timezone.utc).strftime("test-inspect-%Y%m%dT%H%M%S%fZ")
    run_dir.mkdir(parents=True, exist_ok=False)
    try:
        report = run_skill_task(args, run_dir)
        assert report["status"] == "SUCCEEDED"
        assert report["physical_outcome"] == "NOT_APPLICABLE"
    finally:
        shutil.rmtree(run_dir, ignore_errors=True)


@pytest.mark.slow
def test_skill_split_completes_transport_with_passing_evaluation():
    """The full RulePlanner -> 8 skills -> independent-evaluation chain."""
    from humanoid008.contracts import ExecutionBackend, TaskStatus
    from humanoid008.execution import (
        BudgetLedger,
        ControlToken,
        SkillExecutor,
        TaskManager,
        load_budget_limits,
    )
    from humanoid008.execution.planning_worker import PlanningWorker
    from humanoid008.language import RuleVocabulary
    from humanoid008.planners import RulePlanner
    from humanoid008.simulation import ROOT
    from humanoid008.simulation.truth_evaluator import evaluate_transport, transport_accepted
    from humanoid008.supervision import (
        PreconditionSupervisor,
        ProposalValidator,
        RuntimeGuard,
        RuntimePolicy,
        SupervisionPolicy,
    )

    world = MujocoWorld()
    try:
        registry = build_mujoco_registry(world)
        planner = RulePlanner(registry)
        manager = TaskManager(
            runtime=world,
            registry=registry,
            planner=planner,
            executor=SkillExecutor(registry),
            vocabulary=RuleVocabulary.load(str(ROOT / "config" / "planners" / "rule.yaml")),
            execution_backend=ExecutionBackend.MUJOCO,
            validator=ProposalValidator(registry),
            precondition=PreconditionSupervisor(registry, SupervisionPolicy()),
            guard=RuntimeGuard(RuntimePolicy(), ControlToken()),
            budgets=BudgetLedger(
                load_budget_limits(str(ROOT / "config" / "supervision" / "default.yaml"))
            ),
            control=ControlToken(),
            planning_worker=PlanningWorker(planner),
        )
        outcome = manager.run_instruction("把箱子搬到 B 台。", task_id="task-p5-test")
        assert outcome.task_status is TaskStatus.SUCCEEDED
        assert [r.skill for r in outcome.history] == ALL_SKILLS
        assert transport_accepted(evaluate_transport(world.runtime))
    finally:
        world.close()
