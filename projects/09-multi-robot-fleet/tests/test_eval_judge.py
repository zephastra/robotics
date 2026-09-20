"""The evaluator must catch what the estimate-based checks cannot.

`acceptance_p4.py` decides "was this robot inside the protected region" by running the
geometry on **odometry** -- the estimate the controller acts on. An estimate-based test
cannot detect a localisation error, because it asks the robot where it is and then believes
the answer. So the tests that matter here are the ones where the two answers **differ**:

    test_truth_catches_occupancy_the_estimate_hides

is the reason this package exists. The rest pins the surrounding behaviour: attribution and
its refusal to guess, the two odom-frame conventions this project has produced, claims
cross-checked against truth, the conservative fallback, and NOT_RUN discipline.
"""

from __future__ import annotations

import json
import pathlib

import pytest
import yaml

from fleet_evaluation import judge_case, load_spawns
from fleet_core import validate_traffic_config

ROOT = pathlib.Path(__file__).resolve().parents[1]

#: spawn-frame odometry: a robot at rest reports (0,0) because the origin is its spawn.
ODOM_AT_REST = {"r01": [0.0, 0.0, 0.0], "r02": [0.0, 0.0, 0.0]}


@pytest.fixture(scope="module")
def cfg():
    return validate_traffic_config(
        yaml.safe_load((ROOT / "config" / "resources.yaml").read_text(encoding="utf-8")))


@pytest.fixture(scope="module")
def all_spawns():
    """The real spawns file: three robots today."""
    return load_spawns(ROOT / "config" / "spawns.yaml")


@pytest.fixture(scope="module")
def spawns(all_spawns):
    """Two of them, for the synthetic cases below.

    `attribute_truth` only decides an attribution when the number of truth slots equals the
    number of robots being judged -- which is correct: two slots against three robots means
    one robot is missing from the stream, and guessing which is worse than refusing.
    `test_it_attributes_three_robots_too` covers the rest.
    """
    return {rid: pose for rid, pose in all_spawns.items() if rid in ("r01", "r02")}


def _resources(owner=None):
    return {
        name: {"state": "FREE" if owner is None else "RESERVED", "owner": owner, "queue": []}
        for name in ("mid", "mid_left", "mid_right")
    }


def make_case(tmp_path, samples, drivers=()):
    case = tmp_path / "case"
    case.mkdir(parents=True, exist_ok=True)
    (case / "samples-x.jsonl").write_text(
        "\n".join(json.dumps(s) for s in samples), encoding="utf-8")
    for index, driver in enumerate(drivers):
        (case / f"driver{index}.json").write_text(json.dumps(driver), encoding="utf-8")
    return case


def check_of(result, name):
    for check in result["checks"]:
        if check["check"] == name:
            return check
    raise AssertionError(f"{name} missing; got {[c['check'] for c in result['checks']]}")


def sample(t, odom, truth, resources=None):
    """Truth carries a heading, the way fleet_evaluation.recorder writes it."""
    body = {"t": t, "odom": odom, "truth": {str(k): list(v) for k, v in truth.items()}}
    if resources is not None:
        body["resources"] = resources
    return body


def at_rest(t, resources=None):
    """The sample every recording starts with: both robots on their spawns."""
    return sample(t, ODOM_AT_REST, {0: (-6.0, -2.0, 0.0), 1: (6.0, -2.0, 0.0)}, resources)


def r01_claiming(x, y):
    """Spawn-frame odometry that composes to map (x, y). r01 spawns at (-6,-2) with yaw 0."""
    return [x + 6.0, y + 2.0, 0.0]


# --------------------------------------------------------------------------- #
# the reason the package exists
# --------------------------------------------------------------------------- #

def test_truth_catches_occupancy_the_estimate_hides(tmp_path, cfg, spawns):
    """Both robots are inside the corridor per truth; per odometry they are still at home.

    That is not contrived -- it is what a localisation error looks like from the fleet's
    side. Odometry says each robot is sitting quietly on its spawn, so the estimator-based
    containment test reports nothing, and only the world's own stream shows two robots in
    the gap at the same instant.
    """
    samples = [
        at_rest(0.0),
        sample(0.2, ODOM_AT_REST, {0: (0.0, 0.0, 0.0), 1: (0.5, 0.2, 0.0)}),
        sample(0.4, ODOM_AT_REST, {0: (-0.3, 0.1, 0.0), 1: (0.4, -0.1, 0.0)}),
    ]
    result = judge_case(make_case(tmp_path, samples), cfg, spawns)

    simultaneous = check_of(result, "no_simultaneous_occupancy_from_truth")
    assert simultaneous["verdict"] == "FAIL", simultaneous
    assert simultaneous["samples"] == 2, simultaneous

    agreement = check_of(result, "estimate_and_truth_agree_on_containment")
    assert agreement["verdict"] == "FAIL", agreement
    assert agreement["disagreements"] == 4, agreement

    assert result["summary"] == "FAIL"


def test_a_clean_run_passes_from_truth(tmp_path, cfg, spawns):
    """One robot walks the corridor alone; the other stays outside it.

    The odometry is written so that composing it out of the spawn frame lands exactly on the
    truth position. An inconsistent pair here would measure the test, not the judge.
    """
    samples = [at_rest(0.0, _resources(owner="r01"))]
    for index in range(1, 6):
        x = -6.0 + index * 2.0                      # r01 walks west to east along y = 0
        samples.append(sample(index * 0.2,
                              {"r01": r01_claiming(x, 0.0), "r02": [0.0, 0.0, 0.0]},
                              {0: (x, 0.0, 0.0), 1: (6.0, -2.0, 0.0)},
                              _resources(owner="r01")))
    result = judge_case(make_case(tmp_path, samples), cfg, spawns)
    assert check_of(result, "no_simultaneous_occupancy_from_truth")["verdict"] == "PASS"
    assert check_of(result, "no_unauthorised_entry_from_truth")["verdict"] == "PASS"
    assert check_of(result, "estimate_and_truth_agree_on_containment")["disagreements"] == 0
    assert check_of(result, "pose_estimate_error")["per_robot"]["r01"]["p95_m"] == 0.0
    # NOT the summary. This recording carries no driver reports, so the claim cross-check is
    # handed nothing to corroborate and says so; a check that examined nothing is NOT_RUN, and
    # the summary follows it. The safety properties above are what this test is about, and they
    # are asserted individually. Written the other way round this test asserted that "I looked
    # at nothing" and "everything agreed" have the same verdict.
    assert check_of(result, "driver_claims_corroborated_by_truth")["verdict"] == "NOT_RUN"
    assert result["summary"] == "NOT_RUN", (
        "the safety properties passed and the claim cross-check did not happen; a summary of "
        "PASS here would report the second as though it were the first")


def test_truth_sees_an_entry_the_book_calls_unauthorised(tmp_path, cfg, spawns):
    samples = [
        at_rest(0.0, _resources(owner=None)),
        sample(0.2, {"r01": r01_claiming(0.0, 0.0), "r02": [0.0, 0.0, 0.0]},
               {0: (0.0, 0.0, 0.0), 1: (6.0, -2.0, 0.0)}, _resources(owner=None)),
    ]
    result = judge_case(make_case(tmp_path, samples), cfg, spawns)
    unauthorised = check_of(result, "no_unauthorised_entry_from_truth")
    assert unauthorised["verdict"] == "FAIL", unauthorised
    assert unauthorised["samples"] >= 1


def test_a_region_with_an_owner_is_not_an_unauthorised_entry(tmp_path, cfg, spawns):
    samples = [
        at_rest(0.0, _resources(owner="r01")),
        sample(0.2, {"r01": r01_claiming(0.0, 0.0), "r02": [0.0, 0.0, 0.0]},
               {0: (0.0, 0.0, 0.0), 1: (6.0, -2.0, 0.0)}, _resources(owner="r01")),
    ]
    result = judge_case(make_case(tmp_path, samples), cfg, spawns)
    assert check_of(result, "no_unauthorised_entry_from_truth")["verdict"] == "PASS"


# --------------------------------------------------------------------------- #
# frames: the mistake this project has made four times
# --------------------------------------------------------------------------- #

def test_the_odom_claim_is_composed_out_of_its_spawn_frame(tmp_path, cfg, spawns):
    """Odometry starts at zero at the spawn pose, and comparing it raw reads (0,0) as the
    map origin, which the corridor contains, so a stationary robot looks like an intruder."""
    samples = [at_rest(i * 0.2, _resources(owner=None)) for i in range(3)]
    result = judge_case(make_case(tmp_path, samples), cfg, spawns)
    assert check_of(result, "odom_frame_established")["frame"] == "spawn"
    errors = check_of(result, "pose_estimate_error")
    assert errors["verdict"] == "PASS", errors
    assert errors["per_robot"]["r01"]["p95_m"] == pytest.approx(0.0, abs=1e-9)
    assert errors["per_robot"]["r02"]["p95_m"] == pytest.approx(0.0, abs=1e-9)


def test_odom_already_in_the_map_frame_is_not_composed_a_second_time(tmp_path, cfg, spawns):
    """The P4 sampler composed before storing; `fleet_recorder` does not. Both are formats,
    and applying the wrong one shifts every robot by a whole spawn offset -- which reads as
    several metres of localisation error rather than as a format mistake."""
    map_frame = {"r01": [-6.0, -2.0, 0.0], "r02": [6.0, -2.0, 0.0]}
    samples = [sample(i * 0.2, map_frame,
                      {0: (-6.0, -2.0, 0.0), 1: (6.0, -2.0, 0.0)}, _resources(owner=None))
               for i in range(3)]
    result = judge_case(make_case(tmp_path, samples), cfg, spawns)
    frame_check = check_of(result, "odom_frame_established")
    assert frame_check["verdict"] == "PASS", frame_check
    assert frame_check["frame"] == "map"
    errors = check_of(result, "pose_estimate_error")
    assert errors["per_robot"]["r01"]["p95_m"] == pytest.approx(0.0, abs=1e-9)


def test_the_frame_can_be_forced_when_the_data_is_ambiguous(tmp_path, cfg, spawns):
    """Halfway between the spawns and the origin is not decidable from the data."""
    odd = {"r01": [3.0, 1.0, 0.0], "r02": [3.0, -1.0, 0.0]}
    samples = [sample(i * 0.2, odd,
                      {0: (-6.0, -2.0, 0.0), 1: (6.0, -2.0, 0.0)}, _resources(owner=None))
               for i in range(3)]
    auto = judge_case(make_case(tmp_path, samples), cfg, spawns)
    assert check_of(auto, "odom_frame_established")["verdict"] == "NOT_RUN"
    forced = judge_case(make_case(tmp_path / "forced", samples), cfg, spawns,
                        odom_frame="map")
    assert check_of(forced, "odom_frame_established")["frame"] == "map"


def test_a_constant_pose_offset_is_reported_as_error(tmp_path, cfg, spawns):
    samples = [at_rest(0.0, _resources(owner=None))]
    samples += [
        sample(i * 0.2, {"r01": r01_claiming(-6.0 + i, -2.0), "r02": [0.0, 0.0, 0.0]},
               {0: (-6.0, -2.0, 0.0), 1: (6.0, -2.0, 0.0)}, _resources(owner=None))
        for i in range(1, 3)
    ]
    result = judge_case(make_case(tmp_path, samples), cfg, spawns)
    errors = check_of(result, "pose_estimate_error")
    assert errors["per_robot"]["r01"]["p95_m"] > 0.35
    assert errors["verdict"] == "FAIL", "a drifting estimate must not pass a 0.35 m limit"


# --------------------------------------------------------------------------- #
# attribution
# --------------------------------------------------------------------------- #

def test_attribution_refuses_when_the_spawns_are_too_close_to_tell_apart(tmp_path, cfg):
    """A mislabelled slot swaps two trajectories and every number becomes about the wrong
    robot, so an undecidable attribution is NOT_RUN rather than a guess."""
    ambiguous = {"r01": (5.0, 0.0, 0.0), "r02": (5.3, 0.0, 0.0)}
    samples = [sample(0.0, ODOM_AT_REST, {0: (5.0, 0.0, 0.0), 1: (5.3, 0.0, 0.0)},
                      _resources(owner=None))]
    result = judge_case(make_case(tmp_path, samples), cfg, ambiguous)
    attribution = check_of(result, "truth_attribution")
    assert attribution["verdict"] == "NOT_RUN", attribution
    assert result["summary"] == "NOT_RUN"


def test_attribution_reports_its_margin_and_continuity(tmp_path, cfg, spawns):
    samples = [at_rest(i * 0.2, _resources(owner=None)) for i in range(3)]
    result = judge_case(make_case(tmp_path, samples), cfg, spawns)
    attribution = check_of(result, "truth_attribution")
    assert attribution["verdict"] == "PASS", attribution
    assert attribution["margin_m"] > 5.0
    assert attribution["continuity_violations"] == 0
    assert attribution["seed_distances_m"] == {"r01": 0.0, "r02": 0.0}


def test_a_recording_that_starts_mid_motion_cannot_be_attributed(tmp_path, cfg, spawns):
    """The seeding rule needs the first sample to be at rest on the spawns.

    That is a stated precondition, not an accident: slots carry no names, so identity is
    established by where the robots started. A recording that begins after they have moved
    is refused rather than attributed by guessing, and the reason says which numbers made it
    undecidable.
    """
    samples = [sample(0.0, ODOM_AT_REST, {0: (0.0, 0.0, 0.0), 1: (0.5, 0.2, 0.0)},
                      _resources(owner=None))]
    result = judge_case(make_case(tmp_path, samples), cfg, spawns)
    attribution = check_of(result, "truth_attribution")
    assert attribution["verdict"] == "NOT_RUN", attribution
    assert "no slot starts on the spawn of" in attribution["reason"]
    assert "moved 0.000 m in total" in attribution["reason"], (
        "the reason must name the slot that never moved, not just say NOT_RUN")
    assert result["summary"] == "NOT_RUN"


def test_a_robot_absent_from_the_recording_is_reported_not_silently_dropped(tmp_path, cfg,
                                                                          all_spawns):
    """A two-robot recording judged against a three-robot config must say what it judged."""
    samples = [at_rest(i * 0.2, _resources(owner=None)) for i in range(3)]
    result = judge_case(make_case(tmp_path, samples), cfg, all_spawns)
    assert result["robots_judged"] == ["r01", "r02"]
    assert result["robots_absent"] == ["r03"]
    assert check_of(result, "truth_attribution")["verdict"] == "PASS"


def test_it_attributes_three_robots_too(tmp_path, cfg, all_spawns):
    """The P4 sampler hard-coded two slots, so a third robot would have been missed."""
    three = [
        sample(0.0, {r: [0, 0, 0] for r in all_spawns},
               {0: (-6.0, -2.0, 0.0), 1: (6.0, -2.0, 0.0), 2: (-6.0, 0.6, 0.0)},
               _resources(owner=None)),
    ]
    result = judge_case(make_case(tmp_path, three), cfg, all_spawns)
    attribution = check_of(result, "truth_attribution")
    assert attribution["verdict"] == "PASS", attribution
    assert attribution["seed_distances_m"] == {"r01": 0.0, "r02": 0.0, "r03": 0.0}

    partial = judge_case(
        make_case(tmp_path / "partial",
                  [sample(0.0, {r: [0, 0, 0] for r in all_spawns},
                          {0: (-6.0, -2.0, 0.0), 1: (6.0, -2.0, 0.0)},
                          _resources(owner=None))]),
        cfg, all_spawns)
    assert check_of(partial, "truth_attribution")["verdict"] == "NOT_RUN"
    assert partial["summary"] == "NOT_RUN"


# --------------------------------------------------------------------------- #
# claims are cross-checked, never believed
# --------------------------------------------------------------------------- #

def test_a_driver_claiming_success_without_crossing_is_contradicted(tmp_path, cfg, spawns):
    samples = [at_rest(i * 0.2, _resources(owner=None)) for i in range(3)]
    case = make_case(tmp_path, samples, drivers=[
        {"robot": "r01", "direction": "west_to_east", "complete": True},
        {"robot": "r02", "direction": "east_to_west", "complete": False},
    ])
    result = judge_case(case, cfg, spawns)
    claims = check_of(result, "driver_claims_corroborated_by_truth")
    assert claims["verdict"] == "FAIL", claims
    assert [c["robot"] for c in claims["contradicted"]] == ["r01"]
    assert claims["corroborated"][0]["robot"] == "r02"
    assert claims["corroborated"][0]["verdict"] == "NOT_CLAIMED"


def test_a_driver_that_really_crossed_is_corroborated(tmp_path, cfg, spawns):
    samples = [
        at_rest(0.0, _resources(owner=None)),
        sample(0.2, {"r01": r01_claiming(0.0, -1.4), "r02": [0.0, 0.0, 0.0]},
               {0: (0.0, -1.4, 0.0), 1: (6.0, -2.0, 0.0)}, _resources(owner="r01")),
        sample(0.4, {"r01": r01_claiming(4.2, -1.4), "r02": [0.0, 0.0, 0.0]},
               {0: (4.2, -1.4, 0.0), 1: (6.0, -2.0, 0.0)}, _resources(owner="r01")),
    ]
    case = make_case(tmp_path, samples, drivers=[
        {"robot": "r01", "direction": "west_to_east", "complete": True}])
    result = judge_case(case, cfg, spawns)
    claims = check_of(result, "driver_claims_corroborated_by_truth")
    assert claims["verdict"] == "PASS", claims
    entry = claims["corroborated"][0]
    assert entry["verdict"] == "CORROBORATED"
    assert entry["truth_crossed_barrier"] is True
    assert entry["truth_travelled_m"] >= 2.0


# --------------------------------------------------------------------------- #
# the conservative fallback, stated rather than hidden
# --------------------------------------------------------------------------- #

def test_a_recording_without_truth_heading_says_which_test_it_used(tmp_path, cfg, spawns):
    """The P4 recordings have no truth yaw, so containment cannot use the real footprint.

    Two consequences, both visible in the output: the safety checks are conservative (so a
    PASS is stronger than usual), and the estimate-versus-truth comparison is NOT_RUN --
    comparing a conservative test against an exact one measures the tests, not the
    localisation.
    """
    samples = [
        {"t": i * 0.2, "odom": ODOM_AT_REST,
         "truth": {"0": [-6.0, -2.0], "1": [6.0, -2.0]},
         "resources": _resources(owner=None)}
        for i in range(3)
    ]
    result = judge_case(make_case(tmp_path, samples), cfg, spawns)
    assert result["truth_has_heading"] is False
    agreement = check_of(result, "estimate_and_truth_agree_on_containment")
    assert agreement["verdict"] == "NOT_RUN", agreement
    assert agreement["geometry"] == "conservative"
    assert "cannot be asked of this recording" in agreement["detail"]
    for name in ("no_simultaneous_occupancy_from_truth", "no_unauthorised_entry_from_truth"):
        check = check_of(result, name)
        assert check["geometry"] == "conservative", check
        assert "conservative" in check["detail"]
    assert result["summary"] == "NOT_RUN"


# --------------------------------------------------------------------------- #
# NOT_RUN discipline
# --------------------------------------------------------------------------- #

def test_no_samples_is_not_run_never_pass(tmp_path, cfg, spawns):
    case = tmp_path / "empty"
    case.mkdir()
    result = judge_case(case, cfg, spawns)
    assert result["summary"] == "NOT_RUN"
    assert check_of(result, "samples_present")["verdict"] == "NOT_RUN"


def test_without_truth_there_is_no_truth_based_verdict(tmp_path, cfg, spawns):
    """A run that recorded only the fleet's own numbers cannot be judged independently.

    The tempting failure is to fall back to the odometry and report a PASS. That is exactly
    the substitution this package exists to refuse.
    """
    samples = [{"t": i * 0.2, "odom": ODOM_AT_REST, "truth": {},
                "resources": _resources(owner=None)} for i in range(3)]
    result = judge_case(make_case(tmp_path, samples), cfg, spawns)
    assert check_of(result, "truth_attribution")["verdict"] == "NOT_RUN"
    assert check_of(result, "pose_estimate_error")["verdict"] == "NOT_RUN"
    assert result["summary"] == "NOT_RUN"


def test_a_book_less_run_cannot_judge_authorisation(tmp_path, cfg, spawns):
    samples = [at_rest(i * 0.2) for i in range(3)]
    result = judge_case(make_case(tmp_path, samples), cfg, spawns)
    assert check_of(result, "no_unauthorised_entry_from_truth")["verdict"] == "NOT_RUN"
