"""Stage criteria for the H tray task, kept out of the simulator.

The probe measures; this module judges. Splitting them means the criteria can be
exercised with synthetic samples, so every verdict -- PASS, FAIL and NOT_RUN -- can be
shown to be reachable. A threshold set that has only ever returned PASS is not evidence.

The values in THRESHOLDS are DIAGNOSTIC. V1 freezes its own set only after the V1 tray
model and the real work-cell geometry exist (TEST_AND_ACCEPTANCE; open items O03/O05).
The probe and the tests both read this one dict, so they cannot drift apart.

Sample shape expected from the probe:

    {"t": float,                       # sim seconds
     "payload_z": float,               # tray origin height, metres
     "payload_speed": float,           # tray linear speed, m/s
     "hand_contacts": {"left": int, "right": int},
     "arm_dev_rad": float}             # max |arm joint - default posture|, radians
"""

#: Diagnostic thresholds for the carried-over 007 handled tray.
THRESHOLDS = {
    # The tray bottom must clearly leave the support, not buzz against it.
    # 0.05 m is well above the 0.002 m physics step's contact penetration noise.
    "lift_clearance_m": 0.05,
    # Both hands must carry it while airborne for this long, unbroken. A single
    # sample of contact is a collision, not a hold.
    "hold_seconds": 2.0,
    # Contacts required on EACH side. One hand alone is not a two-handed grasp.
    "hand_contact_min": 1,
    # "Back on the support" tolerance. The support is a rigid table and the tray is a
    # rigid box, so this is tight on purpose.
    "place_band_m": 0.010,
    # "Stayed put" after release.
    "rest_speed_mps": 0.010,
    "rest_seconds": 0.5,
    # Arms back near the default posture = hands out of the shared work zone.
    "exit_arm_rad": 0.15,
    # Leaving the zone must not disturb what was just placed. The tray is a free rigid
    # body resting on a table with friction 1.0, so a clean withdrawal moves it by
    # millimetres at most. 5 mm and 20 mm/s are DIAGNOSTIC; V1 freezes its own values
    # once the real work-cell geometry exists (O05).
    "exit_tray_travel_m": 0.005,
    "exit_tray_speed_mps": 0.02,
}

#: The six H stages, in the order the plan lists them.
STAGES = ("grasp", "lift", "hold", "place", "release", "exit")

STAGE_LABEL = {
    "grasp": "双手抓持",
    "lift": "离台",
    "hold": "稳定持有",
    "place": "放下",
    "release": "松手后稳定",
    "exit": "退出操作区",
}

_PASS, _FAIL, _NOT_RUN = "PASS", "FAIL", "NOT_RUN"


def _both_hands(sample, minimum):
    contacts = sample["hand_contacts"]
    return contacts["left"] >= minimum and contacts["right"] >= minimum


def _last_contact_index(samples):
    """Index of the last sample with any hand-tray contact, or None.

    Both `release` and `exit` anchor here. Anchoring on the *first* sample with no contact
    lands in the approach phase, before the grasp, so a healthy run passes for the wrong
    reason -- that bug was live in the first version of both checks.
    """
    last = None
    for index, sample in enumerate(samples):
        contacts = sample["hand_contacts"]
        if contacts["left"] or contacts["right"]:
            last = index
    return last


def _longest_run_seconds(samples, predicate, start_index=0):
    """Longest unbroken stretch (seconds) where `predicate` holds.

    Uses sample timestamps, so it is independent of the sampling period.
    """
    best, run_start = 0.0, None
    for index in range(start_index, len(samples)):
        if predicate(samples[index]):
            if run_start is None:
                run_start = samples[index]["t"]
            best = max(best, samples[index]["t"] - run_start)
        else:
            run_start = None
    return best


def _verdict(ok, detail, numbers):
    return {"verdict": _PASS if ok else _FAIL, "detail": detail, "numbers": numbers}


def _check_grasp(samples, z0, phases):
    minimum = THRESHOLDS["hand_contact_min"]
    first = next((i for i, s in enumerate(samples) if _both_hands(s, minimum)), None)
    if first is None:
        peak = max((max(s["hand_contacts"].values()) for s in samples), default=0)
        return _verdict(False,
                        f"both hands never made contact at the same time "
                        f"(most contacts on one side at any instant: {peak})",
                        {"first_both_hands_t": None, "peak_one_side_contacts": peak})
    left = samples[first]["hand_contacts"]["left"]
    right = samples[first]["hand_contacts"]["right"]
    return _verdict(True,
                    f"both hands in contact at t={samples[first]['t']:.2f} s "
                    f"(left {left}, right {right} contacts)",
                    {"first_both_hands_t": samples[first]["t"],
                     "left": left, "right": right})


def _check_lift(samples, z0, phases):
    clearance = THRESHOLDS["lift_clearance_m"]
    peak = max((s["payload_z"] for s in samples), default=z0)
    rise = peak - z0
    return _verdict(rise >= clearance,
                    f"tray rose {rise:+.4f} m above its support height "
                    f"(need >= {clearance:.3f} m)",
                    {"rise_m": rise, "peak_z": peak, "z0": z0})


def _check_hold(samples, z0, phases):
    clearance = THRESHOLDS["lift_clearance_m"]
    minimum = THRESHOLDS["hand_contact_min"]
    required = THRESHOLDS["hold_seconds"]
    longest = _longest_run_seconds(
        samples, lambda s: _both_hands(s, minimum) and (s["payload_z"] - z0) >= clearance)
    return _verdict(longest >= required,
                    f"both hands carried the tray airborne for {longest:.2f} s unbroken "
                    f"(need >= {required:.1f} s)",
                    {"held_seconds": longest, "required_seconds": required})


def _support_band(z0):
    return THRESHOLDS["place_band_m"]


def _peak_index(samples, z0):
    return max(range(len(samples)), key=lambda i: samples[i]["payload_z"])


def _check_place(samples, z0, phases):
    band = _support_band(z0)
    peak = _peak_index(samples, z0)
    after = samples[peak:]
    if not after:
        return _verdict(False, "no samples after the lift peak", {"closest_m": None})
    minimum = THRESHOLDS["hand_contact_min"]
    best, hit = None, None
    for sample in after:
        gap = abs(sample["payload_z"] - z0)
        if best is None or gap < best[0]:
            best = (gap, sample["t"])
        if gap <= band and _both_hands(sample, minimum):
            hit = sample
            break
    if hit is not None:
        return _verdict(True,
                        f"tray back within {band * 1000:.0f} mm of its support at "
                        f"t={hit['t']:.2f} s with both hands still on it",
                        {"placed_t": hit["t"], "gap_m": abs(hit["payload_z"] - z0)})
    return _verdict(False,
                    f"tray never returned within {band * 1000:.0f} mm of its support while "
                    f"held (closest {best[0] * 1000:.1f} mm at t={best[1]:.2f} s)",
                    {"closest_m": best[0], "closest_t": best[1]})


def _check_release(samples, z0, phases):
    """Judged after the LAST hand contact: hands off, tray on its support, and still.

    The window starts after the last contact rather than at the first sample with no
    contact, because the opening samples of every run have no contact either.
    """
    band = _support_band(z0)
    max_speed = THRESHOLDS["rest_speed_mps"]
    required = THRESHOLDS["rest_seconds"]
    last_contact = _last_contact_index(samples)
    if last_contact is None:
        return _verdict(False, "no hand contact was ever recorded, so nothing was released",
                        {"cleared_t": None, "settled_seconds": 0.0,
                         "speed_at_clear_mps": None})
    window = samples[last_contact + 1:]
    if not window:
        return _verdict(False, "the run ended while the hands were still on the tray",
                        {"cleared_t": None, "settled_seconds": 0.0,
                         "speed_at_clear_mps": None})
    cleared = None
    for index, sample in enumerate(window):
        if abs(sample["payload_z"] - z0) <= band:
            cleared = index
            break
    if cleared is None:
        return _verdict(False,
                        f"after letting go the tray never settled within "
                        f"{band * 1000:.0f} mm of its support",
                        {"cleared_t": window[0]["t"], "settled_seconds": 0.0,
                         "speed_at_clear_mps": None})
    settled = _longest_run_seconds(window,
                                  lambda s: s["payload_speed"] < max_speed,
                                  start_index=cleared)
    number = {"cleared_t": window[cleared]["t"],
              "settled_seconds": settled,
              "speed_at_clear_mps": window[cleared]["payload_speed"]}
    return _verdict(settled >= required,
                    f"hands off at t={window[cleared]['t']:.2f} s and the tray then stayed "
                    f"below {max_speed:.3f} m/s for {settled:.2f} s "
                    f"(need >= {required:.1f} s)",
                    number)


def _empty_exit_numbers(cleared_t=None):
    return {"cleared_t": cleared_t, "reached_t": None, "final_arm_dev_rad": None,
            "tray_travel_m": None, "tray_peak_speed_mps": None}


def _check_exit(samples, z0, phases):
    """Judged from the LAST hand contact onwards: the arms leave AND the tray stays put.

    Three traps this avoids:
      1. Anchoring on the first sample with no contact lands in the approach phase.
      2. Demanding the arm limit at every instant fails while the arms are still
         legitimately travelling back; what matters is that they arrive and then stay.
      3. Checking only the arms ignores what they did on the way out. Withdrawing brushed
         the tray, which slid 13 mm -- an exit that moves what was just placed is not an
         exit, so the tray's own travel and peak speed are bounded too.
    """
    if samples and ("payload_x" not in samples[0] or "payload_y" not in samples[0]):
        return _verdict(False, "the recording carries no tray position, so the "
                               "withdrawal cannot be judged", _empty_exit_numbers())
    limit = THRESHOLDS["exit_arm_rad"]
    travel_limit = THRESHOLDS["exit_tray_travel_m"]
    speed_limit = THRESHOLDS["exit_tray_speed_mps"]
    last_contact = _last_contact_index(samples)
    if last_contact is None:
        return _verdict(False, "no hand contact was ever recorded, so there is no "
                               "withdrawal to judge", _empty_exit_numbers())
    window = samples[last_contact + 1:]
    if not window:
        return _verdict(False, "the run ended while the hands were still on the tray",
                        _empty_exit_numbers())
    clearance = samples[last_contact]
    travel = max(((s["payload_x"] - clearance["payload_x"]) ** 2
                  + (s["payload_y"] - clearance["payload_y"]) ** 2) ** 0.5
                 for s in window)
    peak_speed = max(s["payload_speed"] for s in window)
    numbers = {"cleared_t": window[0]["t"], "reached_t": None,
               "final_arm_dev_rad": None, "tray_travel_m": travel,
               "tray_peak_speed_mps": peak_speed}

    reached = next((i for i, s in enumerate(window) if s["arm_dev_rad"] <= limit), None)
    if reached is None:
        closest = min(s["arm_dev_rad"] for s in window)
        numbers["final_arm_dev_rad"] = closest
        return _verdict(False,
                        f"after letting go the arms never came back within {limit:.2f} rad "
                        f"of the default posture (closest {closest:.3f} rad)",
                        numbers)
    tail = window[reached:]
    if max(s["arm_dev_rad"] for s in tail) > limit:
        numbers["final_arm_dev_rad"] = tail[-1]["arm_dev_rad"]
        return _verdict(False,
                        f"the arms came back within {limit:.2f} rad and then left the "
                        f"posture again", numbers)
    numbers["reached_t"] = tail[0]["t"]
    numbers["final_arm_dev_rad"] = tail[-1]["arm_dev_rad"]

    if travel > travel_limit:
        return _verdict(False,
                        f"withdrawing moved the tray {travel * 1000:.1f} mm from where it "
                        f"was placed (limit {travel_limit * 1000:.0f} mm; peak speed "
                        f"{peak_speed:.4f} m/s)", numbers)
    if peak_speed > speed_limit:
        return _verdict(False,
                        f"withdrawing knocked the tray to {peak_speed:.4f} m/s "
                        f"(limit {speed_limit:.3f} m/s; travel {travel * 1000:.1f} mm)",
                        numbers)
    return _verdict(True,
                    f"arms within {limit:.2f} rad of the default posture from "
                    f"t={numbers['reached_t']:.2f} s, settling to "
                    f"{numbers['final_arm_dev_rad']:.4f} rad; the tray moved "
                    f"{travel * 1000:.2f} mm at most and never exceeded "
                    f"{peak_speed:.4f} m/s",
                    numbers)


_CHECKS = {
    "grasp": _check_grasp,
    "lift": _check_lift,
    "hold": _check_hold,
    "place": _check_place,
    "release": _check_release,
    "exit": _check_exit,
}


def evaluate(samples, phases_performed, payload_z_initial):
    """Judge every H stage. `phases_performed` names the stages the run actually drove.

    A stage the run did not drive is NOT_RUN, never PASS. That distinction is the whole
    point: a run that only stands up has not demonstrated a release.
    """
    stages = {}
    for name in STAGES:
        if name not in phases_performed:
            stages[name] = {"verdict": _NOT_RUN,
                            "detail": f"the run did not drive the {name} phase "
                                      f"({STAGE_LABEL[name]})",
                            "numbers": {}}
            continue
        if not samples:
            stages[name] = {"verdict": _NOT_RUN,
                            "detail": "no samples were recorded",
                            "numbers": {}}
            continue
        stages[name] = _CHECKS[name](samples, payload_z_initial, phases_performed)

    failed = [n for n in STAGES if stages[n]["verdict"] == _FAIL]
    not_run = [n for n in STAGES if stages[n]["verdict"] == _NOT_RUN]
    if failed:
        overall = (f"FAIL: {len(failed)} of {len(STAGES)} stages failed "
                   f"({', '.join(failed)})")
    elif not_run:
        overall = (f"NOT_RUN: {len(not_run)} of {len(STAGES)} stages were not driven "
                   f"({', '.join(not_run)})")
    else:
        overall = "PASS: all six stages"
    return {"stages": stages, "overall": overall,
            "failed": failed, "not_run": not_run,
            "thresholds": dict(THRESHOLDS)}


def summary_line(result):
    """One line per stage, for the console and for the report."""
    lines = []
    for name in STAGES:
        entry = result["stages"][name]
        lines.append(f"{STAGE_LABEL[name]:<10} {entry['verdict']:<8} {entry['detail']}")
    lines.append(f"{'总计':<10} {result['overall']}")
    return lines
