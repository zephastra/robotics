"""Judge for PROBE-A: did a fixed arm actually pick the part up and put it down?

Pure Python over `report.json` plus the per-run trace files. No physics here, so the criteria
can be exercised with synthetic inputs -- including inputs that MUST fail, because a check that
cannot be shown to fail is not a check.

The required evidence is the sentence from `docs/TEST_AND_ACCEPTANCE.md`:

    | 必需证据 | 夹爪接触、物体离台、释放后稳定；规划附着不能充当接触 |

which decomposes into: gripper contact (3), the part leaving the table (4), stability after
release (7), and -- the one that is easy to fake -- nothing attaching the part to the gripper
(2). A planned `attach`, a weld, or an equality would all let a machine "succeed" without ever
touching the part, so check 2 reads `eq_obj*id` out of the compiled model rather than trusting
the report, and check 3 counts SUSTAINED contact rather than one sample.
"""
import argparse
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
REPORTS = ROOT / 'reports'


def evaluate(report, trace_dir, run_id):
    checks = []

    def check(name, verdict, detail):
        checks.append({'check': name, 'verdict': verdict, 'detail': detail})

    runs = report.get('runs') or []
    thresholds = report.get('thresholds') or {}
    if not runs:
        check('the report contains runs', 'FAIL', 'report.json has no runs[] to judge')
        return _finish(checks, run_id, report)

    def load_records(run):
        name = run.get('trace_file')
        if not name:
            return None
        path = pathlib.Path(trace_dir) / name
        if not path.is_file():
            return None
        return json.loads(path.read_text(encoding='utf-8')).get('records')

    rest_z = float(report['table_top_z']) + float(report['payload_half'][2])
    place_xy = tuple(float(v) for v in report['place_xy'])
    pick_xy = tuple(float(v) for v in report['pick_xy'])
    required = list(report.get('requires') or [])
    min_contact = int(thresholds.get('min_contact_samples', 60))
    lift_margin = float(thresholds.get('lift_margin_m', 0.05))
    slip_max = float(thresholds.get('carried_slip_max_m', 0.02))
    place_tol = float(thresholds.get('place_tol_xy_m', 0.03))
    drift_max = float(thresholds.get('release_drift_max_m', 0.006))
    speed_max = float(thresholds.get('release_speed_max_mps', 0.02))
    touchdown_tol = float(thresholds.get('touchdown_tol_m', 0.006))

    def in_envelope(r):
        """Is this run part of the set the skill is CLAIMED to work in?

        Runs outside it are still executed and still reported -- a limit that is never
        exercised is an assumption -- but they are not counted as successes. That is the same
        distinction the C branch draws between a justified refusal and conservatism.
        """
        return (r.get('scenario') or {}).get('in_envelope', True)

    def expected_of(r):
        return (r.get('scenario') or {}).get('expect', 'DONE')

    good_runs = [r for r in runs if in_envelope(r) and expected_of(r) == 'DONE']
    bad_runs = [r for r in runs if expected_of(r) != 'DONE']
    outside_runs = [r for r in runs
                    if not in_envelope(r) and expected_of(r) == 'DONE']

    # -- 1. the part is the declared one, at the declared start --------------------------
    bad_start = []
    for r in runs:
        got = r.get('initial_payload')
        if not got:
            bad_start.append(f'{r["scenario"]["name"]}: no initial pose recorded')
            continue
        off = r.get('part_offset') or [0.0, 0.0]
        want = [pick_xy[0] + off[0], pick_xy[1] + off[1], rest_z]
        if max(abs(a - b) for a, b in zip(got, want)) > 1e-6:
            bad_start.append(f'{r["scenario"]["name"]}: started at {[round(v, 5) for v in got]} '
                             f'not {[round(v, 5) for v in want]}')
    check('the object under test starts as the declared part',
          'PASS' if not bad_start else 'FAIL',
          f'a {[round(v * 1000, 1) for v in report["payload_half"]]} mm half-extent part of '
          f'{report["payload_mass"]} kg at pick xy {pick_xy}, resting at z {rest_z:.4f} '
          f'(table top {report["table_top_z"]} + half the part). '
          + ('Every run started there.' if not bad_start else '; '.join(bad_start)))

    # -- 1b. the perturbation set is real ----------------------------------------------
    offsets = sorted({tuple(r.get('part_offset') or (0.0, 0.0)) for r in good_runs})
    if len(offsets) < 2:
        check('the perturbation set is more than one pose', 'NOT_RUN',
              f'every admitted run started the part at the same place ({offsets}), so this '
              f'report says nothing about repeatability under a displaced part. A single pose '
              f'repeated is a determinism check, not a robustness one')
    else:
        check('the perturbation set is more than one pose',
              'PASS' if len(offsets) >= 3 else 'FAIL',
              f'{len(offsets)} distinct part placements were run: '
              f'{[list(o) for o in offsets]}. A claim of repeatability built on one pose is '
              f'not a claim; three is the minimum this report will credit.')

    # -- 2. nothing attaches the payload (the anti-fake-attach check) --------------------
    # Load the project's world file, not the copy in the report directory: `meshdir` is
    # relative to `assets/`, so the copy cannot be compiled where it sits. Loading the real
    # file and checking its hash against the report is also the stronger test -- if the model
    # changed after the run, this run's evidence is about a different machine.
    world = ROOT / str(report.get('world') or '')
    if not world.is_file():
        check('nothing attaches the payload to the gripper', 'NOT_RUN',
              f'{report.get("world")!r} is not on disk, so the compiled model cannot be '
              f'inspected and a planned attach cannot be ruled out')
    else:
        import hashlib
        import mujoco
        on_disk = hashlib.sha256(world.read_bytes()).hexdigest()
        expected = report.get('world_sha256')
        if expected and on_disk != expected:
            check('nothing attaches the payload to the gripper', 'FAIL',
                  f'{world.name} on disk hashes {on_disk[:12]} but this run was made against '
                  f'{str(expected)[:12]}: the world changed after the run, so the run does not '
                  f'describe the model being judged')
            return _finish(checks, run_id, report)
        model = mujoco.MjModel.from_xml_path(str(world))
        payload = model.body('payload').id
        payload_joint = model.joint('payload_free').id
        freejoint_ok = any(model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE
                           and int(model.jnt_bodyid[j]) == payload
                           for j in range(model.njnt))
        offenders = []
        for e in range(model.neq):
            ids = {int(model.eq_obj1id[e]), int(model.eq_obj2id[e])}
            if payload in ids or payload_joint in ids:
                offenders.append(f'eq[{e}] type={int(model.eq_type[e])}')
        detail = (f'the part carries a freejoint ({freejoint_ok}) and {model.neq} equality '
                  f'constraint(s) exist in the model, none of which reference it: '
                  f'{offenders or "none"}. The upstream gripper couples its own two finger '
                  f'joints, which is why this check is scoped to the PAYLOAD and is not '
                  f'"there is no equality". Whether a planner chose to attach the part is '
                  f'irrelevant here: only contact can hold it.')
        check('nothing attaches the payload to the gripper',
              'PASS' if freejoint_ok and not offenders else 'FAIL', detail)

    # -- 3. the gripper really touches the part, and keeps touching it -------------------
    rows = []
    for r in runs:
        recs = load_records(r)
        if recs is None:
            rows.append((r, None))
            continue
        rows.append((r, recs))
    missing_traces = [r['scenario']['name'] for r, recs in rows if recs is None]
    empty_traces = [r['scenario']['name'] for r, recs in rows if recs == []]
    if empty_traces:
        check('the gripper really touches the payload', 'FAIL',
              f'trace files exist but carry no records for {empty_traces}. An empty trace is '
              f'not missing evidence, it is evidence of nothing happening, and it must not '
              f'crash the judge or pass quietly.')
    elif missing_traces:
        check('the gripper really touches the payload', 'NOT_RUN',
              f'trace files missing for {missing_traces}, so contact cannot be counted')
    else:
        weak = []
        counts = []
        for r, recs in rows:
            if not in_envelope(r) or expected_of(r) != 'DONE':
                continue
            hits = [x for x in recs if x['grip_contacts']]
            counts.append(f'{r["scenario"]["name"]}@mu{r["friction"]}: {len(hits)}')
            both = any({'left_finger', 'right_finger'} <= set(x['grip_contacts']) for x in recs)
            if len(hits) < min_contact:
                weak.append(f'{r["scenario"]["name"]}@mu{r["friction"]} only {len(hits)} '
                            f'contact samples (need {min_contact})')
            elif not both:
                weak.append(f'{r["scenario"]["name"]}@mu{r["friction"]} never had BOTH '
                            f'fingers on the part')
        check('the gripper really touches the payload',
              'PASS' if not weak else 'FAIL',
              f'samples with at least one finger on the part, per admitted run: '
              f'{", ".join(counts)}. A single sample would not do: a brush past the part on '
              f'the way down also produces one, so this counts sustained contact AND requires '
              f'both fingers at some point. '
              + ('Every admitted run held it.' if not weak else '; '.join(weak)))

    # -- 4. the part leaves the table ---------------------------------------------------
    if missing_traces or empty_traces:
        check('the payload leaves the table',
              'FAIL' if empty_traces else 'NOT_RUN',
              'a trace with no records' if empty_traces else 'trace files missing')
    else:
        bad = []
        for r, recs in rows:
            if not in_envelope(r) or expected_of(r) != 'DONE':
                continue
            top = max(x['payload'][2] for x in recs)
            rise = top - rest_z
            airborne = [x for x in recs if x['payload'][2] >= rest_z + lift_margin / 2.0]
            dragged = sum(1 for x in airborne if x['table_contact_n'])
            # If nothing is holding it, nothing may lift it. Without this, a part that simply
            # appeared in mid-air would satisfy "leaves the table" -- which is exactly what a
            # planned attach looks like from the outside.
            unexplained = sum(1 for x in airborne
                              if not x['grip_contacts'] and not x.get('palm_contact_n'))
            if rise < lift_margin:
                bad.append(f'{r["scenario"]["name"]}@mu{r["friction"]} rose only '
                           f'{rise * 1000:.1f} mm (need {lift_margin * 1000:.0f})')
            elif dragged:
                bad.append(f'{r["scenario"]["name"]}@mu{r["friction"]} had {dragged} samples '
                           f'touching the table while at least half-lifted')
            elif unexplained:
                bad.append(f'{r["scenario"]["name"]}@mu{r["friction"]} had {unexplained} samples '
                           f'aloft with NOTHING touching it')
        check('the payload leaves the table',
              'PASS' if not bad else 'FAIL',
              f'the part must rise at least {lift_margin * 1000:.0f} mm above its rest height, '
              f'AND with no table contact once it is half that high -- otherwise "lifted" '
              f'could mean "slid along the surface" -- AND with something actually touching '
              f'it while it is up there, because a part that rises with nothing on it is a '
              f'planned attach wearing a trajectory. '
              + ('Every admitted run lifted it clear.' if not bad else '; '.join(bad)))

    # -- 5. the part is carried, not left behind ----------------------------------------
    if missing_traces or empty_traces:
        check('the payload is carried, not left behind',
              'FAIL' if empty_traces else 'NOT_RUN',
              'a trace with no records' if empty_traces else 'trace files missing')
    else:
        bad = []
        for r, recs in rows:
            if not in_envelope(r) or expected_of(r) != 'DONE':
                continue
            held = [x for x in recs if x['grip_contacts']]
            if not held:
                bad.append(f'{r["scenario"]["name"]}@mu{r["friction"]}: never held')
                continue
            ref = held[0]
            ref_off = [a - b for a, b in zip(ref['payload'], ref['grasp'])]
            worst = 0.0
            for x in held:
                off = [a - b for a, b in zip(x['payload'], x['grasp'])]
                worst = max(worst, max(abs(a - b) for a, b in zip(off, ref_off)))
            if worst > slip_max:
                bad.append(f'{r["scenario"]["name"]}@mu{r["friction"]} slid {worst * 1000:.2f} '
                           f'mm in the hand (limit {slip_max * 1000:.0f})')
        check('the payload is carried, not left behind',
              'PASS' if not bad else 'FAIL',
              f'the position of the part RELATIVE TO THE GRASP FRAME must not change by more '
              f'than {slip_max * 1000:.0f} mm while it is held. Measuring absolute position '
              f'would not show this: a part that fell onto the moving deck... would still be '
              f'moving. This is the same measurement the C branch used for retention. '
              + ('Every admitted run carried it.' if not bad else '; '.join(bad)))

    # -- 6. the part is placed at the target --------------------------------------------
    if missing_traces or empty_traces:
        check('the payload is placed at the declared target',
              'FAIL' if empty_traces else 'NOT_RUN',
              'a trace with no records' if empty_traces else 'trace files missing')
    else:
        bad = []
        for r, recs in rows:
            if not in_envelope(r) or expected_of(r) != 'DONE':
                continue
            end = recs[-1]['payload']
            xy = ((end[0] - place_xy[0]) ** 2 + (end[1] - place_xy[1]) ** 2) ** 0.5
            dz = abs(end[2] - rest_z)
            if xy > place_tol:
                bad.append(f'{r["scenario"]["name"]}@mu{r["friction"]} ended {xy * 1000:.1f} mm '
                           f'from the target (limit {place_tol * 1000:.0f})')
            elif dz > touchdown_tol:
                bad.append(f'{r["scenario"]["name"]}@mu{r["friction"]} ended {dz * 1000:.1f} mm '
                           f'off the surface height (limit {touchdown_tol * 1000:.0f})')
        check('the payload is placed at the declared target',
              'PASS' if not bad else 'FAIL',
              f'final xy must be within {place_tol * 1000:.0f} mm of {place_xy}, and final z '
              f'within {touchdown_tol * 1000:.0f} mm of the resting height {rest_z:.4f} '
              f'(i.e. actually on the surface, not held above it). '
              + ('Every admitted run placed it.' if not bad else '; '.join(bad)))

    # -- 7. released, and it stays put --------------------------------------------------
    if missing_traces or empty_traces:
        check('the payload is released and stays put',
              'FAIL' if empty_traces else 'NOT_RUN',
              'a trace with no records' if empty_traces else 'trace files missing')
    else:
        bad = []
        for r, recs in rows:
            if not in_envelope(r) or expected_of(r) != 'DONE':
                continue
            hold = [x for x in recs if x['state'] == 'HOLD']
            if not hold:
                bad.append(f'{r["scenario"]["name"]}@mu{r["friction"]}: no HOLD window')
                continue
            if hold[-1]['grip_contacts']:
                bad.append(f'{r["scenario"]["name"]}@mu{r["friction"]}: still touching at the '
                           f'end of HOLD ({hold[-1]["grip_contacts"]})')
                continue
            ref = hold[0]['payload']
            drift = max(max(abs(a - b) for a, b in zip(x['payload'], ref)) for x in hold)
            peak = max(x['payload_speed'] for x in hold)
            if drift > drift_max:
                bad.append(f'{r["scenario"]["name"]}@mu{r["friction"]} drifted '
                           f'{drift * 1000:.2f} mm after release (limit {drift_max * 1000:.0f})')
            elif peak > speed_max:
                bad.append(f'{r["scenario"]["name"]}@mu{r["friction"]} peaked at {peak:.3f} m/s '
                           f'after release (limit {speed_max:.3f})')
        check('the payload is released and stays put',
              'PASS' if not bad else 'FAIL',
              f'over the HOLD window the gripper must have let go (no contact at the end), the '
              f'part must move no more than {drift_max * 1000:.1f} mm and never exceed '
              f'{speed_max:.3f} m/s. "Stable after release" is not "the arm stopped moving". '
              + ('Every admitted run settled.' if not bad else '; '.join(bad)))

    # -- 8. the sequence ran, in order, to the end --------------------------------------
    order = {name: i for i, name in enumerate(required)}
    bad = []
    for r in runs:
        if not in_envelope(r) or expected_of(r) != 'DONE':
            continue
        seq = [t['state'] for t in r.get('transitions') or []]
        seen = [s for s in seq if s in order]
        if seen != sorted(seen, key=lambda s: order[s]):
            bad.append(f'{r["scenario"]["name"]}@mu{r["friction"]} out of order: {seen}')
        elif set(required) - set(seen):
            bad.append(f'{r["scenario"]["name"]}@mu{r["friction"]} skipped '
                       f'{sorted(set(required) - set(seen))}')
        elif r.get('final_state') != 'DONE':
            bad.append(f'{r["scenario"]["name"]}@mu{r["friction"]} ended at '
                       f'{r.get("final_state")}')
    check('the sequence ran in order and to the end',
          'PASS' if not bad else 'FAIL',
          f'expected order {required}; an admitted run must reach DONE through all of them. '
          + ('All admitted runs did.' if not bad else '; '.join(bad)))

    # -- 9. the fall latch ---------------------------------------------------------------
    dropped = [f'{r["scenario"]["name"]}@mu{r["friction"]} at {r["fell_at"]}'
               for r in runs if r.get('fell_at') is not None]
    check('no run loses the payload it is carrying',
          'PASS' if not dropped else 'FAIL',
          'a drop must be latched, not merely recorded: the C branch recorded `fell_at` and '
          'read it for nothing, so 211 runs drove on for 79 minutes after losing their load. '
          + ('No run dropped its part.' if not dropped else '; '.join(dropped)))

    # -- 10. every admitted case completes ----------------------------------------------
    failed = [f'{r["scenario"]["name"]}@mu{r["friction"]} -> {r.get("final_state")}'
              for r in good_runs if r.get('final_state') != 'DONE']
    holes = sorted({f'{r["scenario"]["name"]}@mu{r["friction"]}'
                    for r in runs if expected_of(r) != 'DONE'})
    check('the skill is sound: every admitted case completes',
          'PASS' if not failed else 'FAIL',
          f'{len(good_runs)} run(s) were declared to complete; did not: '
          f'{"; ".join(failed) if failed else "none"}. '
          f'{len(holes)} run(s) are declared OUTCOMES-AS-FAILURES rather than successes -- the '
          f'measured limits of the skill, kept in the set so the boundary is tested instead of '
          f'assumed: {", ".join(holes) or "none"}.')

    # -- 11. a case that must fail, does -------------------------------------------------
    if not bad_runs:
        check('a case that must fail does fail', 'NOT_RUN',
              'no run in this report carries an expected failure, so the judge is only ever '
              'shown successes -- which is how a check quietly stops working')
    else:
        wrong = []
        for r in bad_runs:
            want = expected_of(r)
            if r.get('final_state') != want:
                wrong.append(f'{r["scenario"]["name"]}@mu{r["friction"]} ended '
                             f'{r.get("final_state")}, expected {want}')
            elif not r.get('faulted'):
                wrong.append(f'{r["scenario"]["name"]}@mu{r["friction"]} reached {want} '
                             f'without latching a fault')
        check('a case that must fail does fail',
              'PASS' if not wrong else 'FAIL',
              f'{len(bad_runs)} run(s) are declared to fail and must reach their declared '
              f'fault state with the fault latched: '
              + ('all did.' if not wrong else '; '.join(wrong)))

    # -- 11b. the declared envelope is where the claim stops ----------------------------
    if not outside_runs:
        check('the declared envelope is where the claim stops', 'NOT_RUN',
              'no run lies outside the declared envelope, so the limit of the claim is never '
              'exercised -- a boundary that nothing is run against is an assumption')
    else:
        worked = [f'{r["scenario"]["name"]}@mu{r["friction"]}' for r in outside_runs
                  if r.get('final_state') == 'DONE']
        stopped = [f'{r["scenario"]["name"]}@mu{r["friction"]}' for r in outside_runs
                   if r.get('final_state') != 'DONE']
        check('the declared envelope is where the claim stops',
              'PASS',
              f'{len(outside_runs)} run(s) lie OUTSIDE the declared envelope and are reported '
              f'but NOT claimed. {len(stopped)} stopped: {", ".join(stopped) or "none"}. '
              f'{len(worked)} nevertheless completed ({", ".join(worked) or "none"}) -- that is '
              f'CONSERVATISM, not success: the envelope is narrower than the mechanism. Neither '
              f'group is counted toward the skill claim in this report. Note that "completed" '
              f'here means the state machine reached DONE; the +10 mm y case reaches DONE and '
              f'still fails the placement criteria, which is exactly why the envelope had to be '
              f'measured with the criteria rather than with the final state.')

    # -- 12. the whole-A claim, and its scope -------------------------------------------
    claim = report.get('full_A_acceptance')
    in_source = set(report.get('stages_in_source') or [])
    executed = set()
    for r in runs:
        executed |= set(r.get('stages_executed') or ())
    missing_source = sorted(set(required) - in_source)
    missing_run = sorted(set(required) - executed)
    consistent = ((claim == 'NOT_RUN' and (missing_source or missing_run))
                  or (claim == 'CLAIMED' and not missing_source and not missing_run))
    if missing_source or missing_run:
        scope = (f'A PASS here is a statement about the stages this machine has. Whether A as '
                 f'a whole passes cannot be decided from this artifact: '
                 f'{len(missing_source)} of {len(required)} required stage(s) do not exist in '
                 f'the probe'
                 + (f' ({", ".join(missing_source)})' if missing_source else '')
                 + (f'; and {", ".join(missing_run)} did not run in this report'
                    if missing_run else '')
                 + '.')
    else:
        scope = (f'Nothing required is missing from the machine, and every stage ran in this '
                 f'report, so the stage-list half of the claim is satisfied here. What this '
                 f'artifact still does NOT establish is its declared scope: '
                 f'{report.get("scope_claim")}. Read the PASS with that scope attached.')
    check('the whole-A claim is not made without its stages',
          'PASS' if consistent else 'FAIL',
          f'full_A_acceptance is {claim!r}, derived by the probe from its own stage names; '
          f'{len(missing_source)} of {len(required)} required stages do not exist in the probe: '
          f'{", ".join(missing_source) or "none"}; absent from this report: '
          f'{", ".join(missing_run) or "none"}. *** {scope} ***')

    return _finish(checks, run_id, report)


def _finish(checks, run_id, report):
    failed = [c for c in checks if c['verdict'] == 'FAIL']
    not_run = [c for c in checks if c['verdict'] == 'NOT_RUN']
    verdict = 'FAIL' if failed else ('INCOMPLETE' if not_run else 'PASS')
    return {'run_id': run_id, 'verdict': verdict, 'checks': checks,
            'full_A_acceptance': report.get('full_A_acceptance'),
            'scope_claim': report.get('scope_claim')}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--reports-dir', default=str(REPORTS))
    args = parser.parse_args()

    run_dir = pathlib.Path(args.reports_dir) / args.run_id
    report_path = run_dir / 'report.json'
    if not report_path.is_file():
        print(f'no report.json at {report_path}')
        return 2
    report = json.loads(report_path.read_text(encoding='utf-8'))
    payload = evaluate(report, run_dir, args.run_id)

    (run_dir / 'acceptance.json').write_text(json.dumps(payload, indent=1), encoding='utf-8')
    lines = [f'# A fixed-arm pick and place -- `{args.run_id}`', '',
             f'**verdict: {payload["verdict"]}**  '
             f'({len(payload["checks"])} checks: '
             f'{sum(1 for c in payload["checks"] if c["verdict"] == "PASS")} PASS, '
             f'{sum(1 for c in payload["checks"] if c["verdict"] == "FAIL")} FAIL, '
             f'{sum(1 for c in payload["checks"] if c["verdict"] == "NOT_RUN")} NOT_RUN)', '',
             '## Checks', '']
    for c in payload['checks']:
        lines.append(f'- `{c["verdict"]}` **{c["check"]}** -- {c["detail"]}')
    lines += ['', '## Runs', '',
              '| scenario | mu | final state | faulted | sim s | grip-contact samples |',
              '| --- | --- | --- | --- | --- | --- |']
    for r in report.get('runs') or []:
        lines.append(f'| {r["scenario"]["name"]} | {r["friction"]} | {r["final_state"]} | '
                     f'{r["faulted"]} | {r["sim_seconds"]:.2f} | '
                     f'{r["payload_contact_samples"]} |')
    (run_dir / 'acceptance.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')

    print(f'{payload["verdict"]}: '
          f'{sum(1 for c in payload["checks"] if c["verdict"] == "PASS")} PASS, '
          f'{sum(1 for c in payload["checks"] if c["verdict"] == "FAIL")} FAIL, '
          f'{sum(1 for c in payload["checks"] if c["verdict"] == "NOT_RUN")} NOT_RUN')
    return 0 if payload['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
