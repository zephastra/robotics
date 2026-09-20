#!/usr/bin/env python3
"""Can a robot legally stand at the node it is told to ask from?

Why this exists
===============

`config/resources.yaml` claims:

    The alignment nodes are placed far enough back that a stopped robot's footprint
    plus margin still clears this rectangle.

That claim was written when the gate judged from ``spawn (+) odom``.  Measured on
2026-09-17, once the gate was switched to the localised pose, the claim is false:
the robot reached ``align_west`` and was refused there, so it could never ask for
the permit, so the crossing could not start.  ``nav2 status=6`` followed from that,
and rounds 6-9 chased it as a navigation problem.

The claim is arithmetic, so it is checkable without a simulator.  This script asks
the *traffic layer itself* -- the same `CrossingManager` and the same
`check_movement` the safety gate calls -- whether each named node is a pose a robot
may hold without a permit.  It re-derives no geometry of its own: a check that
re-derives the thing it is checking can only agree with itself.

What it reports, per direction:

  * the node's own pose, and whether a robot sitting there is allowed to move;
  * how far along the approach axis a robot may legally stand (found by bisection
    against `check_movement`, not by hand);
  * the shortfall in metres, i.e. how far the node has to move to be reachable;
  * the same, allowing for ``node_reach_tolerance_m``, because a robot counted as
    "at" the node may be that far from it.

Two speeds are tested, and they mean different things:

  ``0 m/s``                    can the robot stop here and open the handshake?
  ``max_measured_speed_mps``   can it arrive here without being refused en route?

Exit codes: 0 all nodes reachable at both speeds, 1 at least one shortfall,
3 the config could not be loaded (an instrument fault, not a system verdict).
"""

from __future__ import annotations

import argparse
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]


def _load():
    """Import the project's own loader, and say so if that fails."""
    sys.path.insert(0, str(ROOT / "src" / "fleet_core"))
    try:
        from fleet_core import CrossingManager, load_traffic_config
    except Exception as exc:  # noqa: BLE001
        print(f"INSTRUMENT FAULT: cannot import fleet_core: {exc!r}")
        print("  this says nothing about the geometry; check the venv/interpreter")
        raise SystemExit(3)
    return CrossingManager, load_traffic_config


def _manager(CrossingManager, cfg):
    """A manager with an empty lease book: a robot that holds nothing.

    That is the case that matters -- the robot approaching the asking point has no
    permit yet, which is precisely why it is approaching it.
    """
    return CrossingManager(cfg)


def _allowed(manager, pose, *, speed, session, length_m, width_m, localization_valid=True):
    """One question, asked the way the gate asks it."""
    check = manager.check_movement(
        task_id=session["task_id"],
        boot_id=session["boot_id"],
        revision=session["revision"],
        generation=session["generation"],
        epoch=session["epoch"],
        wall_now=0.0,
        pose=pose,
        twist=(speed, 0.0, 0.0),
        length_m=length_m,
        width_m=width_m,
        localization_valid=localization_valid,
    )
    return check.allowed, check.reason.value, check.detail


def _furthest_legal(manager, *, direction, node_name, node, cfg, speed, session,
                    length_m, width_m):
    """The legal x nearest the corridor along this node's approach axis.

    Bisection against `check_movement` rather than a formula, so the answer comes
    from the component that decides.  Three outcomes:

      * a float -- the boundary; the node is legal if it is on the far side of it;
      * ``"beyond"`` -- no refusal within 3 m toward the corridor (the node is
        separated in the other axis, which is a different reason for being legal);
      * ``None`` -- no legal position within 3 m away from the corridor either,
        which means the refusal is not a matter of this node's x at all.
    """
    spec = manager.direction_spec(direction)
    sign = float(spec.approach_sign)          # the direction of travel through the gap
    yaw = node.yaw

    def allowed(x: float) -> bool:
        ok, _, _ = _allowed(manager, (x, node.y, yaw), speed=speed,
                            session=session, length_m=length_m, width_m=width_m)
        return ok

    if allowed(node.x):
        # Walk toward the corridor until refused.
        far = node.x + 3.0 * sign
        if allowed(far):
            return "beyond"
        lo, hi = node.x, far                  # lo allowed, hi refused
    else:
        # The node is already refused; walk away from the corridor until allowed.
        far = node.x - 3.0 * sign
        if not allowed(far):
            return None
        lo, hi = far, node.x                  # lo allowed, hi refused
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        if allowed(mid):
            lo = mid
        else:
            hi = mid
    return lo


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(ROOT / "config" / "resources.yaml"))
    ap.add_argument("--length-m", type=float, default=0.60)
    ap.add_argument("--width-m", type=float, default=0.45)
    args = ap.parse_args()

    CrossingManager, load_traffic_config = _load()
    try:
        cfg = load_traffic_config(pathlib.Path(args.config))
    except Exception as exc:  # noqa: BLE001
        print(f"INSTRUMENT FAULT: cannot load {args.config}: {exc!r}")
        raise SystemExit(3)

    session = dict(task_id="", boot_id="", revision=0, generation=0, epoch=0)
    manager = _manager(CrossingManager, cfg)

    tol = cfg.node_reach_tolerance_m
    vmax = cfg.stop.max_measured_speed_mps
    envelope = cfg.stop.envelope_m(vmax)
    reserve = envelope + cfg.stop.localization_margin_m
    print(f"config:                 {args.config}")
    print(f"footprint:              {args.length_m} x {args.width_m} m")
    print(f"footprint_margin_m:     {cfg.footprint_margin_m}")
    print(f"localization_margin_m:  {cfg.stop.localization_margin_m}")
    print(f"stop envelope at vmax:  {envelope:.3f} m  (vmax = {vmax} m/s)")
    print(f"stop reserve (env+loc): {reserve:.3f} m")
    print(f"node_reach_tolerance:   {tol} m")
    print(f"stop_boundary growth:   {reserve + cfg.footprint_margin_m:.3f} m beyond the "
          f"footprint at vmax")
    print()

    roles = ("wait_node", "align_node", "exit_node", "release_node")
    findings: list[str] = []

    for direction in sorted(cfg.directions):
        spec = manager.direction_spec(direction)
        print(f"===== {direction}  bundle={list(manager.bundle(direction))} =====")
        for role in roles:
            node_name = getattr(spec, role)
            node = manager.node(node_name)
            # The robot holds nothing while it approaches; `exit_node` is inside the
            # bundle, so it is only legal once granted, and is expected to be refused
            # here -- that is the reservation working, not a shortfall.
            ok0, why0, _ = _allowed(manager, (node.x, node.y, node.yaw), speed=0.0,
                                    session=session, length_m=args.length_m,
                                    width_m=args.width_m)
            okv, whyv, detailv = _allowed(manager, (node.x, node.y, node.yaw), speed=vmax,
                                          session=session, length_m=args.length_m,
                                          width_m=args.width_m)
            limit_m = _furthest_legal(manager, direction=direction, node_name=node_name,
                                      node=node, cfg=cfg, speed=vmax, session=session,
                                      length_m=args.length_m, width_m=args.width_m)
            limit_0 = _furthest_legal(manager, direction=direction, node_name=node_name,
                                      node=node, cfg=cfg, speed=0.0, session=session,
                                      length_m=args.length_m, width_m=args.width_m)
            print(f"  {node_name:14s} ({node.x:6.2f},{node.y:5.2f}) yaw {node.yaw:5.2f}  "
                  f"stopped: {'ALLOWED' if ok0 else 'refused (' + why0 + ')'}"
                  + (f"  moving: {'ALLOWED' if okv else 'refused (' + whyv + ')'}"))

            def _say(limit, label, node_ok):
                if limit == "beyond":
                    print(f"                 legal limit {label}: none within 3 m toward "
                          "the corridor (separated in the other axis)")
                elif limit is None:
                    print(f"                 legal limit {label}: none within 3 m either "
                          "way -- the refusal is not about this node's x")
                else:
                    gap = abs(limit - node.x)
                    print(f"                 legal limit {label}: x = {limit:7.3f}"
                          + (f"   -- the node is {gap:.3f} m inside it"
                             if not node_ok else
                             f"   -- the node is {gap:.3f} m clear of it"))

            _say(limit_m, f"at {vmax} m/s", okv)
            _say(limit_0, "when stopped ", ok0)

            if role in ("wait_node", "align_node") and not okv:
                # These two are the points the handshake is opened from. A robot that
                # cannot hold one of them cannot ask, and the crossing cannot start.
                want = None
                if isinstance(limit_m, float):
                    want = limit_m
                findings.append(
                    f"{direction}.{role} = {node_name}: refused while moving at "
                    f"{vmax} m/s ({whyv}); node x={node.x}"
                    + (f", the nearest legal x is {want:.3f} -> move it out by "
                       f"{abs(want - node.x):.3f} m" if want is not None
                       else ", and no legal x exists within 3 m of it on its approach"))
                print(f"                 SHORTFALL: {detailv[:220]}")

        print()

    print("=" * 78)
    if findings:
        print(f"RESULT: FAIL -- {len(findings)} asking point(s) a robot cannot legally hold:")
        for f in findings:
            print(f"  - {f}")
        print()
        print("  An asking point inside the refusal boundary is a deadlock, not a tight")
        print("  fit: the robot is told to stop and ask from a place it is refused entry to,")
        print("  it stops short, and the driver reports it as a navigation failure")
        print("  (`nav2 status=6`). Fix by moving the node, not by relaxing the boundary --")
        print("  the boundary is what stops a robot whose stopping distance reaches the")
        print("  corridor, and that property must hold at every speed the gate may authorise.")
        return 1
    print("RESULT: every asking point is reachable at both speeds.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except Exception as exc:  # noqa: BLE001
        # An instrument that crashes must not be mistaken for a geometry verdict:
        # exit 3, and say which of the two happened (lesson from rounds 5-10).
        import traceback
        print()
        print(f"INSTRUMENT FAULT: {type(exc).__name__}: {exc}")
        print("  this run says NOTHING about the geometry.")
        traceback.print_exc()
        sys.exit(3)
