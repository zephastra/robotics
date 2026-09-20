"""L1 §6, §7, §8: resource grants, FIFO, TTL-vs-clearance, atomic bundles.

Maps to TEST_AND_ACCEPTANCE L1:
  * 6  simultaneous request grants exactly one, FIFO, duplicate acquire, renew, expire
  * 7  expiry does not release; unknown pose does not release; occupied exit does not grant
  * 8  bundle reservation is all-or-nothing

These are the rules that keep two robots out of the same corridor. Every test
below is a specific way a naive implementation would let that happen.
"""

from __future__ import annotations

import pytest

from fleet_core import ReasonCode, ResourceId, ResourceKind
from fleet_core.domain import StaleCommandError


def rid(kind: ResourceKind, name: str) -> ResourceId:
    return ResourceId(kind, name)


def bundle(cfg):
    return (cfg.corridor, cfg.exit_buffers["left"])


# --------------------------------------------------------------------------- #
# §6 basic grant / mutual exclusion
# --------------------------------------------------------------------------- #


def test_acquire_grants_when_free(book, cfg):
    res = book.acquire(
        task_id="task-A", robot_id="r01", resources=bundle(cfg),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    assert res.granted is True
    assert res.permit is not None
    assert book.owner_of(cfg.corridor) == "task-A"
    assert book.owner_of(cfg.exit_buffers["left"]) == "task-A"


def test_simultaneous_request_grants_exactly_one(book, cfg):
    r1 = book.acquire(
        task_id="task-A", robot_id="r01", resources=bundle(cfg),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    r2 = book.acquire(
        task_id="task-B", robot_id="r02", resources=bundle(cfg),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    assert r1.granted is True
    assert r2.granted is False
    assert r2.reason is ReasonCode.RESOURCE_BUSY
    assert book.owner_of(cfg.corridor) == "task-A"


def test_duplicate_acquire_by_same_task_is_idempotent(book, cfg):
    first = book.acquire(
        task_id="task-A", robot_id="r01", resources=bundle(cfg),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
        permit_id="permit-A",
    )
    again = book.acquire(
        task_id="task-A", robot_id="r01", resources=bundle(cfg),
        now=0.0, wall_now=1.0, boot_id="b1", revision=0, generation=1, epoch=0,
        permit_id="permit-A-second",
    )
    assert again.granted is True
    assert len(book.holders(cfg.corridor)) == 1, "a duplicate request must not double-grant"


def test_release_frees_the_resource(book, cfg):
    book.acquire(
        task_id="task-A", robot_id="r01", resources=bundle(cfg),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    released = book.release(task_id="task-A")
    assert released
    assert book.is_free(cfg.corridor)


def test_release_with_wrong_generation_leaves_the_hold(book, cfg):
    book.acquire(
        task_id="task-A", robot_id="r01", resources=bundle(cfg),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=7, epoch=0,
    )
    assert book.release(task_id="task-A", expected_generation=99) == []
    assert book.owner_of(cfg.corridor) == "task-A"


def test_cancelled_task_does_not_keep_a_queue_slot(book, cfg):
    book.acquire(
        task_id="task-A", robot_id="r01", resources=bundle(cfg),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    book.acquire(
        task_id="task-B", robot_id="r02", resources=bundle(cfg),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    assert book.queue(cfg.corridor) == ["task-B"]
    book.abandon_queue("task-B")
    assert book.queue(cfg.corridor) == []


# --------------------------------------------------------------------------- #
# §6 FIFO
# --------------------------------------------------------------------------- #


def test_fifo_order_is_respected(book, cfg):
    book.acquire(
        task_id="task-A", robot_id="r01", resources=bundle(cfg),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    # B asks first, C asks second.
    book.acquire(
        task_id="task-B", robot_id="r02", resources=bundle(cfg),
        now=0.0, wall_now=1.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    book.acquire(
        task_id="task-C", robot_id="r03", resources=bundle(cfg),
        now=0.0, wall_now=2.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    assert book.queue(cfg.corridor) == ["task-B", "task-C"]
    assert book.head_of(cfg.corridor) == "task-B"

    book.release(task_id="task-A")

    # B is at the head, so B gets it.
    rb = book.acquire(
        task_id="task-B", robot_id="r02", resources=bundle(cfg),
        now=0.0, wall_now=3.0, boot_id="b1", revision=0, generation=2, epoch=0,
    )
    assert rb.granted is True
    assert book.owner_of(cfg.corridor) == "task-B"


def test_newcomer_cannot_jump_the_queue(book, cfg):
    book.acquire(
        task_id="task-A", robot_id="r01", resources=bundle(cfg),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    book.acquire(
        task_id="task-B", robot_id="r02", resources=bundle(cfg),
        now=0.0, wall_now=1.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    # The corridor frees up, but B is queued. A fresh C must not slip in front.
    book.release(task_id="task-A")
    rc = book.acquire(
        task_id="task-C", robot_id="r03", resources=bundle(cfg),
        now=0.0, wall_now=2.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    assert rc.granted is False
    assert rc.queued_behind == ("task-B",)
    assert book.owner_of(cfg.corridor) is None


# --------------------------------------------------------------------------- #
# §7 expiry is not clearance
# --------------------------------------------------------------------------- #


def test_ttl_expiry_does_not_free_the_resource(book, cfg):
    """F10: the lease lapsed, but nobody proved the corridor is empty."""
    book.permit_ttl_s = 10.0
    book.acquire(
        task_id="task-A", robot_id="r01", resources=bundle(cfg),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    expired = book.expire_due(wall_now=100.0)
    assert len(expired) == 2, "both halves of the bundle lapse together"

    assert book.is_free(cfg.corridor) is False, "an expired lease must NOT read as free"
    assert book.unknown_reason(cfg.corridor) is not None

    res = book.acquire(
        task_id="task-B", robot_id="r02", resources=bundle(cfg),
        now=0.0, wall_now=101.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    assert res.granted is False
    assert res.reason is ReasonCode.RESOURCE_UNKNOWN


def test_explicit_clearance_frees_it(book, cfg):
    book.permit_ttl_s = 10.0
    book.acquire(
        task_id="task-A", robot_id="r01", resources=bundle(cfg),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    book.expire_due(wall_now=100.0)
    cleared = book.confirm_clear(resources=bundle(cfg), proof="evaluator: no footprint inside")
    assert set(cleared) == set(bundle(cfg))
    assert book.is_free(cfg.corridor) is True


def test_confirm_clear_requires_proof(book, cfg):
    book.permit_ttl_s = 10.0
    book.acquire(
        task_id="task-A", robot_id="r01", resources=bundle(cfg),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    book.expire_due(wall_now=100.0)
    with pytest.raises(ValueError):
        book.confirm_clear(resources=bundle(cfg), proof="")


def test_unknown_robot_position_blocks_entry(book, cfg):
    """F14: if we cannot place the robot, the area is unknown, not free."""
    res = book.acquire(
        task_id="task-B", robot_id="r02", resources=bundle(cfg),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
        robot_position_known=False,
    )
    assert res.granted is False
    assert res.reason is ReasonCode.RESOURCE_UNKNOWN


# --------------------------------------------------------------------------- #
# §7 occupied exit buffer
# --------------------------------------------------------------------------- #


def test_occupied_exit_buffer_blocks_the_bundle(book, cfg):
    """F03: never admit a robot to a corridor it cannot leave."""
    exit_left = cfg.exit_buffers["left"]
    book.acquire(
        task_id="task-parked", robot_id="r02", resources=(exit_left,),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    assert book.is_free(cfg.corridor) is True
    assert book.is_free(exit_left) is False

    res = book.acquire(
        task_id="task-A", robot_id="r01", resources=bundle(cfg),
        now=0.0, wall_now=1.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    assert res.granted is False
    assert res.blocked_by == (exit_left,)
    assert book.owner_of(cfg.corridor) is None, "the corridor must stay un-granted too"


# --------------------------------------------------------------------------- #
# §8 atomic bundle
# --------------------------------------------------------------------------- #


def test_bundle_is_all_or_nothing_when_partially_blocked(book, cfg):
    corridor, exit_left = bundle(cfg)
    # Something already holds the corridor.
    book.acquire(
        task_id="task-X", robot_id="r09", resources=(corridor,),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    res = book.acquire(
        task_id="task-A", robot_id="r01", resources=(corridor, exit_left),
        now=0.0, wall_now=1.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    assert res.granted is False
    assert book.owner_of(exit_left) is None, "no half-granted bundle may exist"
    assert len(book.holders(exit_left)) == 0


def test_partial_grant_never_leaves_a_stranded_holder(book, cfg):
    """After a refused bundle, the previously-held half must be untouched."""
    corridor, exit_left = bundle(cfg)
    book.acquire(
        task_id="task-X", robot_id="r09", resources=(corridor, exit_left),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    book.acquire(
        task_id="task-A", robot_id="r01", resources=(corridor, exit_left),
        now=0.0, wall_now=1.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    assert book.owner_of(corridor) == "task-X"
    assert book.owner_of(exit_left) == "task-X"
    assert len(book.holders(corridor)) == 1


# --------------------------------------------------------------------------- #
# §6 renew
# --------------------------------------------------------------------------- #


def test_renew_extends_the_lease(book, cfg):
    book.permit_ttl_s = 10.0
    res = book.acquire(
        task_id="task-A", robot_id="r01", resources=bundle(cfg),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=3, epoch=0,
        permit_id="permit-A",
    )
    assert res.permit is not None
    renewed = book.renew(
        "permit-A", wall_now=9.0, boot_id="b1", revision=0, generation=3, epoch=0
    )
    assert renewed.expires_at == pytest.approx(19.0)
    assert book.expire_due(wall_now=15.0) == []
    assert book.owner_of(cfg.corridor) == "task-A"


def test_renew_with_stale_generation_is_rejected(book, cfg):
    book.acquire(
        task_id="task-A", robot_id="r01", resources=bundle(cfg),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=3, epoch=0,
        permit_id="permit-A",
    )
    with pytest.raises(StaleCommandError):
        book.renew("permit-A", wall_now=1.0, boot_id="b1", revision=0, generation=4, epoch=0)


def test_renew_with_old_boot_id_is_rejected(book, cfg):
    book.acquire(
        task_id="task-A", robot_id="r01", resources=bundle(cfg),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=3, epoch=0,
        permit_id="permit-A",
    )
    with pytest.raises(StaleCommandError):
        book.renew(
            "permit-A", wall_now=1.0, boot_id="b999", revision=0, generation=3, epoch=0
        )


def test_renew_with_old_epoch_is_rejected(book, cfg):
    book.acquire(
        task_id="task-A", robot_id="r01", resources=bundle(cfg),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=3, epoch=0,
        permit_id="permit-A",
    )
    with pytest.raises(StaleCommandError):
        book.renew("permit-A", wall_now=1.0, boot_id="b1", revision=0, generation=3, epoch=1)


def test_renew_unknown_permit_raises(book):
    with pytest.raises(KeyError):
        book.renew("nope", wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0)


# --------------------------------------------------------------------------- #
# capacity > 1
# --------------------------------------------------------------------------- #


def test_capacity_two_allows_two_holders(book, cfg):
    charger = list(cfg.chargers.values())[0]
    book.capacity[charger] = 2
    a = book.acquire(
        task_id="task-A", robot_id="r01", resources=(charger,),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    b = book.acquire(
        task_id="task-B", robot_id="r02", resources=(charger,),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    c = book.acquire(
        task_id="task-C", robot_id="r03", resources=(charger,),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    assert a.granted and b.granted
    assert c.granted is False, "capacity must be enforced, not merely recorded"


def test_empty_resource_list_is_rejected(book):
    res = book.acquire(
        task_id="task-A", robot_id="r01", resources=(),
        now=0.0, wall_now=0.0, boot_id="b1", revision=0, generation=1, epoch=0,
    )
    assert res.granted is False
    assert res.reason is ReasonCode.INVALID_INPUT
