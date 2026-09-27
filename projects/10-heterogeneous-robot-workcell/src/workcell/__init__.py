"""P2 workcell core: the order plane, the resource plane, and the transfer plane.

Four modules, each a pure and clock-injected piece of the contract in `docs/CONTRACTS.md`:

* `schema`    -- section 1-3, 7: the vocabularies and the strict validators. Refuses unknown
                 fields, duplicate JSON keys, non-finite numbers, and undeclared enum values.
* `ledger`    -- the order state machine, including the constrained states that must remember
                 where they came from.
* `resources` -- section 6: leases with owner/generation/epoch/ttl, where a late reply cannot
                 re-fill the TTL and an occupied resource is not released by a clock.
* `transfer`  -- sections 4-5: the eight-stage handover, and the honest difference between
                 "proven not to have moved" (`ABORTED`) and "cannot be proven" (`NEEDS_ATTENTION`).

None of them imports ROS, MuJoCo, or a wall clock: that is what makes the core testable in a
clean shell, and what lets the same code run under sim time and under a replay.
"""

from . import schema
from . import ledger
from . import resources
from . import transfer

__all__ = ["schema", "ledger", "resources", "transfer"]
