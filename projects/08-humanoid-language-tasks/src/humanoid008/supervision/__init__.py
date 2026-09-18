"""Three-layer supervision: structure, preconditions, runtime (section 13).

Layer 1 :mod:`.validator`      -- schema, entities, revision, arguments
Layer 2 :mod:`.preconditions`  -- freshness, declared preconditions, budget
Layer 3 :mod:`.runtime_guard`  -- stop requests, command expiry, body faults

A rejected proposal, a triggered runtime guard, and a post-hoc contact record are
three different things and are reported separately. None of them is an ex-ante
safety guarantee.
"""

from .preconditions import PreconditionSupervisor, SupervisionPolicy
from .runtime_guard import RuntimeGuard, RuntimePolicy
from .validator import OK, ProposalValidator, Verdict

__all__ = [
    "OK",
    "PreconditionSupervisor",
    "ProposalValidator",
    "RuntimeGuard",
    "RuntimePolicy",
    "SupervisionPolicy",
    "Verdict",
]
