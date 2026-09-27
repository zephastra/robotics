"""Docking: the contract, the observations, and the state machine that connects them.

WHY THIS PACKAGE EXISTS
-----------------------
`MOVE_TO_STATION` and `DOCK` are two different precision problems. Nav2 lands the vehicle in a
pre-dock area with a measured 0.225-0.283 m arrival error (`D059`, `D062`), and C's transfer window
is millimetres wide. The guidance is explicit that nav2 succeeding must not start the belt, and that
"tune AMCL until it is millimetre-accurate" is not the only answer. So:

  * `contract` holds the declared geometry and the window, with every threshold carrying its source;
  * `observations` is the ONLY way the controller may learn where it is relative to a station, and it
    refuses to answer when the answer is unknown rather than returning zero;
  * `machine` walks IDLE -> ... -> DOCKED, and the handover of command authority is a state
    transition rather than a comment.

NOTHING HERE READS THE SIMULATOR. These modules are pure, stdlib-only and clock-injected, the same
shape as the rest of `workcell`, so they can be tested without physics and so the physical probe
cannot smuggle a truth value in through an import.
"""
from workcell.docking.contract import (
    CONTRACT_REFUSALS, ContractRefused, check_window, load_contract, window_of,
)
from workcell.docking.machine import (
    DOCK_REFUSALS, DOCKED_VALIDITY_S, FAILURE_STATES, STATES, DockMachine, Permits,
)
from workcell.docking.observations import (
    OBSERVATION_REFUSALS, ObservationRefused, accept_observation, validate_observation,
)

__all__ = [
    'CONTRACT_REFUSALS', 'ContractRefused', 'check_window', 'load_contract', 'window_of',
    'DOCK_REFUSALS', 'DOCKED_VALIDITY_S', 'FAILURE_STATES', 'STATES', 'DockMachine', 'Permits',
    'OBSERVATION_REFUSALS', 'ObservationRefused', 'accept_observation', 'validate_observation',
]
