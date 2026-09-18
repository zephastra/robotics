"""MuJoCo -> WorldState bridge (P5).

The P2/P3 task pipeline consumes a ``WorldState`` with entities, stations and a
robot summary. This module adapts the MuJoCo robot's real sensors to that
contract:

- ``box_01``  <- the payload, localised by the head-camera colour marker (vision)
- ``station_a`` <- the source table (known fixture map)
- ``station_b`` <- the destination table (visually localised marker)

``view()`` is **cheap** (re-reads the last sensor state, no rendering); the
expensive render + detect happens inside the skills' ``capture()``. This is what
lets the executor call ``world_provider()`` every tick without re-rendering.
"""

from __future__ import annotations

import json

import numpy as np

from ..contracts import (
    CapabilityState,
    EntityObservation,
    EvidenceSource,
    RobotState,
    SkillName,
    StationState,
    StationStatus,
    WorldState,
)
from . import ROOT
from .camera import HeadCamera
from .model_builder import build_runtime
from .robot_runtime import OPEN
from .tactile import sense
from .vision import detect

CFG = json.loads((ROOT / "config" / "task.json").read_text())


class MujocoWorld:
    """Shared MuJoCo runtime + sensor bridge + cross-skill manipulation context.

    Exactly one instance exists per task; the skills share it and must never
    rebuild the model or reset qpos (section 10).
    """

    def __init__(self) -> None:
        self.runtime = build_runtime()
        self.camera = HeadCamera(self.runtime, CFG["camera_width"], CFG["camera_height"])

        # sensor state (latest known)
        self.box = None
        self.destination = None        # station_b (green marker)
        self.destination_marker = None
        self.destination_c = None      # station_c (blue marker)
        self.destination_marker_c = None
        self.touch = {"left": 0.0, "right": 0.0, "source": False, "destination": False}
        self._next_vision = 0.0

        # cross-skill manipulation context (section 10)
        self.arm = None
        self.hands = {s: OPEN.copy() for s in ("left", "right")}
        self.head = np.array([0.1, 0.0])
        self.targets = None
        self.goal = np.zeros(3)
        self.axis = np.array([0.0, 1.0, 0.0])
        self.box_center = None
        self.retries = 0
        self.carry_origin = None
        self.route = []
        self.hold_anchor = None
        self.contact_loss_since = None  # cross-skill bilateral-contact-loss timer
        self.holding = None  # "box_01" once lifted, None after placed
        self.placed_target = None  # station id once placed, None before
        self.stance_plans = []
        self._revision = 0
        self._seq = 0
        self._fixture = json.loads((ROOT / "config" / "workcell.json").read_text())

        self._publish_revision()

    # ------------------------------------------------------------------ #
    # sensor pipeline
    # ------------------------------------------------------------------ #

    def capture(self):
        """Render the head camera and refresh box/destination detections."""
        if self.runtime.d.time < self._next_vision:
            return
        self._next_vision = self.runtime.d.time + CFG["vision_period"]
        frame = self.camera.capture()
        box = detect(frame, "box")
        destination = detect(frame, "destination")
        destination_c = detect(frame, "destination_c")
        changed = False
        if box is not None:
            if self.box is None or not _same_pose(self.box, box):
                changed = True
            self.box = box
        offsets = self._fixture.get(
            "marker_offset_y", {"station_b": 0.75, "station_c": 0.75}
        )
        if destination is not None:
            if self.destination is None or not _same_pose(self.destination, destination):
                changed = True
            self.destination_marker = destination.center.copy()
            destination.center = (
                destination.center - offsets["station_b"] * destination.long_axis
            )
            self.destination = destination
        if destination_c is not None:
            if self.destination_c is None or not _same_pose(self.destination_c, destination_c):
                changed = True
            self.destination_marker_c = destination_c.center.copy()
            destination_c.center = (
                destination_c.center - offsets["station_c"] * destination_c.long_axis
            )
            self.destination_c = destination_c
        if changed:
            self._revision += 1

    def sense(self):
        self.touch = sense(self.runtime)

    def close(self):
        self.camera.close()

    # ------------------------------------------------------------------ #
    # robot actuation (delegates to the shared Runtime)
    # ------------------------------------------------------------------ #

    def step(self, command, arms=None, hands=None, head=None, stationary=False):
        self.runtime.step(command, arms, hands, head, stationary)

    # ------------------------------------------------------------------ #
    # world contract
    # ------------------------------------------------------------------ #

    def _publish_revision(self):
        self._revision += 1

    def view(self) -> WorldState:
        from ..contracts import CapabilityState, SkillName

        placeholder = CapabilityState(available=tuple(SkillName), blocked={})
        return self._build(placeholder)

    def snapshot(self, capabilities: CapabilityState) -> WorldState:
        self._seq += 1
        return self._build(capabilities)

    def destination_for(self, target_id: str):
        """Visually-localised destination for a station id (``station_b`` default).

        Section 11: the destination *position* comes from the marker, never from
        scene ground truth. ``station_c`` maps to the blue marker, everything
        else to the green marker.
        """
        if target_id == "station_c":
            return self.destination_c
        return self.destination

    def destination_marker_for(self, target_id: str):
        """The marker centre (before the table-centre back-off) for a station id."""
        if target_id == "station_c":
            return self.destination_marker_c
        return self.destination_marker

    def marker_bearing(self, target_id: str) -> np.ndarray:
        """Known approximate marker position, used only to orient the head for
        OBSERVE_TARGET (section 11: fixture position may come from a known map;
        the exact pose always comes from the visual marker)."""
        key = "destination_marker_c" if target_id == "station_c" else "destination_marker_b"
        return np.asarray(self._fixture.get(key, [1.23, 0.75, 0.8105]))

    def _station_state(self, station_id: str, destination) -> StationState:
        """A StationState for a fixture whose position is known but whose
        occupancy is a V1 fixture assumption (single-target scene, section 11)."""
        if self.placed_target == station_id:
            status = StationStatus.OCCUPIED
        elif destination is not None:
            status = StationStatus.FREE
        else:
            status = StationStatus.UNKNOWN
        return StationState(
            station_id=station_id,
            status=status,
            source=EvidenceSource.KNOWN_FIXTURE_MAP,
            observed_at_s=float(destination.time) if destination is not None else 0.0,
        )

    def _build(self, capabilities: CapabilityState) -> WorldState:
        now = float(self.runtime.d.time)

        # box entity: proprioception while held, vision while on the source table
        if self.holding is not None:
            # While held, the box position is a body estimate from the two grasp
            # sites -- NOT fresh RGB-D vision. It must be labelled as
            # proprioception, not as a visual marker (section 9.3 / 25.5).
            grasp = np.mean(
                [self.runtime.d.site(p + "grasp").xpos for p in ("lh_", "rh_")], axis=0
            )
            box = EntityObservation(
                entity_id="box_01",
                category="box",
                x=float(grasp[0]),
                y=float(grasp[1]),
                visible=True,
                observed_at_s=round(now, 6),
                source=EvidenceSource.PROPRIOCEPTION,
            )
        elif self.box is not None:
            box = EntityObservation(
                entity_id="box_01",
                category="box",
                x=float(self.box.center[0]),
                y=float(self.box.center[1]),
                visible=True,
                observed_at_s=float(self.box.time),
                source=EvidenceSource.RGBD_MARKER,
            )
        else:
            box = EntityObservation(
                entity_id="box_01",
                category="box",
                x=None,
                y=None,
                visible=False,
                observed_at_s=0.0,
                source=EvidenceSource.RGBD_MARKER,
            )

        # stations
        station_a = StationState(
            station_id="station_a",
            status=StationStatus.FREE,
            source=EvidenceSource.KNOWN_FIXTURE_MAP,
            observed_at_s=0.0,
        )
        station_b = self._station_state("station_b", self.destination)
        station_c = self._station_state("station_c", self.destination_c)

        robot = RobotState(
            holding_object_id=self.holding,
            current_skill=None,
            hand_contact_n=float(max(self.touch["left"], self.touch["right"])),
        )

        return WorldState.from_dict(
            {
                "schema_version": "1.0",
                "world_revision": self._revision,
                "snapshot_seq": self._seq,
                "sim_time_s": round(now, 6),
                "robot": robot.to_dict(),
                "entities": {"box_01": box.to_dict()},
                "stations": {
                    "station_a": station_a.to_dict(),
                    "station_b": station_b.to_dict(),
                    "station_c": station_c.to_dict(),
                },
                "capabilities": capabilities.to_dict(),
            }
        )


def _same_pose(a, b) -> bool:
    return bool(np.linalg.norm(a.center - b.center) < 0.02)
