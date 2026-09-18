"""Ideal simulated contact sensing; no object pose is exposed to the controller.

Adapted from the 007 baseline (section 6.1).
"""

import mujoco
import numpy as np


def sense(r):
    result = {"left": 0.0, "right": 0.0, "source": False, "destination": False}
    payload = r.m.body("payload").id
    for i in range(r.d.ncon):
        contact = r.d.contact[i]
        body1, body2 = r.m.geom_bodyid[[contact.geom1, contact.geom2]]
        if payload not in (body1, body2):
            continue
        other = contact.geom2 if body1 == payload else contact.geom1
        name = r.m.body(r.m.geom_bodyid[other]).name
        force = np.zeros(6)
        mujoco.mj_contactForce(r.m, r.d, i, force)
        if name.startswith("lh_"):
            result["left"] += max(float(force[0]), 0.0)
        if name.startswith("rh_"):
            result["right"] += max(float(force[0]), 0.0)
        # Match the station table by pattern, not by equality. Station C's
        # table geom is named "destination_c_table" (assets/combined.xml:772),
        # so an exact comparison against "destination_table" could NEVER be
        # true for C -- the payload could rest on the table forever and
        # touch["destination"] would stay False, hanging PlaceObject.LOWER
        # until its tick budget ran out (see docs/C_STATION_ROOTCAUSE.md).
        # The suffix test still rejects destination_c_leg, source_leg and
        # destination_marker_plate.
        geom_name = r.m.geom(other).name
        for station in ("source", "destination"):
            if (
                geom_name.startswith(station)
                and geom_name.endswith("_table")
                and force[0] > 0.01
            ):
                result[station] = True
    return result
