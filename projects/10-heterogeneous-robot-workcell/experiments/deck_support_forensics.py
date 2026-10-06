"""Independent cached-state support measurement; never produces permission/control.

mj_step leaves contacts/transforms from the pre-integration state. The caller
labels samples with data.time - timestep, not the advanced integration clock.
No mj_forward/mj_step or physical state write is performed by this instrument.
"""
import math
import mujoco
import numpy as np


def cylinder_plane_gap(floor_center, floor_rotation, floor_half, cylinder_center,
                       cylinder_axis, radius, half_length):
    """Analytic signed plane clearance with conservative footprint overlap.

    This is not a general convex distance: a positive value proves separation
    from the floor's lower plane; negative is NOT proof of penetration/contact.
    """
    axes = np.asarray(floor_rotation).reshape(3, 3)
    axis = np.asarray(cylinder_axis)
    delta = np.asarray(floor_center)-np.asarray(cylinder_center)
    extent = []
    for i in range(3):
        cosine = float(np.clip(np.dot(axes[:, i], axis), -1., 1.))
        extent.append(half_length*abs(cosine)+radius*math.sqrt(max(0., 1.-cosine*cosine)))
    if any(abs(float(np.dot(delta, axes[:, i]))) > floor_half[i]+extent[i] for i in (0, 1)):
        return None
    return float(np.dot(delta, axes[:, 2]))-floor_half[2]-extent[2]


class SeparationEpisodes:
    def __init__(self):
        self.active = None
        self.episodes = []
        self.samples = 0
        self.last_time = None

    def sample(self, sim_s, contact_count, gap=None, force=0.):
        if not math.isfinite(sim_s) or (self.last_time is not None and sim_s <= self.last_time):
            raise ValueError('strictly increasing finite sample times required')
        self.last_time = sim_s
        self.samples += 1
        if contact_count == 0:
            if self.active is None:
                self.active = dict(start_sim_s=sim_s, last_missing_sim_s=sim_s,
                                   samples=0, peak_signed_gap_m=None, min_signed_gap_m=None,
                                   max_normal_force_n=0.)
            row = self.active
            row['last_missing_sim_s'] = sim_s
            row['samples'] += 1
            row['max_normal_force_n'] = max(row['max_normal_force_n'], force)
            if gap is not None:
                row['peak_signed_gap_m'] = max(row['peak_signed_gap_m'], gap) if row['peak_signed_gap_m'] is not None else gap
                row['min_signed_gap_m'] = min(row['min_signed_gap_m'], gap) if row['min_signed_gap_m'] is not None else gap
        elif self.active is not None:
            row = dict(self.active, recovered_sim_s=sim_s,
                       duration_until_recovery_s=sim_s-self.active['start_sim_s'], censored=False)
            self.episodes.append(row)
            self.active = None

    def summary(self):
        rows = list(self.episodes)
        if self.active is not None:
            rows.append(dict(self.active, recovered_sim_s=None, censored=True,
                             observed_missing_span_s=self.active['last_missing_sim_s']-self.active['start_sim_s']))
        return dict(samples=self.samples, episodes=rows, completed_episodes=len(self.episodes),
                    last_sample_sim_s=self.last_time, permission_output=False)


class DeckSupportForensics:
    def __init__(self, model):
        self.model = model
        self.floor = model.geom('c_tray_floor').id
        if model.geom_type[self.floor] != mujoco.mjtGeom.mjGEOM_BOX:
            raise ValueError('analytic probe requires declared box tray floor')
        self.tray_body = model.body('c_payload').id
        self.deck = tuple(g for g in range(model.ngeom)
                          if (model.geom(g).name or '').startswith('c_deck_roller')
                          and (model.geom_contype[g] or model.geom_conaffinity[g]))
        if not self.deck:
            raise ValueError('no collision-enabled deck support geometries')
        self.deck_set = set(self.deck)
        self.episodes = SeparationEpisodes()
        self.first_missing = None
        self.max_gap_record = None

    def sample(self, data):
        count = 0
        normal_force = 0.
        for index, contact in enumerate(data.contact[:data.ncon]):
            geoms = (int(contact.geom1), int(contact.geom2))
            if any(g in self.deck_set for g in geoms) and any(
                    int(self.model.geom_bodyid[g]) == self.tray_body for g in geoms):
                count += 1
                force = np.empty(6)
                mujoco.mj_contactForce(self.model, data, index, force)
                normal_force += max(0., float(force[0]))
        # Signed surface distance is measured only for missing-support samples.
        # No contact does NOT automatically mean positive geometric separation.
        record = None
        if count == 0:
            distances = [(float(mujoco.mj_geomDistance(self.model, data, self.floor, g, .1, None)), g)
                         for g in self.deck]
            gap, geom = min(distances)
            planes = []
            for g in self.deck:
                if self.model.geom_type[g] != mujoco.mjtGeom.mjGEOM_CYLINDER:
                    raise ValueError('analytic probe requires declared cylindrical deck rollers')
                value = cylinder_plane_gap(data.geom_xpos[self.floor], data.geom_xmat[self.floor],
                    self.model.geom_size[self.floor], data.geom_xpos[g],
                    data.geom_xmat[g].reshape(3, 3)[:, 2], *self.model.geom_size[g][:2])
                if value is not None: planes.append((value, g))
            plane, plane_geom = min(planes) if planes else (None, None)
            record = dict(cached_state_sim_s=float(data.time)-float(self.model.opt.timestep),
                          min_signed_surface_gap_m=gap, distance_search_limit_m=.1,
                          distance_censored=gap >= .1, closest_geom=self.model.geom(geom).name,
                          tray_xyz=data.body('c_payload').xpos.tolist(),
                          deck_xyz=data.body('c_deck').xpos.tolist(),
                          tray_rotation=data.body('c_payload').xmat.tolist(),
                          deck_rotation=data.body('c_deck').xmat.tolist(),
                          analytic_plane_clearance_m=plane,
                          analytic_closest_geom=self.model.geom(plane_geom).name if plane_geom is not None else None,
                          analytic_scope='lower-plane separation bound; negative does not prove penetration')
            if self.first_missing is None:
                self.first_missing = record
            if plane is not None and (self.max_gap_record is None or
                    plane > self.max_gap_record['analytic_plane_clearance_m']):
                self.max_gap_record = record
        self.episodes.sample(float(data.time)-float(self.model.opt.timestep), count,
                             record['analytic_plane_clearance_m'] if record else None, normal_force)

    def summary(self):
        return dict(self.episodes.summary(), first_missing=self.first_missing,
                    max_gap_record=self.max_gap_record, instrument='cached_solver_state_2ms',
                    timing='post-mj_step contacts/transforms labelled time minus timestep',
                    distance='episodes use analytic plane clearance; raw convex distance separately preserved/unqualified',
                    scope='JUDGE_ONLY; no control or acceptance threshold changes')
