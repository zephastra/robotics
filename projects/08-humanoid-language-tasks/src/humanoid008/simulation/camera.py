"""Sensor adapter: rendered RGB/depth plus calibrated robot camera extrinsics.

Adapted from the 007 baseline (section 6.1).
"""

import mujoco

from .vision import Frame


class HeadCamera:
    def __init__(self, r, width=320, height=240):
        self.r = r
        self.renderer = mujoco.Renderer(r.m, height, width)
        self.last = None

    def capture(self):
        self.renderer.disable_depth_rendering()
        self.renderer.update_scene(self.r.d, camera="eyes")
        rgb = self.renderer.render().copy()
        self.renderer.enable_depth_rendering()
        depth = self.renderer.render().copy()
        self.renderer.disable_depth_rendering()
        camera = self.r.d.camera("eyes")
        self.last = Frame(
            rgb,
            depth,
            camera.xpos.copy(),
            camera.xmat.reshape(3, 3).copy(),
            float(self.r.m.camera("eyes").fovy[0]),
            float(self.r.d.time),
        )
        return self.last

    def close(self):
        self.renderer.close()
