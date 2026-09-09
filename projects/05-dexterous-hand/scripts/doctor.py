import os
import platform
import tkinter
import mujoco
from hand005.model import verify_assets, load

manifest = verify_assets()
m, d = load()
for _ in range(100):
    mujoco.mj_step(m, d)
print('Python:', platform.python_version(), '| MuJoCo:', mujoco.__version__, '| Tk:', tkinter.TkVersion)
print('Pinned Allegro revision:', manifest['revision'])
print('Assets: OK | 16 finger actuators + 2 fixture axes:', m.nu == 18)
print('Physics step: OK | Attachment constraints:', m.neq)
print('Display:', os.environ.get('DISPLAY', 'not set — use --mode demo --headless'))
print('This checks imports and physics, not human GUI input or hardware readiness.')
