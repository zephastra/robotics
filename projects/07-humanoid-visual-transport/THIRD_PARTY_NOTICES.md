# Third-party notices

EngineAI T800 model, configuration, pretrained policy, and adapted example:
https://github.com/engineai-robotics/engineai_robotics_native_sdk

Pinned revision: `335c60e88772c26c7852d0abd6b3c7439037dd8f`.
BSD-3-Clause notice retained in `licenses/EngineAI-BSD-3-Clause.txt` and
`policies/t800/LICENSE-EngineAI.txt`. The walking policy was not trained here.

Allegro left/right hands:
https://github.com/google-deepmind/mujoco_menagerie/tree/8161bba264d7fa7c99ca301e91e7fb44737676ad/wonik_allegro

Pinned revision: `8161bba264d7fa7c99ca301e91e7fb44737676ad`.
BSD-2-Clause notice retained in `licenses/Allegro-BSD-2-Clause.txt`.
Original finger dynamics retained; names prefixed and both hands attached
through experimental rigid adapters replacing the wrist placeholder spheres.

Integration code uses the root Apache-2.0 license except where upstream notices
apply. No endorsement or manufacturer-supported hardware configuration is implied.

Camera conventions follow MuJoCo documentation: forward is camera negative Z,
up is positive Y. The camera is attached to the moving head body, not the viewer:
https://mujoco.readthedocs.io/en/stable/programming/visualization.html
