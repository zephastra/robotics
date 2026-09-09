# Third-party provenance

## Allegro Hand V3

- Source: https://github.com/google-deepmind/mujoco_menagerie/tree/8161bba264d7fa7c99ca301e91e7fb44737676ad/wonik_allegro
- Pinned revision: `8161bba264d7fa7c99ca301e91e7fb44737676ad`.
- Model: `right_hand.xml`, Allegro V3 right hand, four fingers and 16 articulated joints.
- Upstream description derives from SimLab's Allegro URDF. Copyright (c) 2016, SimLab.
- License: BSD-2-Clause, retained at `licenses/Allegro-BSD-2-Clause.txt` by setup.
- No upstream hardware-control scripts or pretrained policies are executed.

The preparation script retains original meshes, finger kinematics, masses, collision shapes, position actuators and joint ranges. It changes the palm's mounting orientation, adds a simulated two-axis position-actuated wrist fixture, fingertip sites, lighting, a free spherical object, source pedestal and receiving tray. Integration is 2 ms with implicitfast. Contact and receiving-pad assumptions are documented in README; they are not measured material properties.

No object attachment, equality weld, animation, or run-time root-pose manipulation is used to simulate a successful grasp. The upstream robot description is simplified and this work does not assert hardware equivalence, affiliation, endorsement, or real-robot readiness.

MuJoCo, NumPy, Pillow, pytest and their dependencies retain their own licenses. Original project code is licensed under Apache-2.0.
