# Third-party materials

## EngineAI T800

Source: https://github.com/engineai-robotics/engineai_robotics_native_sdk

Pinned revision: `335c60e88772c26c7852d0abd6b3c7439037dd8f`.

Model, configuration, pretrained walking policy, and adaptation of its walking
example are subject to the retained BSD-3-Clause license in
`licenses/EngineAI-BSD-3-Clause.txt` and `policies/t800/LICENSE-EngineAI.txt`.
The policy is upstream work, not trained by this project.

## Allegro right hand

Source: https://github.com/google-deepmind/mujoco_menagerie/tree/8161bba264d7fa7c99ca301e91e7fb44737676ad/wonik_allegro

Pinned revision: `8161bba264d7fa7c99ca301e91e7fb44737676ad`.

Retained BSD-2-Clause license: `licenses/Allegro-BSD-2-Clause.txt`.
The original hand parameters are retained; names are prefixed and the hand
is mounted through an experimental rigid adapter on the T800's right wrist.
This replaces only the wrist-end placeholder collision sphere.

## Project implementation

Project integration code is provided under the root Apache-2.0 license, except
where upstream notices specify otherwise. Upstream materials retain their
own licenses. This project does not imply endorsement or hardware compatibility
by EngineAI, Wonik/SimLab, Google DeepMind, or MuJoCo.
