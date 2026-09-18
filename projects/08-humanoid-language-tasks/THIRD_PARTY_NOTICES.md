# 第三方来源与许可

- EngineAI T800 模型、配置和预训练策略来自
  https://github.com/engineai-robotics/engineai_robotics_native_sdk ，
  固定提交 `335c60e88772c26c7852d0abd6b3c7439037dd8f`。
  许可保留于 `licenses/EngineAI-BSD-3-Clause.txt`；步态非本项目训练。
- 左右 Allegro 手来自 https://github.com/google-deepmind/mujoco_menagerie ，
  固定提交 `8161bba264d7fa7c99ca301e91e7fb44737676ad` 的 `wonik_allegro`。
  许可保留于 `licenses/Allegro-BSD-2-Clause.txt`。
- 本项目从 Zephastra 007 实验快照一次性导入并适配机器人、视觉、触觉和站位代码。
  文件映射与资源哈希见 `assets/baseline_manifest.json` 和 `docs/BASELINE_IMPORT.md`。
  此来源不是稳定基线；导入后独立维护，不运行或导入 007。
- 双手刚性安装、头部相机与工装为实验配置，不是厂商支持的真机配置。
  Apache-2.0 不替代第三方 BSD 许可，也不意味着厂商背书。
- Git 保留组合 XML、配置、来源清单及许可；大网格、纹理、权重由本项目
  `scripts/prepare_assets.py` 从上述固定版本获取并校验，不使用竞赛资产包。
