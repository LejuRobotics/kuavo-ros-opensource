# kuavo_solver 文档索引

本目录的文档面向 **代码维护者**，聚焦 `kuavo_solver` 五个几何解算器的**原理** + **代码对照**。
更偏用户视角的整体架构（版本映射、MJCF 一致性、WebUI 可视化节点图等）见仓库级知识库：

- `kuavo-knowledge/ankle_solver_geometry.md`（踝：FixedAxis vs AxisOffset 的写实版）
- `kuavo-knowledge/knee-ankle-solver-architecture.md`（膝/踝解算器架构）

## 本目录文件

- [`geometry.md`](./geometry.md) — **踝 / 膝 / 肘 / 腕 / 腰** 五个关节的几何原理总纲，
  以统一的 `JacobianSystem` 抽象（`Jc·dq + Ja·dp = 0`）串起来。
- [`validation.md`](./validation.md) — 严格验证（`scripts/mujoco_unified_cli.py` + `scripts/validation/`）的对齐逻辑、
  MuJoCo site/efc_J 与 solver 解析雅可比的比对约定。
