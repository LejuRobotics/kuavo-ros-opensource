# kuavo_solver 文档索引

> **新用户？** 请先阅读 [`../QUICKSTART.md`](../QUICKSTART.md) 了解 5 分钟快速上手指南，包括前置条件、常用工作流、命令速查和排障。

本目录的文档面向 **代码维护者**，聚焦 `kuavo_solver` 五个几何解算器的**原理** + **代码对照**。
更偏用户视角的整体架构（版本映射、MJCF 一致性、WebUI 可视化节点图等）见仓库级知识库：

- `kuavo-knowledge/ankle_solver_geometry.md`（踝：FixedAxis vs AxisOffset 的写实版）
- `kuavo-knowledge/knee-ankle-solver-architecture.md`（膝/踝解算器架构）

## 本目录文件

- [`geometry.md`](./geometry.md) — **踝 / 膝 / 肘 / 腕 / 腰** 五个关节的几何原理总纲，
  以统一的 `JacobianSystem` 抽象（`Jc·dq + Ja·dp = 0`）串起来。
- [`validation.md`](./validation.md) — 严格验证：双入口、canonical 多版本矩阵、误差定义与框架说明。

## 验证快速开始

```bash
cd kuavo-ros-control
catkin build kuavo_solver && source devel/setup.bash
pip3 install textual            # 终端 UI（验证中心 + 可视化表）

# 统一终端界面（推荐：单窗口完成可视化 / 验证 / 批量回归）
./src/kuavo_solver/scripts/validate_solver.sh
# 或
python3 src/kuavo_solver/scripts/solver_validation_hub.py

# 命令行
./src/kuavo_solver/scripts/validate_solver.sh smoke         # L1
./src/kuavo_solver/scripts/validate_solver.sh full          # L2
```

详见 [`validation.md`](./validation.md)。
