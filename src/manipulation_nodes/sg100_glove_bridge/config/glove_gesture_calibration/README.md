# 个人手势标定数据目录

本仓库默认在这里保存并读取个人手势标定，与启动账号 HOME 和终端当前目录无关。

```text
active.yaml          下次启动自动加载的配置（完整标定通过后生成）
profiles/            已通过验证的具名个人配置
recordings/          原始记录与断点文件
reports/             质量验证报告
*.before-*           替换配置前的备份
```

本机完整路径：`/home/chenzibo/kuavo_ws/kuavo-ros-control/src/manipulation_nodes/sg100_glove_bridge/config/glove_gesture_calibration/`。
当前工作区通过本机 `.git/info/exclude` 忽略数据文件，不会随代码提交或拉取自动同步；本说明文件正常跟踪。新克隆工作区的本地忽略规则需单独配置。
`--directory` 可覆盖保存目录；全身启动的 `glove_gesture_calibration_file` 可覆盖读取文件。
旧 `~/.ros/glove_gesture_calibration` 不再是默认读取位置，已有文件保留作备份。

使用步骤见[个人手势标定说明](../../docs/personal_gesture_calibration.md)。

在新克隆仓库根目录，可向 `.git/info/exclude` 添加以下本地规则（不修改远端保护的 `.gitignore`）：

```gitignore
/src/manipulation_nodes/sg100_glove_bridge/config/glove_gesture_calibration/*
!/src/manipulation_nodes/sg100_glove_bridge/config/glove_gesture_calibration/README.md
```
