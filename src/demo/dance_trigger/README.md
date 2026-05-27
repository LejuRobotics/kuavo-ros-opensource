# 舞蹈触发示例 trigger_dance

通过 ROS 服务触发机器人舞蹈，命令发送成功即提示用户。适用于无手柄、自动化测试或上位机远程触发等场景。

能否触发完全交由控制器判定：服务返回 `success=True` 即触发成功；返回 `False` 说明当前不允许切换（例如正在跳舞、舞蹈列表为空等），脚本据此提示用户，无需自行做状态检测。

## 前置条件

- 机器人控制器（`humanoid_controller`）已正常启动。
- 当前机型已启用舞蹈控制器：`rl_controllers.yaml` 中 `dance_controller` 为 `enabled: true`，且 `dance_param.info` 指向存在的模型文件。
- 机器人处于可进入舞蹈的状态（通常需站立 / stance）。

## 快速开始

```bash
source devel/setup.bash
python3 trigger_dance.py
```

不带参数运行时触发舞蹈列表中的第一支。

## 命令参数

| 命令 | 说明 |
| --- | --- |
| `trigger_dance.py` | 触发首支舞 |
| `trigger_dance.py --dance "#1"` | 触发列表中下标为 1 的舞蹈 |
| `trigger_dance.py --dance <名称>` | 按名称触发指定舞蹈 |
| `trigger_dance.py --list` | 列出所有可用舞蹈及其下标 |

`--dance` 取值规则：

- `""`（空，默认）：舞蹈列表中的第一支。
- `"#0"`、`"#1"` …：按下标选择。
- 其它字符串：按舞蹈名称匹配。

退出码：`0` 触发成功；`1` 触发失败或当前不可切换；`130` 用户 `Ctrl+C`。

## 涉及的 ROS 服务

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `/humanoid_controller/switch_to_dance_controller` | `kuavo_msgs/SetString` | 触发舞蹈，`data` 为空 / `#N` / 舞蹈名 |
| `/humanoid_controller/get_dance_controller_list` | `kuavo_msgs/GetStringList` | 获取可用舞蹈列表 |
