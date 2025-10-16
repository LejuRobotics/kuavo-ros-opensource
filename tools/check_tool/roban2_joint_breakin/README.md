# Roban2机器人统一磨线启动脚本

## 简介
用于启动Roban2机器人手臂和腿部磨线测试的统一脚本。支持单独运行手臂磨线、单独运行腿部磨线，或同时运行两个磨线程序。

## 注意事项

⚠️ **重要提醒**：
- 请将机器人吊起
- 腿部+腰部：将电机摆正到零点，包括腰部（大腿电机则直接自然垂直向下）
- 手部：将电机摆正到零点位置。

## 使用方法

### 0. 腿部磨线脚本编译（可选）
若有使用腿部磨线功能，将会自动检测并编译腿部磨线脚本，无需手动编译。
如需手动编译，可执行以下命令：
```bash
cd roban2_leg_breakin
```
```bash
mkdir build && cd build
```
```bash
cmake ..
```
```bash
make
```
### 1. 运行脚本
```bash
# 进入脚本目录
cd roban2_joint_breakin
```

```bash
# 使用root权限运行
sudo python3 roban2_joint_breakin.py
```

## 运行模式说明

### 模式1：仅启动手臂磨线
- 自动执行零点设置
- 12秒倒计时后开始磨线
- 使用RUIWO电机控制

### 模式2：仅启动腿部磨线
- 立即开始执行
- 使用EC_Master电机控制

### 模式3：同时启动手臂、腿部磨线
- 两个程序并行运行
- 手臂磨线：延迟12秒后开始，等待腿部EC启动
- 腿部磨线：立即开始

### 模式4：仅零点设置
- 只执行手臂零点设置
- 不进行磨线测试

## 监控和停止

### 查看日志
```bash
# 查看手臂磨线日志
tail -f /tmp/roban2_breakin_logs/arm_breakin.log

# 查看腿部磨线日志
tail -f /tmp/roban2_breakin_logs/leg_breakin.log
```

### 查看进程
```bash
ps aux | grep -E '(arm_breakin|roban2_leg_breakin)'
```

### 停止程序
```bash
# 停止特定进程（使用实际PID）
kill <进程ID>

# 停止所有磨线进程
pkill -f "arm_breakin"
pkill -f "roban2_leg_breakin"
```
