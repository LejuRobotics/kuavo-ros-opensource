---
title: "Kuavo 5-W 快速调试"
---

# Kuavo 5-W 快速调试

## 型号支持

Kuavo 5-W

## 使用流程概览

- 开机并连接
- 零点校准
- 基础控制

## 基础操作步骤

### 机器人开机

长按机器人底盘上的绿色按钮进行底盘开机，按下机器人背后开关对本体进行开机。

### 启动前准备

[启动前准备](./Kuavo%205-W%20启动前准备.md)

### 零点校准

[全身零点标定](./Kuavo%205-W%20全身零点标定.md)

💡 校正失败时：按下急停断电，重新调整零点姿态后重试。


### 底盘运动

1、新建终端，执行以下命令启动机器人：

```txt
cd kuavo-ros-opensource  
sudo su  
source devel/setup.bash  
roslaunch humanoid_controllers load_kuavo_real_wheel.launch
```

启动后，在终端按 `o` 使机器人站起。

2、开启第 2 个终端，执行底盘运动：

```txt
cd kuavo-ros-opensource   
sudo su   
source devel/setup.bash   
cd src/demo/test_kuavo_wheel_real   
python3 cmd_pos_base_test.py
```

底盘将按顺序执行以下动作：

- 向前 15 cm
- 原地逆时针旋转 30°
- 原地顺时针旋转 30°
- 向右 15 cm
- 向后 30 cm
- 向左 30 cm
- 向前 30 cm
- 向右 15 cm（回到原点）

### 北通遥控底盘控制

1、将北通遥控器接收器插入下位机 USB 口，完成遥控器与机器人配对。

2、在终端启动机器人：

```txt
cd kuavo-ros-opensource  
sudo su  
source devel/setup.bash  
roslaunch humanoid_controllers load_kuavo_real_wheel.launch joystick_type:=bt2
```

若无反应，可将参数改为 `joystick_type:=bt2pro` 后重试；启动后在终端按 `o` 使机器人站起。

- 左摇杆：控制底盘行走
- 右摇杆：控制底盘原地转向
- 退出：按遥控器 `Back` 键，或在终端使用 `Ctrl+C`
