#  kuavo_led
## rules
- 本规则旨在固定 LED 串口设备的串口号，以设备号为判断标准来判断是否有 LED 设备接入，并重命名串口号。
- [udev规则](./rules/99-led.rules)
### 使用说明
- 本文档只适用于Roban机器人，kuavo机器人灯条位于上位机。
- 将规则文件放到以下目录：
```bash
    sudo cp ~/kuavo-ros-control/src/kuavo_led/rules/99-led.rules /etc/udev/rules.d/
    加载规则：
    sudo udevadm control --reload-rules
    sudo udevadm trigger

```
- 完成后重启设备或热插拔可以在 /dev 目录下看到 ttyLED0 的设备号：
```bash
    ls /dev/ttyLED*
```
## kuavo_led_controller
- 本节点用于控制机器人头部的 LED 灯带，支持多种显示模式，包括常亮、呼吸、闪烁和律动等效果。

## 使用说明

### 安装依赖
- 确保系统已安装以下依赖：
```bash
    pip install pyserial
```
### 编译和启动
```bash
    catkin build  kuavo_led_controller
    source devel/setup.bash
    roslaunch kuavo_led_controller set_led_mode.launch
```
## 接口说明
- 本节点主要维护两个服务：
1. control_led:
- 控制灯带的服务，用户通过调用该服务来控制灯带的十个灯的颜色，服务的消息如下定义：
```bash
    # 请求部分
uint8 mode      #用于控制灯带的模式：0 常亮，1 呼吸，2 闪烁，3 律动（该模式下灯带颜色固定）
uint8[3] color1 #第一颗灯的颜色[r,g,b]--->[(0~255),(0~255),(0~255)],靠近 FPC 连接口的为第一颗灯。
uint8[3] color2 
uint8[3] color3 
uint8[3] color4 
uint8[3] color5 
uint8[3] color6 
uint8[3] color7 
uint8[3] color8 
uint8[3] color9 
uint8[3] color10 
---
# 响应部分
bool success   
```
2. close_led:
- 关闭灯带的服务，用户可以通过调用该服务来关闭灯带。
## 测试用例
一、 灯带串口通信测试
- [led_test](./kuavo_led_controller/test/led_test.py)
- 该文件直接与硬件通信，不需要依赖于 ros 节点，用于验证串口是否正常以及灯带硬件是否正常。
- 使用方法：
```bash
    cd ~/kuavo_ros_application/src/kuavo_led/kuavo_led_controller/test
    python3 led_test.py
```
- 现象为：
1. 常亮模式 3 秒的红灯
2. 呼吸模式 3 秒的绿灯
3. 闪烁模式 3 秒的蓝灯
4. 律动模式 3 秒。
---
二、 灯带服务控制测试：
- [kuavo_led_controller](./kuavo_led_controller/test/kuavo_led_client.py)
- 该文件需要启动节点 kuavo_led_controller，参考本文档的 **编译和启动** 一栏。
- 使用方法： 
```bash
- 使用方法：
```bash
    cd ~/kuavo_ros_application/src/kuavo_led/kuavo_led_controller/test
    python3 kuavo_led_client.py
```
- 现象为：常亮的彩虹状灯持续 5 秒，然后熄灭。

## LED服务使用指南

### 服务列表

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/control_led` | SetLEDMode | 控制10个LED灯的颜色和模式 |
| `/control_led_free` | SetLEDMode_free | 使用Color消息数组的LED控制服务（推荐） |
| `/close_led` | Trigger | 关闭所有LED灯 |

### 显示模式说明

| mode值 | 模式名称 | 效果说明 |
|--------|----------|----------|
| 0 | 常亮 | LED保持指定颜色不变 |
| 1 | 呼吸 | LED颜色从暗到亮循环变化 |
| 2 | 闪烁 | LED快速闪烁 |
| 3 | 律动 | LED产生波浪律动效果 |

### 颜色设置说明

每个LED灯的颜色使用RGB格式，取值范围为0-255：
- `color1` 到 `color10` 分别对应第1到第10颗LED灯
- 颜色格式为 `[R, G, B]`，例如 `[255, 0, 0]` 表示红色
- 靠近FPC连接接口的为第1颗灯

### 常用颜色参考

| 颜色 | RGB值 |
|------|-------|
| 红色 | [255, 0, 0] |
| 绿色 | [0, 255, 0] |
| 蓝色 | [0, 0, 255] |
| 黄色 | [255, 255, 0] |
| 青色 | [0, 255, 255] |
| 紫色 | [128, 0, 128] |
| 白色 | [255, 255, 255] |
| 橙色 | [255, 165, 0] |
| 粉色 | [255, 192, 203] |
| 熄灭 | [0, 0, 0] |

### 命令行使用示例

#### 1. 设置所有LED为红色常亮
```bash
rosservice call /control_led "mode: 0
color1: [255, 0, 0]
color2: [255, 0, 0]
color3: [255, 0, 0]
color4: [255, 0, 0]
color5: [255, 0, 0]
color6: [255, 0, 0]
color7: [255, 0, 0]
color8: [255, 0, 0]
color9: [255, 0, 0]
color10: [255, 0, 0]"
```

#### 2. 设置所有LED为绿色呼吸
```bash
rosservice call /control_led "mode: 1
color1: [0, 255, 0]
color2: [0, 255, 0]
color3: [0, 255, 0]
color4: [0, 255, 0]
color5: [0, 255, 0]
color6: [0, 255, 0]
color7: [0, 255, 0]
color8: [0, 255, 0]
color9: [0, 255, 0]
color10: [0, 255, 0]"
```

#### 3. 设置所有LED为蓝色闪烁
```bash
rosservice call /control_led "mode: 2
color1: [0, 0, 255]
color2: [0, 0, 255]
color3: [0, 0, 255]
color4: [0, 0, 255]
color5: [0, 0, 255]
color6: [0, 0, 255]
color7: [0, 0, 255]
color8: [0, 0, 255]
color9: [0, 0, 255]
color10: [0, 0, 255]"
```

#### 4. 设置彩虹渐变律动效果
```bash
rosservice call /control_led "mode: 3
color1: [255, 0, 0]
color2: [255, 127, 0]
color3: [255, 255, 0]
color4: [0, 255, 0]
color5: [0, 255, 255]
color6: [0, 0, 255]
color7: [75, 0, 130]
color8: [148, 0, 211]
color9: [255, 0, 127]
color10: [255, 255, 255]"
```

#### 5. 关闭所有LED
```bash
rosservice call /close_led
```

## 电池信息接口

本包同时集成了电池信息读取功能，通过 `battery_info_node.py` 节点实现。

### 话题列表
| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/battery_info_1` | `kuavo_led_controller/BatteryInfo` | BAT1（左电池）信息 |
| `/battery_info_2` | `kuavo_led_controller/BatteryInfo` | BAT2（右电池）信息 |

### BatteryInfo 消息定义
```msg
uint8 battery_id            # 电池ID (0:左电池, 1:右电池)
uint32 voltage              # 电压 (mV)
int32 current               # 电流 (mA，正数充电，负数放电)
uint32 remaining_capacity   # 剩余容量 (mAh)
uint32 full_capacity        # 满充容量 (mAh)
uint8 percentage            # 剩余电量百分比 (0-100)
uint16 cycle_count          # 放电循环次数
uint32 protection_flags     # 保护状态标志位
int16[6] temperatures       # 6个NTC温度 (摄氏度)
time timestamp              # 时间戳
```

### 电池功能使用示例
```bash
# 查看电池信息（话题订阅）
rostopic echo /battery_info_1

# 只查看电压和百分比
rostopic echo /battery_info_1 | grep -E 'voltage|percentage'

# 查看温度
rostopic echo /battery_info_1 | grep temperatures

# 查询电池信息（服务调用）
rosservice call /get_battery_info "battery_id: 0"

# 查询BAT2（右电池）
rosservice call /get_battery_info "battery_id: 1"
```

### 电池信息查询服务

**服务名**: `/get_battery_info`

**请求**:
```msg
# 电池ID (0:左电池/BAT1, 1:右电池/BAT2)
uint8 battery_id
```

**响应**:
```msg
# 电池ID
uint8 battery_id
# 电压 (mV)
uint32 voltage
# 电流 (mA)
int32 current
# 剩余容量 (mAh)
uint32 remaining_capacity
# 满充容量 (mAh)
uint32 full_capacity
# 剩余电量百分比 (0-100)
uint8 percentage
# 放电循环次数
uint16 cycle_count
# 保护状态标志位
uint32 protection_flags
# 温度传感器数据 (6个NTC, 摄氏度)
int16[6] temperatures
# 是否成功
bool success
# 消息
string message
```

**使用示例**：
```bash
# 查询BAT1信息
rosservice call /get_battery_info "battery_id: 0"
# 返回示例：
# battery_id: 0
# voltage: 59940
# current: -510
# percentage: 59
# success: True
# message: "电池0信息读取成功"

# 查询BAT2信息
rosservice call /get_battery_info "battery_id: 1"
```

### 启动参数
在 `set_led_mode.launch` 中可配置以下参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `LED` | `enable` | 是否启动LED节点 |
| `BATTERY` | `enable` | 是否启动电池节点 |
| `port` | `/dev/ttyLED0` | 串口设备路径（电池节点） |
| `baudrate` | `115200` | 波特率（电池节点） |
| `publish_rate` | `1.0` | 电池信息发布频率 (Hz) |

**灵活启动方式**：
```bash

# 启动LED
roslaunch kuavo_led_controller set_led_mode.launch

# 启动电池
roslaunch kuavo_led_controller battery_info.launch

# 启动机器人时启动
roslaunch humanoid_controllers load_kuavo_real.launch with_battery:=true（默认关闭）
```


## 硬件说明

### 电源板通信协议
| 指令 | 功能 | 说明 |
|------|------|------|
| 0x02 | LED灯条设置 | 控制10个LED灯的颜色和模式 |
| 0x03 | BAT1电池信息读取 | 读取左电池电压、电流、容量等 |
| 0x04 | BAT2电池信息读取 | 读取右电池电压、电流、容量等 |

### 注意事项
1. **硬件兼容性**：
   - 本包适用于Kuavo机器人的电源板（CH340, ID: `1a86:7523`）
   - 该电源板同时支持LED控制和电池读取功能
   - 不适用于Roban机器人的独立LED板（ID: `1a86:55d3`）

2. **串口共享**：LED和电池节点共用同一个串口设备 `/dev/ttyLED0`，实测多进程访问无冲突