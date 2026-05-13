# LED Strip ROS 服务接口说明

## 1. 服务概述

LED Strip ROS 服务提供对 LED 灯带的模式设置、颜色控制和关闭功能。通过 ROS 服务接口，其他节点可以方便地控制 LED 灯带的显示效果。

### 可用服务

| 服务名称 | 服务类型 | 功能 |
|---------|---------|------|
| `/led_strip_set_mode_and_color` | `kuavo_msgs/SetLEDMode_free` | 设置 LED 显示模式和颜色 |
| `/led_strip_close` | `std_srvs/Trigger` | 关闭所有 LED 灯 |

---

## 2. 服务详细说明

### 2.1 `/led_strip_set_mode_and_color`

设置 LED 显示模式和颜色，支持 24 个 LED 灯的独立颜色控制。

#### 服务类型

```
kuavo_msgs/SetLEDMode_free
```

#### 请求参数 (Request)

| 参数名 | 类型 | 说明 |
|--------|------|------|
| `mode` | `uint8` | 显示模式（见下表） |
| `colors` | `Color[24]` | 24 个 LED 灯的颜色列表，每个包含 `(r, g, b)` |

#### Color 消息定义

| 参数名 | 类型 | 取值范围 | 说明 |
|--------|------|----------|------|
| `r` | `uint8` | 0-255 | 红色分量 |
| `g` | `uint8` | 0-255 | 绿色分量 |
| `b` | `uint8` | 0-255 | 蓝色分量 |

#### 显示模式说明

| 模式值 | 模式名称 | 十六进制 | 说明 |
|--------|----------|----------|------|
| 0 | `CONSTANT` | `0x00` | 常亮模式 |
| 1 | `BREATHING` | `0x01` | 呼吸模式 |
| 2 | `FLASH` | `0x02` | 快闪模式 |
| 3 | `RHYTHM` | `0x03` | 律动模式（硬件自动生成效果，颜色数据无效） |

#### 响应参数 (Response)

| 参数名 | 类型 | 说明 |
|--------|------|------|
| `success` | `bool` | 设置成功返回 `True`，失败返回 `False` |

#### 注意事项

1. **颜色数量必须为 24 个**，否则服务会返回 `success=False`
2. **律动模式**（模式值 3）的颜色数据无效，硬件会自动生成固定效果
3. 串口被打开后会被独占，确保同一时间只有一个进程使用该串口

---

### 2.2 `/led_strip_close`

关闭所有 LED 灯（设置为全灭状态）。

#### 服务类型

```
std_srvs/Trigger
```

#### 请求参数 (Request)

无

#### 响应参数 (Response)

| 参数名 | 类型 | 说明 |
|--------|------|------|
| `success` | `bool` | 关闭成功返回 `True`，失败返回 `False` |
| `message` | `string` | 返回消息，描述操作结果 |

---

## 3. 使用示例

### 3.1 Python 示例

完整测试示例见：`src/kuavo_led/kuavo_led_controller/src/service/test/test_led_strip_service.py`

#### 示例 1：设置 24 个灯全红常亮

```python
import rospy
from kuavo_msgs.srv import SetLEDMode_free
from kuavo_msgs.msg import Color

def set_all_red():
    """设置 24 个灯全红常亮"""
    rospy.wait_for_service('/led_strip_set_mode_and_color')
    set_mode_service = rospy.ServiceProxy('/led_strip_set_mode_and_color', SetLEDMode_free)
    
    # 创建 24 个红色灯的颜色数据
    colors = []
    for i in range(24):
        color_msg = Color()
        color_msg.r = 255
        color_msg.g = 0
        color_msg.b = 0
        colors.append(color_msg)
    
    # 调用服务（模式 0 = 常亮）
    response = set_mode_service(mode=0, colors=colors)
    print(f"设置结果: {'成功' if response.success else '失败'}")

if __name__ == '__main__':
    rospy.init_node('led_test')
    set_all_red()
```

#### 示例 2：呼吸模式 - 绿灯

```python
import rospy
from kuavo_msgs.srv import SetLEDMode_free
from kuavo_msgs.msg import Color

def breathing_green():
    """设置呼吸模式，24 个灯全绿"""
    rospy.wait_for_service('/led_strip_set_mode_and_color')
    set_mode_service = rospy.ServiceProxy('/led_strip_set_mode_and_color', SetLEDMode_free)
    
    # 创建 24 个绿色灯的颜色数据
    colors = []
    for i in range(24):
        color_msg = Color()
        color_msg.r = 0
        color_msg.g = 255
        color_msg.b = 0
        colors.append(color_msg)
    
    # 调用服务（模式 1 = 呼吸）
    response = set_mode_service(mode=1, colors=colors)
    print(f"设置结果: {'成功' if response.success else '失败'}")
```

#### 示例 3：关闭所有 LED

```python
import rospy
from std_srvs.srv import Trigger

def close_leds():
    """关闭所有 LED 灯"""
    rospy.wait_for_service('/led_strip_close')
    close_service = rospy.ServiceProxy('/led_strip_close', Trigger)
    
    response = close_service()
    print(f"关闭结果: {'成功' if response.success else '失败'}, 消息: {response.message}")

if __name__ == '__main__':
    rospy.init_node('led_test')
    close_leds()
```

---

## 4. 完整测试流程

运行测试脚本可以验证所有模式：

```bash
# 首先启动 LED 服务
roslaunch kuavo_led_controller set_led_mode.launch
# 进入到kuavo-ros-control仓库下
# 在另一个终端运行测试
python3 src/kuavo_led/kuavo_led_controller/src/service/test/test_led_strip_service.py
```

测试流程：
1. **常亮模式 - 红灯** (3秒)
2. **呼吸模式 - 绿灯** (3秒)
3. **闪烁模式 - 蓝灯** (3秒)
4. **律动模式** (3秒，硬件自动生成效果)
5. **关闭所有灯**

---

## 5. 命令行调用示例

### 使用 rosservice call

#### 设置常亮模式（红色）

```bash
rosservice call /led_strip_set_mode_and_color "mode: 0
colors:
- {r: 255, g: 0, b: 0}
- {r: 255, g: 0, b: 0}
# ... (重复 24 个颜色数据)
"
```

#### 关闭所有 LED

```bash
rosservice call /led_strip_close
```

---

## 6. 常见问题

### Q1: 服务调用失败，返回 "service not available"

**解决方案**: 确保 LED 服务节点已启动：
```bash
rosrun kuavo_led_controller led_strip_service.py
```

### Q2: 设置颜色时返回 success=False

**可能原因**:
- 颜色数量不是 24 个
- 串口设备不存在（`/dev/ttyLED0` 未创建）
- 串口权限问题

**检查方法**:
```bash
# 检查设备是否存在
ls -l /dev/ttyLED0

# 查看服务节点日志
rosnode info /led_strip_service_node
```

### Q3: 律动模式设置后颜色没有变化

**说明**: 律动模式（模式值 3）由硬件自动生成固定效果，传入的颜色数据会被忽略，这是正常行为。

---

## 7. 相关文档

- LED 串口配置说明：`led_serial.md`
- 底层 LEDStrip 类实现：`src/controller/led/led_strip.py`
- 测试代码：`src/service/test/test_led_strip_service.py`
