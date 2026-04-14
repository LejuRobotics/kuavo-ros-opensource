# LED 灯带串口控制模块使用说明

## 1. 使用环境

### 硬件要求
- 设备装有 Kuavo 5 胸部灯带

### 串口配置

运行 `setup_udev_rules.sh` 脚本来配置串口规则：

```bash
cd /path/to/kuavo-ros-control/src/kuavo_led  # 替换为你的仓库路径
./setup_udev_rules.sh
```

**注意：执行脚本后需要重启设备。**

重启后，检查设备是否成功加载：

```bash
# 检查 /dev/ttyLED0 设备是否存在
ls -l /dev/ttyLED0

# 或者查看 udev 规则是否生效
udevadm info -a -n /dev/ttyLED0 | grep ATTRS{idVendor}
```

如果设备节点成功创建，会显示类似 `/dev/ttyLED0 -> ttyUSBX` 的信息。

---

## 2. 快速开始

### 初始化

```python
from led.led_strip import LEDStrip, LEDMode

# 初始化 LED 灯带（自动打开串口）
led_strip = LEDStrip(port='/dev/ttyLED0', baudrate=115200)
```

### 设置模式和颜色

使用 `set_mode_and_color()` 方法设置 LED 的显示模式和颜色,设置颜色的时候需要设置24个LED的RGB颜色值：

```python
# 示例：设置 24 个灯全部为红色常亮
red_colors = [(255, 0, 0)] * LEDStrip.LED_COUNT
success = led_strip.set_mode_and_color(LEDMode.CONSTANT, red_colors)

if success:
    print("设置成功")
else:
    print("设置失败")
```

### 关闭所有 LED

使用 `close()` 方法关闭所有 LED 灯：

```python
led_strip.close()
```

---

## 3. API 说明

### 类: `LEDStrip`

#### 构造函数

```python
LEDStrip(port: str = '/dev/ttyLED0', baudrate: int = 115200)
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `port` | `str` | `'/dev/ttyLED0'` | 串口设备路径 |
| `baudrate` | `int` | `115200` | 波特率 |

#### 方法: `set_mode_and_color()`

设置 LED 显示模式和颜色，通过串口发送指令并校验硬件回显。

```python
set_mode_and_color(mode: LEDMode, colors: List[Tuple[int, int, int]]) -> bool
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `mode` | `LEDMode` | 显示模式枚举（见下表） |
| `colors` | `List[Tuple[int, int, int]]` | 24 个灯的颜色列表，每个为 `(R, G, B)` 元组，取值范围 0-255 |

**返回值**: `bool` - 发送和校验成功返回 `True`，失败返回 `False`

#### 方法: `close()`

关闭所有 LED 灯（设置为全灭状态），通过串口发送指令并校验硬件回显。

```python
close() -> bool
```

**返回值**: `bool` - 发送和校验成功返回 `True`，失败返回 `False`

---

### 模式说明: `LEDMode`

| 枚举值 | 十六进制 | 说明 |
|--------|----------|------|
| `LEDMode.CONSTANT` | `0x00` | 常亮模式 |
| `LEDMode.BREATHING` | `0x01` | 呼吸模式 |
| `LEDMode.FLASH` | `0x02` | 快闪模式 |
| `LEDMode.RHYTHM` | `0x03` | 律动模式（硬件自动生成效果，颜色数据无效） |

---

## 4. 使用示例

### 示例 1：24 灯全红常亮

```python
from led.led_strip import LEDStrip, LEDMode

led_strip = LEDStrip()

# 设置 24 个灯全红
red_colors = [(255, 0, 0)] * LEDStrip.LED_COUNT
led_strip.set_mode_and_color(LEDMode.CONSTANT, red_colors)

# 关闭所有灯
led_strip.close()
```

### 示例 2：渐变颜色效果

```python
from led.led_strip import LEDStrip, LEDMode

led_strip = LEDStrip()

# 设置呼吸模式，24 个灯从红到蓝渐变
gradient_colors = []
for i in range(LEDStrip.LED_COUNT):
    r = int(255 * (1 - i / 23))
    b = int(255 * (i / 23))
    gradient_colors.append((r, 0, b))

led_strip.set_mode_and_color(LEDMode.BREATHING, gradient_colors)
```

### 示例 3：律动模式

```python
from led.led_strip import LEDStrip, LEDMode

led_strip = LEDStrip()

# 律动模式由硬件自动生成，颜色数据无效（传入任意值即可）
dummy_colors = [(0, 0, 0)] * LEDStrip.LED_COUNT
led_strip.set_mode_and_color(LEDMode.RHYTHM, dummy_colors)
```

---

## 5. 测试程序

`test/test_led.py` 是 LED 灯带的测试程序，用于验证各种显示模式和颜色的效果,使用示例也可以参考测试程序。

### 测试流程
1. 常亮模式 - 红灯（3秒）
2. 呼吸模式 - 绿灯（3秒）
3. 闪烁模式 - 蓝灯（3秒）
4. 律动模式 - 硬件自动生成效果（3秒）
5. 关闭所有灯

### 运行方式

```bash
cd /path/to/kuavo-ros-control/src/kuavo_led/kuavo_led_controller/src/controller/test
python3 test_led.py
```

### 输出示例

```
==================================================
LED 灯带测试程序
==================================================

[1] 初始化 LED 灯带...

[2] 常亮模式 - 红灯 (3秒)
[LEDStrip] 发送指令 (80 字节): FF FF 00 4C 02 02 00 FF 00 00 ...
[LEDStrip] 收到硬件回显: FF FF C8 02 00 35
[LEDStrip] 硬件回显校验成功
  设置结果: 成功
...

==================================================
测试序列执行完毕!
==================================================
```

---

## 6. 注意事项

1. **律动模式**：律动模式（`LEDMode.RHYTHM`）的颜色数据无效，硬件会自动生成固定的律动效果，传入的颜色数据会被忽略。
2. **串口占用**：串口被打开后会被独占，确保同一时间只有一个进程使用该串口。
3. **颜色数量**：必须传入恰好 24 个灯的颜色数据，否则会返回 `False`。
4. **错误处理**：建议在程序退出或发生异常时调用 `close()` 方法关闭所有 LED 灯。
