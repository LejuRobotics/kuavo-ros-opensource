# Can 总线配置说明文档

> 本文档适用于:
> 
> - Roban2: 双 Can 总线即左右手臂+末端各接一个 Can 模块
> - Kuavo4pro 和 Kuavo5: 双 Can 总线即左右手臂+末端各接一个 Can 模块
>
> 本文档暂不适用于(2026-1-10):
>
> - Roban2: 单 Can 总线即左右手臂接线一个 Can 模块
> - Kuavo4pro 和 Kuavo5: 单 Can 总线即左右手臂接线一个 Can 模块
>
> **Notes: 事实上, 目前的代码是支持 Roban2 单总线和 Kuavo 单总线的，但由于代码仅开发自测通过，未经集成部门严格测试，因此无法确保可在生产环境使用。**
> 

## 硬件配置

### 双 Can 总线

- 左右手臂各接一个 Can 模块, 左手臂为A款，右手臂为B款 (适用于 BUSTMUST 和自研 Can)
- 对于 revo1 和 revo2 灵巧手得单独刷支持 Can 通讯的固件，左右灵巧手分别接线到 A款和B款 Can 模块上
- 对于 自研夹爪，左右分别接线到 A款和B款 Can 模块上

## 软件配置

**软件上需要执行脚本来配置**:
```bash
./tools/check_tool/canbus_config.sh
```

双Can总线的机器应当包含如下配置文件:
- `~/.config/lejuconfig/CanbusWiringType.ini`用来表示canbus的接线方式，在双can机器上，内容为`dual_bus`
- `~/.config/lejuconfig/HandProtocolType.ini`用来表示灵巧手的通讯协议，485协议的内容为`proto_buf`，can协议的内容为`proto_can`
- `~/.config/lejuconfig/canbus_device_cofig.yaml`用于描述can模块与总线上的设备关系

**判断配置脚本是否成功:**
- `~/.config/lejuconfig/CanbusWiringType.ini` **文件存在且内容为`dual_bus`**
- `~/.config/lejuconfig/HandProtocolType.ini` **文件存在且内容为`proto_can`**
- `~/.config/lejuconfig/canbus_device_cofig.yaml`  文件存在且里面正确描述了左右Can的设备信息

**如果改回单总线，请删除`~/.config/lejuconfig/CanbusWiringType.ini`. 如果使用 485 协议灵巧手，请删除`~/.config/lejuconfig/HandProtocolType.ini`**

**如果改回单总线，请删除`~/.config/lejuconfig/CanbusWiringType.ini`. 如果使用 485 协议灵巧手，请删除`~/.config/lejuconfig/HandProtocolType.ini`**

**如果改回单总线，请删除`~/.config/lejuconfig/CanbusWiringType.ini`. 如果使用 485 协议灵巧手，请删除`~/.config/lejuconfig/HandProtocolType.ini`**

## 配置文件说明

目前默认的配置文件存放在`src/kuavo_assets/config`:
- `roban2_dual_canbus_cofig.yaml`:  Roban2 双 Can 总线基础模板
- `roban2_single_canbus_cofig.yaml`: Roban2 单 Can 总线基础模板
- `kuavo4pro_dual_canbus_cofig.yaml`: Kuavo4pro 双 Can 总线基础模板
- `kuavo4pro_single_canbus_cofig.yaml`: Kuavo4pro 单 Can 总线基础模板
- `kuavo5_dual_canbus_cofig.yaml`: Kuavo5 双 Can 总线基础模板
- `kuavo5_single_canbus_cofig.yaml`: Kuavo5 单 Can 总线基础模板

## 电机方向识别（可选步骤，发现手臂动作不符合预期时可执行本章节验证）

本步骤为可选步骤，用于识别电机方向是否正确。

**执行工具**:
```bash
./tools/check_tool/motorevo_tool.sh --negative
```

**工具功能**: motorevo_tool 测试工具，用于电机识别方向

## 配置文件字段说明

### canbus_interfaces 配置

每个CAN总线接口包含以下字段：

- `type`: CAN模块类型 (BUSMUST_A, BUSMUST_B, 自研CAN等)
- `name`: CAN模块的设备名称，用于识别具体硬件(预留字段，当前暂未使用)
- `nbitrate`: 仲裁段波特率 (单位: kbps)，通常设置为1000
- `dbitrate`: 数据段波特率 (单位: kbps)，通常设置为2000
- `nsamplepos`: 仲裁段采样点位置 (百分比)，通常设置为75%
- `dsamplepos`: 数据段采样点位置 (百分比)，通常设置为75%
- `enable`: 是否启用该总线接口 (预留字段，当前暂未使用)

### 设备配置 (canbusX_devices)

每个设备包含以下字段：

- `name`: 设备名称，用于ROS系统中的关节命名
- `class`: 设备类型，支持以下类型：
  - `motor`: 电机设备
  - `revo2_hand`: Revo2灵巧手设备
  - `lejuclaw`: 自研夹爪设备
- `device_id`: 设备ID，（设备类型+设备ID=唯一标识）
- `params`: 设备参数数组，对于电机包含7个参数：
  - `[vel, kp_pos, kd_pos, tor, kp_vel, kd_vel, ki_vel]`
  - vel: 速度限制
  - kp_pos: 位置比例增益
  - kd_pos: 位置微分增益
  - tor: 力矩限制
  - kp_vel: 速度比例增益
  - kd_vel: 速度微分增益
  - ki_vel: 速度积分增益
- `ratio`: 减速比，用于将电机角度转换为关节角度
- `negative`: 电机方向是否反向 (true/false)
- `ignore`: 是否忽略/禁用该设备 (true/false)

**重要说明:**
- 如需禁用设备，请设置 `ignore: true`，不要修改 device_id 为 0x00
- 如需完全移除设备，请删除或注释该设备配置


## 故障排查

### 配置文件检查

#### 1. 检查配置文件是否存在
```bash
# 检查接线类型配置文件
ls -la ~/.config/lejuconfig/CanbusWiringType.ini

# 检查灵巧手协议类型配置文件
ls -la ~/.config/lejuconfig/HandProtocolType.ini

# 检查设备配置文件
ls -la ~/.config/lejuconfig/canbus_device_cofig.yaml
```

**期望结果:**
- 双总线配置: `CanbusWiringType.ini` 文件存在且内容为 `dual_bus`
- 灵巧手 can 协议配置: `HandProtocolType.ini` 文件存在且内容为 `proto_can`
- `canbus_device_cofig.yaml` 文件存在且格式正确

#### 2. 检查配置文件格式
```bash
# 检查YAML格式是否正确
python3 -c "import yaml; yaml.safe_load(open('~/.config/lejuconfig/canbus_device_cofig.yaml'))"
```

**常见格式错误:**
- **缩进问题**: YAML使用空格缩进，不能使用Tab
- **冒号后缺少空格**: `key:value` 应为 `key: value`
- **引号不匹配**: 字符串值需要正确配对引号
- **列表格式错误**: 设备列表需要使用 `-` 开头

#### 3. 重新生成配置文件
如果配置文件损坏或格式错误，可以重新执行配置脚本：
```bash
./tools/check_tool/canbus_config.sh
```

### 设备通信问题

#### 4. 检查设备是否被忽略
```bash
# 检查被忽略的设备
grep -n "ignore: true" ~/.config/lejuconfig/canbus_device_cofig.yaml
```

**问题排查:**
- 如果设备不使能但没有响应，检查 `ignore` 字段是否为 `true`
- 设备被忽略时会返回默认数据0，不会产生错误信息

### 常见错误及解决方案

| 错误现象 | 可能原因 | 解决方案 |
|---------|---------|---------|
| 配置脚本执行失败 | 权限不足 | 使用 `sudo` 执行或检查脚本权限 |
| 设备无响应 | device_id错误 | 检查设备ID是否与实际硬件一致 |
| 部分设备不工作 | ignore: true | 将对应设备的 `ignore` 设为 `false` |
| CAN通信失败 | CAN模块类型错误 | 检查 `type` 字段与硬件是否匹配 |
| 配置文件读取失败 | YAML格式错误 | 检查缩进、冒号、引号等格式问题 |
| 双总线只识别到一个 | CanbusWiringType.ini缺失 | 确认文件存在且内容为 `dual_bus` |


