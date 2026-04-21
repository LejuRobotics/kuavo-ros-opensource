# 配置睿尔曼电机id

- 新的电机id都是0x03, 需要配置成左手是0x01, 右手是0x02

- 先只接一个电机的电源, 然后修改并运行`setID.py`脚本

- 例如, 设置当前的电机id为0x01, 修改setID.py中的newID变量为0x01, 运行setID.py脚本即可

- 设置完之后要重启电机才能生效
# 设置零点

- 将电机掰到零位

- 执行`setZero.py`脚本

# SDK说明

数据发送和接收超时时间为**1S**

使用方法：从**SimpleSDK.py**中导入并创建对象

```python
from SimpleSDK import WHJ30Tools
whj30 = WHJ30Tools()
```

## open_canbus()

作用：开启CAN总线通讯

参数：无

返回值：

- 成功返回True，失败返回False
- 发生异常返回异常信息

```python
whj30.open_canbus() # 开启CAN总线通讯
```

## close_canbus()

作用：关闭CAN总线通讯

参数：无

返回值：

- 成功返回True，失败返回False
- 发生异常返回异常信息

```python
whj30.close_canbus() # 关闭CAN总线通讯
```

## write_reg(dev_id, addr, dat, len）

作用：写寄存器

参数：

- dev_id：关节ID
- addr：寄存器地址
- dat：数据
- len：数据字节

返回值：

- 成功返回True，失败返回False
- 未收到设备应答包返回False
- 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
- 发生异常返回异常信息

```python
whj30.write_reg(0x01, 0x44, 0x1234, 2) # 向ID为0x01的关节，地址为0x44的寄存器写入2字节数据，0x1234
```

## read_reg(dev_id, addr, len）

作用：读寄存器

参数：

- dev_id：关节ID
- addr：寄存器地址
- len：寄存器个数

返回值：

- 成功返回寄存器数据列表，失败返回False
- 未收到设备应答包返回False
- 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
- 发生异常返回异常信息

```python
whj30.read_reg(0x01, 0x44, 1) # 读取ID为0x01，地址为0x44的寄存器数据，返回数据列表[0x34, 0x12]
```

## skip_iap_update(dev_id）

  作用：跳过IAP更新（**必要操作**）

  参数：

  - dev_id：关节ID

  返回值：

  - 成功返回True，失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
whj30.skip_iap_update(0x01) # ID为0x01的关节跳过IAP更新
```

## enable_joint(dev_id）

  作用：使能关节

  参数：

  - dev_id：关节ID

  返回值：

  - 成功返回True，失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
whj30.enable_joint(0x01) # 使能ID为0x01的关节
```

## disable_joint(dev_id）

  作用：失能关节

  参数：

  - dev_id：关节ID

  返回值：

  - 成功返回True，失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
whj30.disable_joint(0x01) # 失能ID为0x01的关节
```

## get_joint_state(dev_id）

  作用：获取关节状态

  参数：

  - dev_id：关节ID

  返回值：

  - 成功返回状态列表（错误代码，系统电压，系统温度，使能状态，抱闸状态，当前位置，当前电流），失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
whj30.get_joint_state(0x01) # 获取ID为0x01的关节状态，返回状态列表[0, 23.0, 38.0, 1, 0, 100.18430000000001, -380]
```

## clear_joint_error(dev_id）

  作用：清除关节错误

  参数：

  - dev_id：关节ID

  返回值：

  - 成功返回True，失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
whj30.clear_joint_error(0x01) # 清除ID为0x01的关节错误
```

## set_joint_id(dev_id, new_id）

  作用：设置关节ID

  参数：

  - dev_id：关节ID
  - new_id：需要修改的ID

  返回值：

  - 成功返回True，失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
whj30.set_joint_id(0x01, 0x02) # 将ID为0x01的关节的ID修改为0x02
```

## set_joint_zero_position(dev_id）

  作用：设置关节零点

  参数：

  - dev_id：关节ID

  返回值：

  - 成功返回True，失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
whj30.set_joint_id(0x01) # 将ID为0x01的关节当前位置设置为零点
```

## save_joint_param(dev_id）

  作用：保存关节参数

  参数：

  - dev_id：关节ID

  返回值：

  - 成功返回True，失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
whj30.save_joint_param(0x01) # 保存ID为0x01的关节参数
```

## set_joint_operate_mode(dev_id, operate_mode）

  作用：设置关节工作模式，目前仅实现位置模式功能

  参数：

  - dev_id：关节ID
  - operate_mode：工作模式，0x01（电流模式）、0x02（速度模式）、0x03（位置模式）

  返回值：

  - 成功返回True，失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
whj30.set_joint_operate_mode(0x01, 0x03) # ID为0x01的关节工作模式设置为位置模式
```

## set_joint_position(dev_id, target_position）

  作用：设置关节位置，关节单次运动范围为±10°以内

  参数：

  - dev_id：关节ID
  - target_position：目标位置，单位°

  返回值：

  - 成功返回True，失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
whj30.set_joint_position(0x01, 100) # ID为0x01的关节位置设置到100°
```

## set_joint_min_position(dev_id, min_position）

  作用：设置关节最小位置

  参数：

  - dev_id：关节ID
  - min_position：最小位置，单位°

  返回值：

  - 成功返回True，失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
whj30.set_joint_min_position(0x01, 0) # ID为0x01的关节最小位置设置为0°
```

## set_joint_max_position(dev_id, max_position）

  作用：设置关节最大位置

  参数：

  - dev_id：关节ID
  - min_position：最大位置，单位°

  返回值：

  - 成功返回True，失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
whj30.set_joint_max_position(0x01, 100) # ID为0x01的关节最大位置设置到360°
```

# 注意事项

**python-can-3.3.4：**包含霸码科技USB转CANFD模块Python开发所需内容，其基于python-can包修改，python-can-3.3.4/examples文件夹下有部分示例可作参考，关于python-can的更多信息可访问[官网](https://python-can.readthedocs.io/en/stable/index.html)

**WHJ30:**包含睿尔曼关节简单示例代码

运行脚本前需要设置**PYTHONPATH**和**LD_LIBRARY_PATH**两个环境变量，可参考**run.sh**中内容：

1. PYTHONPATH：指向/python-can-3.3.4
2. LD_LIBRARY_PATH：指向/python-can-3.3.4/can/interfaces/bmcan

请先插入USB转CANFD模块再以**root**权限运行脚本
