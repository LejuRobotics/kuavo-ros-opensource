# 前提

- 确保电机已经连接到CAN总线，并确保CAN总线通讯正常。
- 确保电机的驱动中设置的控制模式和sdk中的控制模式一致，并确保电机id已经正确设置。
- 确保重新上电后的电机的零位已经设置正确

# 配置yaml文件

- 先根据电机数量修改config.yaml文件，在config文件夹中有对应电机数量的的参考参数文件，如config8.yaml、config12.yaml和config14.yaml等，可供参考。config.yaml文件中可能需要修改的部分：电机id、控制模式、需要反转电机的id和小臂电机的id，其中，控制模式为**servo**表示伺服模式，**ptm**表示力矩模式，未使用的电机id需要设置为0x00，需要反转电机的id可以使用电机辨识脚本“Negtive.sh”来自动添加，小臂电机的id则需要手动修改。新出厂的机器人以及更换电机的机器人需要执行电机辨识环节，并根据电机的控制模式修改参数。

- 然后复制config.yaml到~/.config/lejuconfig，可执行以下命令：“cp ~/“kuavo所在的目录“/lib/ruiwo_controller/config.yaml ~/.config/lejuconfig”来实现

# 电机扫描和零位设置

- 电机扫描：在~/kuavo/lib/ruiwo_controller/目录下执行：sudo ./Scan.sh

- 零位：在~/kuavo/lib/ruiwo_controller/目录下执行：sudo ./Setzero.sh

# 电机辨识

- 在~/kuavo/lib/ruiwo_controller/目录下执行sudo ./Negtive.sh脚本，在终端中修改的"DEV_ID"参数，匹配真实的电机ID，ID号采用十进制数，该脚本会让对应ID的电机朝电机默认的正方向运行，若发现电机旋转与规定的正方向不符合，在终端出现“Do you want to add 'DEV_ID' to negtive_address in config.yaml? (yes/no):”提示时输入yes，会自动将该ID添加到config.yaml中，若不想添加，则输入no，并且根据终端提示选择删除"del"或保留"save",脚本会自动执行删除或保存操作。  
# 启用手臂电机多圈编码器  
- 在yaml文件里面将`Multi-turn_Encoder_mode: False`改为`True`，开启多圈编码器功能。  
- **设置手臂多圈零点位置：** 将手臂摆放到你认为的零点位置，执行设置多圈编码器零点脚本：在~/kuavo/lib/ruiwo_controller/目录下执行：sudo ./setMultiturnEncoderZero.sh    
- 设置多圈编码器零点位置后（只用设置一次），每次断电零点都会保存，意味着手臂零点固定。如果手臂电机编码器意外断电，需要重新设置多圈编码器零点  
- 如果想要**改变手臂零点位置**，只需再将手臂摆放到你想的位置，重新在~/kuavo/lib/ruiwo_controller/目录下执行：sudo ./setMultiturnEncoderZero.sh
- 多圈编码器供电是从机器人电池供电的，当机器人开机的时候会给编码器电池充电。当机器人电池过放掉电之后，多圈编码器自身的电池最长可以待机一个月左右。超过这个时间，机器人重新开机后，需要重新标定手臂多圈编码器零点  
- 头部电机如果不是多圈版本需要每次上电将头部单独**摆正**并执行`sudo ./Setzero.sh`脚本，如果是多圈版本，所有锐沃电机均不用setzero

# 测试方式

确认id正确后，在~/kuavo/bulid/ 目录下执行sudo ./lib/ruiwo_controller/ruiwo_test 

若成功测试，会有以下类似输出

# SDK说明

数据发送和接收超时时间为**1S**

使用方法：从**SimpleSDK.py**中导入并创建对象


注意事项：

- 控制函数必须与电机当前运行模式相匹配，电机默认设置为**伺服模式**

- 位置默认范围：-12.5rad ~ +12.5rad
- 速度默认范围：-10rad/s ~ +10rad/s
- 力矩默认范围：-50Nm ~ + 50Nm
- 数据发送和接收超时时间为**1S**

使用方法：从**SimpleSDK.py**中导入并创建对象

```python
from SimpleSDK import RUIWOTools
ruiwo = RUIWOTools()
```

## open_canbus()

作用：开启CAN总线通讯

参数：无

返回值：

- 成功返回True，失败返回False
- 发生异常返回异常信息

```python
ruiwo.open_canbus() # 开启CAN总线通讯
```

## close_canbus()

作用：关闭CAN总线通讯

参数：无

返回值：

- 成功返回True，失败返回False
- 发生异常返回异常信息

```python
ruiwo.close_canbus() # 关闭CAN总线通讯
```

## enter_motor_state(dev_id）

  作用：使能电机

  参数：

  - dev_id：电机ID

  返回值：

  - 成功返回电机信息（电机ID, 电机位置，电机速度，电机力矩，驱动板温度，故障码），失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
ruiwo.enter_motor_state(0x01) # 使能ID为0x01的电机
```

## enter_reset_state(dev_id）

  作用：失能电机

  参数：

  - dev_id：电机ID

  返回值：

  - 成功返回电机信息（电机ID, 电机位置，电机速度，电机力矩，驱动板温度，故障码），失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
ruiwo.enter_reset_state(0x01) # 失能ID为0x01的电机
```

## set_zero_positon(dev_id）

  作用：设置电机零点

  参数：

  - dev_id：电机ID

  返回值：

  - 成功返回电机信息（电机ID, 电机位置，电机速度，电机力矩，驱动板温度，故障码），失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
ruiwo.set_zero_positon(0x01) # 将ID为0x01的电机当前位置设置为零点
```

## run_servo_mode(dev_id, pos, vel, pos_kp, pos_kd, vel_kp, vel_kd, vel_ki)

作用：电机伺服模式运行

参数：

- dev_id：电机ID
- pos：期望位置
- vel：期望速度
- pos_kp：位置环kp
- pos_kd：位置环kd
- vel_kp：速度环kp
- vel_kd：速度环kd
- vel_ki：速度环ki

  返回值：

  - 成功返回电机信息（电机ID, 电机位置，电机速度，电机力矩，驱动板温度，故障码），失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
ruiwo.run_servo_mode(0x01, 10, 1, 3, 2, 5, 0, 0.001)
```

## run_ptm_mode(dev_id, pos, vel, pos_kp, pos_kd, torque)

作用：电机力位混合模式运行

参数：

- dev_id：电机ID
- pos：期望位置
- vel：期望速度
- pos_kp：位置环kp
- pos_kd：位置环kd
- torque：期望力矩

  返回值：

  - 成功返回电机信息（电机ID, 电机位置，电机速度，电机力矩，驱动板温度，故障码），失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
ruiwo.run_ptm_mode(0x01, 10, 1, 3, 2, 1)
```

## run_vel_mode(dev_id, vel, vel_kp, vel_kd, vel_ki)

作用：电机速度模式运行

参数：

- dev_id：电机ID
- vel：期望速度
- vel_kp：位置环kp
- vel_kd：位置环kd
- vel_ki：位置环ki

  返回值：

  - 成功返回电机信息（电机ID, 电机位置，电机速度，电机力矩，驱动板温度，故障码），失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
ruiwo.run_vel_mode(0x01, 1, 5, 0, 0.001)
```

## run_torque_mode(dev_id, torque)

作用：电机力矩模式运行

参数：

- dev_id：电机ID
- torque：期望力矩

  返回值：

  - 成功返回电机信息（电机ID, 电机位置，电机速度，电机力矩，驱动板温度，故障码），失败返回False
  - 未收到设备应答包返回False
  - 等待设备应答包期间收到其他设备CAN数据，则返回接收的CAN数据
  - 发生异常返回异常信息

```python
ruiwo.run_torque_mode(0x01, 1)
```

