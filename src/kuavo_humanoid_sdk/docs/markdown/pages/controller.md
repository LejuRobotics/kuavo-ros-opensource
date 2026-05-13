# 控制器管理

Kuavo Humanoid SDK 提供了完整的机器人控制器管理功能，允许用户查询、切换和管理机器人的各种控制器。

## 主要功能


* **控制器查询**：获取当前可用的控制器列表及当前控制器信息


* **控制器切换**：支持切换到指定控制器、下一个控制器或上一个控制器


* **特殊控制器**：支持切换到 VMP 控制器（武术动作）和舞蹈控制器

## 核心类

### KuavoRobotController


### _class_ kuavo_humanoid_sdk.kuavo.robot_controller.KuavoRobotController()
Bases: `object`

Kuavo 机器人控制器管理类，用于获取和查询机器人控制器信息。


#### get_controller_list()
获取控制器列表信息。


* **Returns**

    控制器列表信息，包含以下字段：


        * controller_names (list): 可用控制器名称列表


        * count (int): 控制器数量


        * current_index (int): 当前控制器索引


        * current_controller (str): 当前控制器名称


        * success (bool): 获取是否成功


        * message (str): 返回消息

    None: 如果服务调用失败则返回 None




* **Return type**

    ControllerListInfo



#### get_current_controller_name()
获取当前控制器名称。

返回内部定时查询存储的当前控制器名称。


* **Returns**

    当前控制器名称，如果尚未查询到则返回 None



* **Return type**

    Optional[str]



#### switch_controller(controller_name: str)
切换到指定的控制器。


* **Parameters**

    **controller_name** (*str*) – 要切换到的控制器名称



* **Returns**

    切换结果，包含：


        * success (bool): 切换是否成功


        * message (str): 返回消息




* **Return type**

    ControllerResult



#### switch_to_dance_controller(data: str = "")
切换舞蹈控制器。服务为 ``/humanoid_controller/switch_to_dance_controller``（``kuavo_msgs/SetString``）：

* ``data == ""``：配置中舞蹈列表的第一项
* ``"#0"``、``"#1"`` 等：按已加载舞蹈列表下标
* 否则：按在 ``rl_controllers.yaml`` 中注册的控制器名称

* **Returns**

    操作结果，包含：


        * success (bool): 操作是否成功


        * message (str): 返回消息




* **Return type**

    ControllerResult



#### switch_to_next_controller()
切换到下一个控制器。

按顺序切换到列表中的下一个控制器（循环切换）。


* **Returns**

    切换结果，包含：


        * success (bool): 切换是否成功


        * message (str): 返回消息


        * current_controller (str): 当前控制器名称


        * target_controller (str): 下一个控制器名称


        * current_index (int): 当前控制器索引


        * target_index (int): 下一个控制器索引




* **Return type**

    SwitchControllerResult



#### switch_to_previous_controller()
切换到上一个控制器。

按顺序切换到列表中的上一个控制器（循环切换）。


* **Returns**

    切换结果，包含：


        * success (bool): 切换是否成功


        * message (str): 返回消息


        * current_controller (str): 当前控制器名称


        * target_controller (str): 上一个控制器名称


        * current_index (int): 当前控制器索引


        * target_index (int): 上一个控制器索引




* **Return type**

    SwitchControllerResult



#### switch_to_vmp_controller()
切换到 VMP 控制器。

VMP (Variational Motion Primitive) 控制器是一种基于运动原语的强化学习控制器，
用于执行预定义的复杂动作轨迹（如武术动作、舞蹈等）。
它通过 VAE 编码器将参考运动轨迹编码为潜在空间表示，
结合实时观测数据，由策略网络生成关节控制命令。


* **Returns**

    操作结果，包含：


        * success (bool): 操作是否成功


        * message (str): 返回消息




* **Return type**

    ControllerResult


## 数据类

以下数据类用于控制器相关的数据传输：

### ControllerListInfo


### _class_ kuavo_humanoid_sdk.interfaces.Controller.ControllerListInfo(controller_names: list, count: int, current_index: int, current_controller: str, success: bool, message: str)
Bases: `object`

表示控制器列表信息的数据类


#### controller_names(_: lis_ )
可用控制器名称列表


#### count(_: in_ )
控制器数量


#### current_controller(_: st_ )
当前控制器名称


#### current_index(_: in_ )
当前控制器索引


#### message(_: st_ )
返回消息


#### success(_: boo_ )
获取是否成功

### ControllerResult


### _class_ kuavo_humanoid_sdk.interfaces.Controller.ControllerResult(success: bool, message: str)
Bases: `object`

表示控制器操作结果的数据类


#### message(_: st_ )
返回消息


#### success(_: boo_ )
操作是否成功

### SwitchControllerResult


### _class_ kuavo_humanoid_sdk.interfaces.Controller.SwitchControllerResult(success: bool, message: str, current_controller: str, target_controller: str, current_index: int, target_index: int)
Bases: `object`

表示控制器切换结果的数据类


#### current_controller(_: st_ )
当前控制器名称


#### current_index(_: in_ )
当前控制器索引


#### message(_: st_ )
返回消息


#### success(_: boo_ )
切换是否成功


#### target_controller(_: st_ )
目标控制器名称（下一个/上一个）


#### target_index(_: in_ )
目标控制器索引

## 使用示例

### 获取控制器列表

```python
from kuavo_humanoid_sdk.kuavo.robot_controller import KuavoRobotController

robot_controller = KuavoRobotController()
controller_info = robot_controller.get_controller_list()

if controller_info:
    print(f"控制器数量：{controller_info.count}")
    print(f"当前控制器：{controller_info.current_controller}")
    print("可用控制器:")
    for i, name in enumerate(controller_info.controller_names):
        marker = " <-- 当前" if i == controller_info.current_index else ""
        print(f"  [{i}] {name}{marker}")
```

### 切换控制器

```python
# 切换到指定控制器
result = robot_controller.switch_controller("mpc")
if result.success:
    print(f"切换成功：{result.message}")

# 切换到下一个控制器
result = robot_controller.switch_to_next_controller()
print(f"从 {result.current_controller} 切换到 {result.target_controller}")
```

### 切换到 VMP 控制器

```python
# VMP (Variational Motion Primitive) 控制器是一种基于运动原语的强化学习控制器
# 用于执行预定义的复杂动作轨迹（如武术动作、舞蹈等）
result = robot_controller.switch_to_vmp_controller()
if result.success:
    print("已切换到 VMP 控制器")
```

**NOTE**: VMP 控制器不适合长时间静止站立，在做完动作后应尽快切换到常规控制器（如 MPC 或 AMP）。

## 相关文档


* [数据类型](data_types.md#data-types) - 数据类型参考


