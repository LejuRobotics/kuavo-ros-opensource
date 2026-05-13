.. _controller:

####################################
控制器管理
####################################

.. module:: kuavo_humanoid_sdk.kuavo.robot_controller

Kuavo Humanoid SDK 提供了完整的机器人控制器管理功能，允许用户查询、切换和管理机器人的各种控制器。

主要功能
========

- **控制器查询**：获取当前可用的控制器列表及当前控制器信息
- **控制器切换**：支持切换到指定控制器、下一个控制器或上一个控制器
- **特殊控制器**：支持切换到 VMP 控制器（武术动作）和舞蹈控制器

核心类
======

KuavoRobotController
--------------------

.. autoclass:: KuavoRobotController
    :members:
    :undoc-members:
    :show-inheritance:

数据类
======

以下数据类用于控制器相关的数据传输：

ControllerListInfo
------------------

.. autoclass:: kuavo_humanoid_sdk.interfaces.Controller.ControllerListInfo
    :members:
    :undoc-members:
    :show-inheritance:

ControllerResult
----------------

.. autoclass:: kuavo_humanoid_sdk.interfaces.Controller.ControllerResult
    :members:
    :undoc-members:
    :show-inheritance:

SwitchControllerResult
----------------------

.. autoclass:: kuavo_humanoid_sdk.interfaces.Controller.SwitchControllerResult
    :members:
    :undoc-members:
    :show-inheritance:

使用示例
========

获取控制器列表
--------------

.. code-block:: python

    from kuavo_humanoid_sdk.kuavo.robot_controller import KuavoRobotController

    robot_controller = KuavoRobotController()
    controller_info = robot_controller.get_controller_list()

    if controller_info:
        print(f"控制器数量：{controller_info.count}")
        print(f"当前控制器：{controller_info.current_controller}")
        print("可用控制器:")
        for name in controller_info.controller_names:
            marker = " <-- 当前" if name == controller_info.current_controller else ""
            print(f"  - {name}{marker}")

切换控制器
----------

.. code-block:: python

    # 切换到指定控制器
    result = robot_controller.switch_controller("mpc")
    if result.success:
        print(f"切换成功：{result.message}")

    # 切换到下一个控制器
    result = robot_controller.switch_to_next_controller()
    print(f"从 {result.current_controller} 切换到 {result.target_controller}")

切换到 VMP 控制器
-----------------

.. code-block:: python

    # VMP (Variational Motion Primitive) 控制器是一种基于运动原语的强化学习控制器
    # 用于执行预定义的复杂动作轨迹（如武术动作、舞蹈等）
    result = robot_controller.switch_to_vmp_controller()
    if result.success:
        print("已切换到 VMP 控制器")

.. note::
    VMP 控制器不适合长时间静止站立，在做完动作后应尽快切换到常规控制器（如 MPC 或 AMP）。

相关文档
========

- :ref:`data_types` - 数据类型参考
