.. _quickstart:

********
快速开始
********
.. include:: components/tips_launch_kuavo.rst

入门指南
========

在使用 SDK 之前，您必须首先通过调用 ``KuavoSDK().Init()`` 来初始化它。这是使用任何其他 SDK 功能之前的必要步骤。

以下是一个最小示例：

.. code:: python3

    from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot

    # 初始化 SDK - 这是必需的！
    if not KuavoSDK().Init():
        print("Init KuavoSDK failed, exit!")
        exit(1)

    # 成功初始化后创建机器人实例
    robot = KuavoRobot()

    # Now you can use the robot object...

.. warning::

    在没有首先调用 ``KuavoSDK().Init()`` 或当它返回 ``False`` 时调用 SDK 函数是非法的，可能导致未定义的行为。后果是不可预测的，可能很危险。

基础示例
--------
让我们分解这个示例：

1. 首先，我们导入所需的模块：

   - ``KuavoSDK`` - 主要的 SDK 接口
   - ``KuavoRobot`` - 机器人控制接口
   - ``time`` - 用于时间控制

2. 在 ``main()`` 函数中：

   - 我们使用 ``KuavoSDK().Init()`` 初始化 SDK - 这是关键且必须的第一步
   - 使用 ``KuavoRobot()`` 创建机器人实例

3. 基本机器人控制方法：

   - ``arm_reset()`` - 将手臂归位
   - ``stance()`` - 切换到站立状态模式
   - ``trot()`` - 切换到小跑步态模式

4. 行走控制：

   - 我们让机器人以 0.3 m/s 的速度行走 4 秒

这个示例演示了初始化 SDK 和控制机器人运动的基本工作流程：

.. literalinclude:: ../../examples/quickstart.py
  :language: python

以下是一个完整的示例，展示了初始化后获取机器人 apriltag 视觉数据：

.. literalinclude:: ../../examples/atomic_skills/vision_robot_example.py
  :language: python
