.. _examples:

************
使用示例
************
.. include:: components/tips_launch_kuavo.rst

获取机器人信息
==============
这个示例展示了如何获取机器人的基本信息。

.. literalinclude:: ../../examples/atomic_skills/robot_info_example.py
  :language: python

机器人运动控制
==============
这个示例展示了如何初始化 SDK 并控制机器人的运动，包括站立，行走，转向，下蹲等。

.. literalinclude:: ../../examples/atomic_skills/motion_example.py
  :language: python

末端执行器控制
==============

LejuClaw 夹爪
--------------
这个示例展示了如何控制夹爪，支持整体开合、单侧控制、目标位置设定等多种操作，通过 `claw.open()`、`claw.close()`、`claw.control_left()`、`claw.control_right()`、`claw.control()` 等接口实现。

示例代码展示了以下主要功能：

#. 基本开合控制：

   * ``claw.close()`` 完全闭合夹爪
   * ``claw.open()`` 完全打开夹爪 
   * 每个动作后使用 ``wait_for_finish()`` 等待动作完成

#. 单侧夹爪控制：

   * ``claw.control_left([50])`` 控制左侧夹爪到50度位置
   * ``claw.control_right([80])`` 控制右侧夹爪到80度位置
   * 可以分别精确控制左右两侧夹爪的位置

#. 双侧同步控制：

   * ``claw.control([20, 100])`` 同时控制左右夹爪到不同位置
   * 第一个参数控制左侧，第二个参数控制右侧
   * 位置范围为0-100度，0表示完全闭合，100表示完全打开

注意事项：

* 每次动作后建议调用 ``wait_for_finish()`` 等待完成，避免动作叠加
* 可以设置超时时间，如 ``wait_for_finish(timeout=2.0)``
* 连续发送指令时要注意等待前一个动作完成，否则可能被丢弃

.. literalinclude:: ../../examples/atomic_skills/lejuclaw_example.py
  :language: python

Qiangnao 灵巧手
----------------
这个示例展示了如何控制灵巧手/触觉灵巧手，支持单侧控制、双侧同步控制、预设手势调用等功能。示例代码展示了：
#. 单侧手指控制：

   * 左手指向"点赞"手势
   * 右手指向"666"手势

#. 预设手势调用：

   * 获取所有支持的手势名称
   * 调用预设手势（如"点赞"和"OK"手势）

.. literalinclude:: ../../examples/atomic_skills/dexhand_example.py
  :language: python

手臂运动控制
============
这个示例展示了如何控制机器人手臂运动，包括轨迹控制和目标姿态控制。

示例代码包含三个主要的控制函数：

#. control_arm_traj()：关节角度插值运动

   * 从初始位置q0开始，通过90步插值运动到目标位置q1
   * q1设置了右臂抬起50度的姿态
   * 每步之间间隔0.02秒，实现平滑过渡
   * 运动完成后恢复到初始位置
   * 最后重置手臂位置

#. control_arm_joint_trajectory()：关节轨迹控制

   * 定义了7个关键时间点的目标姿态
   * 机器人会自动完成关键帧之间的轨迹规划
   * 执行完毕后重置手臂位置

#. control_arm_end_effector_pose()：末端执行器位姿控制

   * 直接指定左右手末端在世界坐标系中的目标位置和姿态
   * 通过逆运动学自动计算所需的关节角度
   * 机器人自动规划轨迹到达目标位姿
   * 完成后重置手臂位置

.. literalinclude:: ../../examples/atomic_skills/ctrl_arm_example.py
  :language: python

正向和逆向运动学控制
====================
这个示例展示了如何使用正向运动学 (FK) 从关节角度计算末端执行器位置，以及如何使用逆向运动学 (IK) 计算实现所需末端执行器姿态所需的关节角度。

.. literalinclude:: ../../examples/atomic_skills/arm_ik_example.py
  :language: python

头部运动控制
============
这个示例展示了如何控制机器人头部运动，包括点头 (pitch) 和摇头 (yaw) 运动。

#. 头部上下点头运动：

   * 从0度开始，先向上抬到25度
   * 然后从25度向下转到-25度
   * 最后从-25度回到0度
   * 重复2个周期

#. 头部左右摇头运动：

   * 从0度开始，先向左转到60度
   * 然后从60度向右转到-60度
   * 最后从-60度回到0度
   * 重复2个周期

每次运动都以较小的角度增量(2度)变化，并设置了0.1秒的时间间隔，以实现平滑的运动效果。

.. literalinclude:: ../../examples/atomic_skills/ctrl_head_example.py
  :language: python

单步落足控制
============
这个示例展示了如何通过自定义落足点轨迹控制机器人运动。

#.  首先需要确保机器人在站立状态，才能切换到自定义落足点控制模式

#.  向前步进0.8米

#.  等待机器人回到站立状态

#.  再次步进0.2米并旋转90度

注意事项:

* 步进控制只能在站立模式下使用
* 每次步进后需要等待机器人回到站立状态 
* 可以设置超时时间等待状态转换

.. literalinclude:: ../../examples/atomic_skills/step_control_example.py
  :language: python

控制机器人在世界坐标系或基座坐标系中的位姿
===============================================

通过 `control_command_pose_world` 和 `control_command_pose` 函数分别控制机器人在世界坐标系和基座坐标系中的位置和姿态。示例中展示了：

#.  在世界坐标系中控制机器人前进1米并旋转90度

#.  在基座坐标系中控制机器人后退2米并旋转-90度

.. literalinclude:: ../../examples/atomic_skills/cmd_pose_example.py
  :language: python

视觉获取 AprilTag 数据
======================

**演示如何获取和处理机器人的视觉数据，特别是AprilTag标记的识别结果。**

通过 `KuavoRobotVision` 类获取相机识别到的AprilTag数据。示例代码展示了：

1. 获取不同坐标系下的AprilTag数据:

   * 相机坐标系
   * 基座坐标系 
   * 里程计坐标系

2. 获取特定标记的详细信息:

   * 标记ID
   * 标记大小
   * 位置和姿态信息

3. 通过ID查询特定标记的数据

.. literalinclude:: ../../examples/atomic_skills/vision_robot_example.py
  :language: python

音频功能
========
这个示例展示了如何使用音频功能，包括播放音频和语音合成。

支持播放指定音频文件、TTS语音合成、停止音乐等操作，通过 `KuavoRobotAudio` 的 `play_audio`、`text_to_speech`、`stop_music` 等接口实现。

.. literalinclude:: ../../examples/atomic_skills/audio_example.py
  :language: python


电机 Kp/Kd 参数调整
====================
这个示例展示了如何调整电机 Kp/Kd 参数，包括关节角度、关节速度和关节力矩。

**注意**: 该示例仅支持在实物且`youda`类型的驱动器电机上运行

.. literalinclude:: ../../examples/atomic_skills/motor_param_example.py
  :language: python


获取观测信息（控制指令）等
==========================

这个示例展示了如何获取机器人当前的观测信息，如手臂位置指令等。

通过 `KuavoRobotObservation` 实时获取并打印手臂的目标位置指令，适合用于调试和监控机器人状态。

.. literalinclude:: ../../examples/atomic_skills/observation_example.py
  :language: python