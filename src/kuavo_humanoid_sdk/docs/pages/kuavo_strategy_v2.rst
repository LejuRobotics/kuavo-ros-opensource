.. _kuavo-strategy-v2:

***************
策略模块 v2
***************

概述
==========================
.. currentmodule:: kuavo_humanoid_sdk.kuavo_strategy_v2

策略模块 v2 调用 kuavo_humanoid_sdk 的sdk接口实现一些基本的任务，提供了更模块化和事件驱动的策略编写框架。
在不同层级上，这些任务可以被复用。


为什么有了sdk还需要策略模块？
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
例如简单的走路到一个指定点这件事情，可以调用kuavo_humanoid_sdk里发送/cmd_pos_world话题的接口实现，但是如果给定一个tag的位置，并要走到tag相对的一个位置，则需要对输入做坐标转换。并且，在走的过程中，我希望监听，走路是否成功到达目标，还是因为某种原因失败了。
事件就是这样一个封装：每个事件是一个类，可以在接收目标的时候做一些处理（如坐标转换，和范围检测），并且在执行过程中维护自己的状态，等等。

案例 >> 策略 >> 事件 >> SDK。它们之间的关系如下图：

.. image:: structure.jpg
   :alt: 策略模块 v2 结构图
   :width: 400px
   :align: center

数据结构
====================

机器人SDK
------------------------
.. currentmodule:: kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk

.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk.RobotSDK
    :members:
    :undoc-members:
    :show-inheritance:

数据类型
------------------------
.. currentmodule:: kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type

.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type.Frame
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type.Point
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type.Pose
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type.Tag
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type.Transform3D
    :members:
    :undoc-members:
    :show-inheritance:

事件系统
====================

基础事件
------------------------
.. currentmodule:: kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event

.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event.EventStatus
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event.BaseEvent
    :members:
    :undoc-members:
    :show-inheritance:

移动操作事件
------------------------
.. currentmodule:: kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate

.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate.head_events.EventPercep
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate.head_events.EventHeadMoveKeyPoint
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate.walk_events.EventWalkToPose
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate.arm_events.EventArmMoveKeyPoint
    :members:
    :undoc-members:
    :show-inheritance:

工具类
====================

日志工具
------------------------
.. currentmodule:: kuavo_humanoid_sdk.kuavo_strategy_v2.utils

.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy_v2.utils.logger_setup.Logger
    :members:
    :undoc-members:
    :show-inheritance:

策略函数
====================

箱子抓取策略
------------------------
.. currentmodule:: kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.strategy

.. autofunction:: kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.strategy.search_tag_with_head

.. autofunction:: kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.strategy.walk_approach_target_with_perception_loop

.. autofunction:: kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.strategy.move_arm_and_backward

.. autofunction:: kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.strategy.grab_box_and_backward

.. autofunction:: kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.strategy.place_box_and_backward

.. autofunction:: kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.strategy.return_to_idle

配置模块
====================

仿真配置
------------------------
.. currentmodule:: kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.configs.config_sim

.. automodule:: kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.configs.config_sim
    :members:
    :undoc-members:
    :show-inheritance:

实物配置
------------------------
.. currentmodule:: kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.configs.config_real

.. automodule:: kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.configs.config_real
    :members:
    :undoc-members:
    :show-inheritance:


搬箱子示例
============

注意ROS里Tag识别的坐标系如下：

.. image:: tag_frame.png
   :alt: Tag的坐标系定义
   :width: 400px
   :align: center

以下是一个使用箱子抓取策略的基本示例:

gazebo 仿真运行
------------------------

准备
^^^^^^^^^^^^

第一次启动 gazebo 场景前需要修改tag尺寸:

在 ``/opt/ros/noetic/share/apriltag_ros/config/tags.yaml`` 文件中将 tag 的 size 尺寸修改为和立方体 tag 码的尺寸一致（只需做一次）

.. code-block:: yaml

    standalone_tags:
      [
        {id: 0, size: 0.088, name: 'tag_0'},
        {id: 1, size: 0.088, name: 'tag_1'},
        {id: 2, size: 0.088, name: 'tag_2'},
        {id: 3, size: 0.088, name: 'tag_3'},
        {id: 4, size: 0.088, name: 'tag_4'},
        {id: 5, size: 0.088, name: 'tag_5'},
        {id: 6, size: 0.088, name: 'tag_6'},
        {id: 7, size: 0.088, name: 'tag_7'},
        {id: 8, size: 0.088, name: 'tag_8'},
        {id: 9, size: 0.088, name: 'tag_9'}
      ]

编译
^^^^^^^^^^^^

上位机需要编译相关功能包:

.. code-block:: bash

    git clone https://www.lejuhub.com/ros-application-team/kuavo_ros_application.git
    cd kuavo_ros_application
    git checkout dev
    catkin build kuavo_tf2_web_republisher

下位机首先需要编译相关功能包:

.. code-block:: bash

    git clone https://gitee.com/leju-robot/kuavo-ros-control.git
    cd kuavo-ros-control
    git checkout dev
    catkin build humanoid_controllers kuavo_msgs gazebo_sim ar_control


【或者】Mujoco 仿真运行
------------------------
.. code-block:: bash

    roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch

启动【事件】测试脚本
^^^^^^^^^^^^^^^^^^^^^^^^

行走事件测试
------------------------
.. code-block:: bash

    python3 kuavo_humanoid_sdk/kuavo_strategy_v2/pick_place_box/examples/walk_example.py


手臂移动事件测试
------------------------
.. code-block:: bash

    python3 kuavo_humanoid_sdk/kuavo_strategy_v2/pick_place_box/examples/arm_example.py

启动完整搬框案例
^^^^^^^^^^^^^^^^^^^^^^^^
.. code-block:: bash

    python3 kuavo_humanoid_sdk/kuavo_strategy_v2/pick_place_box/case.py

目录结构
============

.. toctree::
   :maxdepth: 2
   :caption: 目录

文件
----

- why_v2.md: 解释为什么使用 v2 版本。
- README.md: 项目的简要介绍。

目录
----

- **pick_place_box**: 包含与抓取和放置相关的策略。
- **resource**: 存储资源文件。
- **utils**: 实用工具和辅助函数。
- **common**: 通用模块和共享代码。
- **log**: 日志文件和记录。
