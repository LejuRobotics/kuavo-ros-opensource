.. _kuavo-strategy:
.. include:: components/tips_launch_kuavo.rst

********
策略模块
********
原子技能层级
====================================================
.. currentmodule:: kuavo_humanoid_sdk.kuavo_strategy

.. autoclass:: kuavo_humanoid_sdk.KuavoRobot
    :members:
    :undoc-members:
    :show-inheritance:
    :noindex:

.. autoclass:: KuavoRobotVision
    :members:
    :undoc-members:
    :show-inheritance:
    :noindex:
    
.. autoclass:: KuavoRobotState
    :members:
    :undoc-members:
    :show-inheritance:
    :noindex:


.. autoclass:: KuavoRobotTools
    :members:
    :undoc-members:
    :show-inheritance:
    :noindex:
    
基础策略接口
====================
.. currentmodule:: kuavo_humanoid_sdk.kuavo_strategy

.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy.kuavo_strategy.KuavoRobotStrategyBase
    :members:
    :undoc-members:
    :show-inheritance:

箱子抓取策略
====================
.. currentmodule:: kuavo_humanoid_sdk.kuavo_strategy.grasp_box

箱子信息数据结构
------------------------
.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy.BoxInfo
    :members:
    :undoc-members:
    :show-inheritance:
    :noindex:

箱子抓取策略类
------------------------
.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy.KuavoGraspBox
    :members:
    :undoc-members:
    :show-inheritance:

------------------------


搬箱子示例
============

以下是一个使用箱子抓取策略的基本示例:

gazebo 仿真运行
-----------------

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
    
    git clone https://gitee.com/leju-robot/kuavo-ros-opensource.git
    cd kuavo-ros-opensource
    git checkout dev
    catkin build humanoid_controllers kuavo_msgs gazebo_sim ar_control
    
运行
^^^^^^^^^^^^

.. warning::
    在运行之前, 需要确认机器人版本 ``ROBOT_VERSION=45`` ，否则会机器人末端控制会有问题

.. warning::
    在运行之前, 需要修改 ``src/demo/grab_box/cfg/kuavo_v45/bt_config.yaml`` 中的 ``safe_space`` 参数:

    .. code-block:: yaml
    
        safe_space: [2.0, -4.0, 1.2, -1.2] # [x_+, x_-, y_+, y_-]
    
    更改为:

    .. code-block:: yaml
    
        safe_space: [20.0, -20.0, 12, -12] # [x_+, x_-, y_+, y_-]
        
.. note::
    案例中箱子的 AprilTag ID 为 1, 货架桌子上的 AprilTag ID 为 0, 请根据你的实际场景修改示例代码中的 AprilTag ID

上位机仓库 kuavo_ros_application 需要运行相关功能包:

.. code-block:: bash

    source devel/setup.bash
    roslaunch kuavo_tf2_web_republisher start_websocket_server.launch

启动仿真环境:

.. code-block:: bash

    # 终端1: 启动gazebo场景
    source devel/setup.bash
    roslaunch humanoid_controllers load_kuavo_gazebo_manipulate.launch joystick_type:=bt2pro

    # 终端2: 启动ar_tag转换码操作和virtual操作
    source devel/setup.bash
    roslaunch ar_control robot_strategies.launch  

运行搬箱子示例:

.. code-block:: bash

    cd src/kuavo_humanoid_sdk/examples/strategies
    python3 grasp_box_example.py --sim

实物运行
--------------

准备
^^^^^^^^^^^^

上位机需要先修改 ``./src/ros_vision/detection_apriltag/apriltag_ros/config/tags.yaml`` 文件, 将 tag 的 size 尺寸修改为实际大小，比如 0.1 米

.. code-block:: yaml

    standalone_tags:
        [
            {id: 0, size: 0.1, name: 'tag_0'},
            {id: 1, size: 0.1, name: 'tag_1'},
            {id: 2, size: 0.1, name: 'tag_2'},
            {id: 3, size: 0.1, name: 'tag_3'},
            {id: 4, size: 0.1, name: 'tag_4'},
            {id: 5, size: 0.1, name: 'tag_5'},
            {id: 6, size: 0.1, name: 'tag_6'},
            {id: 7, size: 0.1, name: 'tag_7'},
            {id: 8, size: 0.1, name: 'tag_8'},
            {id: 9, size: 0.1, name: 'tag_9'}
        ]

**零点标定** !! 非常重要 !!

首先需要插工装标定腿部电机零点，和摆正手臂和头部电机标定零点

然后安装标定打印件到机器人上，执行更加精准的头部和手臂零点标定工具:

.. code-block:: bash

    sudo su
    ./scripts/joint_cali/One_button_start.sh

编译
^^^^^^^^^^^^

上位机需要编译相关功能包:

.. code-block:: bash

    git clone https://www.lejuhub.com/ros-application-team/kuavo_ros_application.git
    cd kuavo_ros_application
    git checkout dev
    catkin build apriltag_ros kuavo_camera kuavo_tf2_web_republisher

下位机需要编译相关功能包:

.. code-block:: bash

    git clone https://gitee.com/leju-robot/kuavo-ros-opensource.git
    cd kuavo-ros-opensource
    git checkout dev
    catkin build humanoid_controllers grab_box ar_control

运行
^^^^^^^^^^^^

上位机运行
^^^^^^^^^^^^

.. code-block:: bash

    cd ~/kuavo_ros_application
    source devel/setup.bash
    roslaunch dynamic_biped apriltag.launch
    roslaunch kuavo_tf2_web_republisher start_websocket_server.launch

下位机运行
^^^^^^^^^^^^
.. warning::
    在运行之前, 需要修改 ``src/demo/grab_box/cfg/kuavo_v45/bt_config.yaml`` 中的 ``safe_space`` 参数:

    .. code-block:: yaml
    
        safe_space: [2.0, -4.0, 1.2, -1.2] # [x_+, x_-, y_+, y_-]
    
    更改为:

    .. code-block:: yaml
    
        safe_space: [20.0, -20.0, 12, -12] # [x_+, x_-, y_+, y_-]

.. note::
    案例中箱子的 AprilTag ID 为 1, 货架桌子上的 AprilTag ID 为 0, 请根据你的实际场景修改示例代码中的 AprilTag ID

.. note::
    如果在搬运过程中单步转身或搬起箱子时机器人前倾或前倾摔倒，则需要在工装标定的零点基础上，修改 ``~/.config/lejuconfig/offset.csv`` 中3/9号分别减去2度（视倾斜程度而定）

    如果在搬运过程中单步转身或搬起箱子时机器人后仰或后仰摔倒，则需要在工装标定的零点基础上，修改 ``~/.config/lejuconfig/offset.csv`` 中3/9号分别加上一点度数（视倾斜程度而定）

.. code-block:: bash

    source devel/setup.bash
    roslaunch humanoid_controllers load_kuavo_real.launch joystick_type:=bt2pro

    source devel/setup.bash
    roslaunch ar_control robot_strategies.launch real:=true

运行搬箱子示例:

.. code-block:: bash

    cd src/kuavo_humanoid_sdk/examples/strategies
    python3 grasp_box_example.py

示例代码
--------------
.. literalinclude:: ../../examples/strategies/grasp_box_example.py
  :language: python