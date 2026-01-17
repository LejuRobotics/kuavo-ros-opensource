.. _examples:

************
Examples
************

.. warning::

    Before running any code examples, make sure to start the robot first by executing either:
    
    - For simulation: ``roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch`` (Example command)
    - For real robot: ``roslaunch humanoid_controllers load_kuavo_real.launch`` (Example command)

Robot Info
===========
Examples showing how to get basic robot information.

.. literalinclude:: ../../examples/atomic_skills/robot_info_example.py
  :language: python

Basic Robot Control
====================
A basic example showing how to initialize the SDK and control the robot's movement.

.. literalinclude:: ../../examples/atomic_skills/motion_example.py
  :language: python

End Effector Control
=====================

LejuClaw Gripper
----------------
Examples demonstrating how to control the LejuClaw gripper end effector, including position, velocity and torque control.

.. literalinclude:: ../../examples/atomic_skills/lejuclaw_example.py
  :language: python

QiangNao DexHand
----------------
Examples showing how to control the QiangNao DexHand, a dexterous robotic hand with multiple degrees of freedom for complex manipulation tasks.

.. literalinclude:: ../../examples/atomic_skills/dexhand_example.py
  :language: python

Arm Control
============
Examples showing arm trajectory control and target pose control.

.. literalinclude:: ../../examples/atomic_skills/ctrl_arm_example.py
  :language: python

Forward and Inverse Kinematics
==============================
Examples demonstrating how to use forward kinematics (FK) to compute end-effector positions from joint angles, and inverse kinematics (IK) to calculate joint angles needed to achieve desired end-effector poses.

.. literalinclude:: ../../examples/atomic_skills/arm_ik_example.py
  :language: python

Arm Control (Protected)
======================
Examples showing how to control the robot's arm movements with collision protection.

.. literalinclude:: ../../examples/atomic_skills/ctrl_arm_example_protected.py
  :language: python

Head Control
=============
Examples showing how to control the robot's head movements, including nodding (pitch) and shaking (yaw) motions.

.. literalinclude:: ../../examples/atomic_skills/ctrl_head_example.py
  :language: python

Step-by-Step Control
====================
Examples showing how to control the robot's movements step by step, including individual foot placement and trajectory control.

.. literalinclude:: ../../examples/atomic_skills/step_control_example.py
  :language: python