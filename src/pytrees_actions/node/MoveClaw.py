from time import sleep
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from .utils import filter_tree_path

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from kuavo_humanoid_sdk.kuavo.leju_claw import LejuClaw
from kuavo_msgs.msg import lejuClawCommand
from kuavo_humanoid_sdk.kuavo_strategy_v2.pick_place_box.strategy_slam_optimize import control_leju_claw
import time
import rospy
from .performance_monitor import performance_monitor

class MoveClaw(Behaviour):
    def __init__(self, name: str, label:str, namespace: str, params):
        super().__init__(name)
        self.claw = LejuClaw()
        self.sdk = RobotSDK()
        self.executed = False
        self.label = label.split('/', -1)[-1]

        self.params = params

        velocities = self.params.get("velocity", "90, 90")
        target_positions = self.params.get("target_positions", "0, 0")
        torques = self.params.get("torque", "1.0, 1.0")
        self.velocity = [float(vel) for vel in velocities.split(',')]
        self.torque = [float(tor) for tor in torques.split(',')]
        self.target_positions = [float(pos) for pos in target_positions.split(',')]

        blackboard_namespace = namespace
        self.local_blackboard = py_trees.blackboard.Client(name=name, namespace=blackboard_namespace)
        self.global_blackboard = self.attach_blackboard_client(name=name)

        self.success = False

    def pub_leju_claw_comand(self, pos, vel, effort):
        pub = rospy.Publisher('/leju_claw_command', lejuClawCommand, queue_size=10)
        msg = lejuClawCommand()
        msg.data.name = ['left_claw', 'right_claw']
        msg.data.position = pos
        msg.data.velocity = vel
        msg.data.effort = effort
        for i in range(5):
            pub.publish(msg)
            time.sleep(0.1)
        return True

    @performance_monitor(method_name="initialise")
    def initialise(self):
        self.logger.info(f"开始移动夹爪")

        try:
            # open
            # control_leju_claw(position=self.target_positions, velocity=self.velocity, effort=self.torque)
            self.pub_leju_claw_comand(pos=self.target_positions, vel=[90, 90], effort=[2, 2])
            # time.sleep(1)

            self.success = True

        except Exception as e:
            self.logger.error(f"移动夹爪失败: {e}")
            self.success = False

    @performance_monitor(method_name="update")
    def update(self):
        if self.success:
            return Status.SUCCESS
        else:
            return Status.FAILURE