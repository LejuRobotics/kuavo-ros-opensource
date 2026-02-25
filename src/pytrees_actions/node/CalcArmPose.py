
from pickle import NONE
from time import sleep
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Frame
import numpy as np
from scipy.spatial.transform import Rotation as R
from py_trees.common import Status

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Tag, Pose, Frame, Transform3D
import py_trees
from py_trees.behaviour import Behaviour

from .utils import filter_tree_path
from .performance_monitor import performance_monitor

from typing import Dict, Tuple
class CalcArmPose(Behaviour):
    def __init__(self, name: str, label: str, namespace, params):
        super().__init__(name)

        self.label = label.split('/', -1)[-1]

        blackboard_namespace = namespace
        self.local_blackboard = py_trees.blackboard.Client(name=name, namespace=blackboard_namespace)
        self.global_blackboard = self.attach_blackboard_client(name=name)
        self.robot_sdk = RobotSDK()
        self.robot_type = None
        self.global_blackboard.register_key(key="Common_robot_type", access=py_trees.common.Access.READ)
        # 注册黑板键
        self.global_blackboard.register_key(key="AllTagInfoOfBase", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="ArmJointTrajectories", access=py_trees.common.Access.WRITE)
        self.global_blackboard.register_key(key="TargetTag", access=py_trees.common.Access.WRITE)
        self.global_blackboard.register_key(key="ArmPoseAndWrench", access=py_trees.common.Access.WRITE)

        self.global_blackboard.register_key(key="Common_BoxLength", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="Common_BoxWidth", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="Common_BoxHeight", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="GraspBox_ForceRatioZ", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="GraspBox_LateralForce", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="Common_BoxMass", access=py_trees.common.Access.READ)

        self.params = params
        self.mode = self.params.get('mode', 'smt_arm_move_to_head')

        self.initialise_success = False

    @performance_monitor(method_name="initialise")
    def initialise(self):
        if self.mode.startswith("smt_"):
            self._initialise_smt()
        elif self.mode.startswith("box_"):
            self._initialise_box()
        elif self.mode.startswith("depalletize_"):
            self._initialise_depalletize()
        elif self.mode.startswith("depservo_"):
            self._initialise_depservo()

    @performance_monitor(method_name="update")
    def update(self):
        """更新节点状态"""
        if not self.initialise_success:
            return Status.FAILURE

        if self.mode.startswith("smt_"):
            print("calc smt arm pose finished")
            return self._smt_update()

        if self.mode.startswith("box_"):
            return self._box_update()

        if self.mode.startswith("depalletize_"):
            return self._depalletize_update()

        if self.mode.startswith("depservo_"):
            return self._depservo_update()

        # 如果 mode 不匹配任何已知模式，返回失败
        self.logger.error(f"未知的 mode: {self.mode}")
        return Status.FAILURE

    def _initialise_box(self):
        """初始化box节点"""
        self.initialise_success = True
    def _initialise_depalletize(self):
        """初始化depalletize节点"""
        self.initialise_success = True
    def _initialise_depservo(self):
        """初始化depservo节点"""
        self.initialise_success = True
    def _initialise_smt(self):
        self.global_blackboard.register_key(key="TargetSMTIDRow", access=py_trees.common.Access.READ)
        self.global_blackboard.register_key(key="TargetSMTID", access=py_trees.common.Access.READ)
        """初始化smt节点"""
        # 从黑板读取 TargetSMTIDRow
        arm_move_traject_pose = {
            "arm_prepare_pose_2_row": {
                "debug_row_name": "第2排",
                "pre_target_poses": [1, [-106.85238229061974, 4.109115893988121, -13.72617749137773, -122.8884587280882, -47.167398360385825, -69.08980792799103, 28.785516002186327, 23.71018190496966, -9.15813954898618, 10.535026483425678, -46.79561867573792, -17.725952990917378, -3.2784274298137466, 16.414342537054377]]
            },
            "arm_prepare_pose_3_row": {
                "debug_row_name": "第3排",
                "pre_target_poses": [1, [2.8647889756541165, 8.05107028413583, -20.239425790387873, -155.280450419045, -70.81646143255071, -51.51684415072679, 30.162496568858863, 31.869767676671174, -11.387540287867827, 10.076100510932786, -60.21584540802573, -14.534893400895921, -3.344325573807411, 11.169118793863937]]
            },
            "arm_prepare_pose_4_row": {
                "debug_row_name": "第4排",
                "pre_target_poses": [1, [14.15574986741625, 1.267738855175653, -18.556561235675144, -126.53295149458229, -78.20707162846004, -24.333228525075874, 28.173621915903205, 20.006521077312904, -7.606215440794908, 10.05430073236978, -36.52280718574757, -14.556566552333793, -1.8796529686609185, 14.491094959056634]]
            },
            "arm_prepare_pose_5_row": {
                "debug_row_name": "第5排",
                "pre_target_poses": [1, [0.0, 5.857608060188173, -22.14115796403405, -112.95666608218802, -72.76180900622764, -37.15703043067502, 27.277647103455514, 24.3578262111569, -9.464123367119862, 10.841078143705733, -45.79020328141009, -13.376324898219677, -1.0053658144573046, 7.0816243110008426]]
            },
            "arm_prepare_pose_6_row": {
                "pre_target_poses": [1, [11.459155902616466, 11.459155902616466, 0.0, -56.10673260968163, -69.43961129751536, -5.442170892922082, 6.875493541569878, 28.231035444091514, -10.928499902315075, 11.562282146891853, -52.06315435669036, -12.414658521341295, -1.11476842245067, 7.6058437636619765]]
            },
            "arm_move_to_head": [
                [0.8, [-30, -40, -20, -90, 0, -15, 3, 2.5, -1.5, 1.3, -2, 0.17, 0.065, 0.37]]
            ],
            "arm_place_execute": [
                [0.5, [-55.948581, 11.847661, -59.109643, -64.087237, -19.720226, -8.202122, -20.115264, 27.047374,
                    -11.406113, 11.485359, -56.491676, -13.801708, -1.090193, 8.369241]],
                [0.8, [-58.980504, -12.534475, -67.546314, -17.813916, -59.497553, 19.461069, -24.524301, 25.500536,
                    -7.994606, 12.113954, -51.104790, -11.337213, -3.625663, 7.541371]],
                [1.1, [-56.032080, -10.262997, -63.050543, -15.809181, -50.157864, -7.583795, -27.906737, 24.703313,
                    -7.701698, 12.249448, -49.079786, -11.744624, -2.908911, 6.466917]],
                [1.3, [-56.143881, -10.302724, -62.923698, -16.181655, -49.867205, -9.029821, -27.759823, 24.589863,
                    -7.718482, 12.225287, -48.762838, -11.986387, -2.829542, 6.420256]]
            ],
            "arm_backward_after_place": [
                [0.2, [-39.276179, 17.997447, -62.841195, -76.585956, -47.677527, -17.101594, -13.604045, 28.374740,
                    -11.619686, 13.296413, -56.728886, -14.791013, -2.110495, 14.649980]],
                [0.4, [-33.783113, 30.951001, -62.474620, -93.021795, -52.939490, -6.111006, -18.186877, 25.932421,
                    -10.528018, 12.913447, -51.203134, -15.100647, -1.816185, 13.363724]],
                [0.6, [-13.952884, 36.701338, -54.991005, -99.395940, -47.932208, 6.417060, -9.210062, 23.318515,
                    -9.138297, 11.900506, -46.785559, -14.979767, -1.638199, 11.195697]],
                [0.8, [2.861067, 33.726738, -48.653137, -96.991292, -42.766185, 10.552176, -1.193767, 23.862625,
                    -9.142449, 12.187975, -46.781768, -14.938248, -1.710637, 10.138234]],
                [1.0, [10.573113, 4.605678, -20.807334, -20.349897, -9.777089, 12.313784, -9.395159, 10.291423,
                    -4.738348, 11.655110, -20.388633, -15.214493, -1.107452, 7.228860]]
            ]
        }

        target_row = self.global_blackboard.TargetSMTIDRow
        if target_row is None:
            self.logger.error("TargetSMTIDRow 未在黑板中找到！")
            self.initialise_success = False
            return

        # 映射 TargetSMTIDRow 到行名称
        row_mapping = {
            2: "Second",
            3: "Third",
            4: "Fourth",
            5: "Fifth",
            6: "Sixth"
        }
        row_name = row_mapping[target_row]
        if row_name is None:
            self.logger.error(f"无效的 TargetSMTIDRow 值: {target_row}")
            self.initialise_success = False
            return

        self.smt_arm_trajectories = {}
        # 动态加载 arm_prepare_pose
        try:
            arm_prepare_pose_key = f"arm_prepare_pose_{int(target_row)}_row"
            self.smt_arm_trajectories["smt_arm_prepare_pose"] = arm_move_traject_pose[arm_prepare_pose_key]["pre_target_poses"]
            self.smt_arm_trajectories["smt_arm_place_execute"] = arm_move_traject_pose["arm_place_execute"]
            self.smt_arm_trajectories["smt_arm_backward_after_place"] = arm_move_traject_pose["arm_backward_after_place"]
            self.smt_arm_trajectories["smt_move_to_head"] = arm_move_traject_pose["arm_move_to_head"]
            self.logger.info(f"成功加载 {row_name} 的 arm_prepare_pose")
        except KeyError as e:
            self.logger.error(f"从 board.json 加载 arm_prepare_pose 失败: {e}")
            self.initialise_success = False
            return

        # 动态加载 pick_pose_params
        try:
            self.smt_pick_pose_params = {
                "pre_pos": self._read_board_value(f"SMTPickPoseOffsets_{row_name}RowPrePos"),
                "pre_euler_deg": self._read_board_value(f"SMTPickPoseOffsets_{row_name}RowPreEulerDeg"),
                "grasp_pos": self._read_board_value(f"SMTPickPoseOffsets_{row_name}RowGraspPos"),
                "grasp_euler_deg": self._read_board_value(f"SMTPickPoseOffsets_{row_name}RowGraspEulerDeg")
            }
            self.smt_post_pose_params = {
                "withdraw_pos_offset": self._read_board_value(f"SMTPostGraspOffsets_{row_name}RowWithdrawPosOffset"),
                "withdraw_euler_deg": self._read_board_value(f"SMTPostGraspOffsets_{row_name}RowWithdrawEulerDeg"),
                "observe_pos_abs": self._read_board_value(f"SMTPostGraspOffsets_{row_name}RowObservePosAbs"),
                "observe_euler_deg": self._read_board_value(f"SMTPostGraspOffsets_{row_name}RowObserveEulerDeg")
            }
            self.logger.info(f"成功加载 {row_name} 的 pick_pose_params")
        except KeyError as e:
            self.logger.error(f"从 board.json 加载 pick_pose_params 失败: {e}")
            self.success = False
            return

        self.initialise_success = True

    def _depservo_update(self):
        """更新depservo节点"""
        if self.mode.startswith('depservo_'):
            try:
                height_compensation = 0.0
                pick_left_arm_poses = [
                    Pose.from_euler(pos=( - 0.3, -0.05, -0.02), euler=(0, -0, 90), degrees=True,
                                    frame=Frame.TAG),
                    Pose.from_euler(pos=(-0.17, -0.05, 0.01), euler=(-20, -15,90), degrees=True,
                                    frame=Frame.TAG),
                    Pose.from_euler(pos=(-0.17, 0.15, 0.01), euler=(-20, -15, 90), degrees=True,
                                    frame=Frame.TAG),
                    ]

                pick_right_arm_poses = [
                    # Pose.from_euler(pos=(0.2, -0.3, 0.3), euler=(0, -90, 0), degrees=True,
                    #                 frame=Frame.BASE),
                    # Pose.from_euler(pos=(0.2, -0.3, 0.3), euler=(0, -90, 0), degrees=True,
                    #                 frame=Frame.BASE),
                    # Pose.from_euler(pos=(0.2, -0.3, 0.3), euler=(0, -90, 0), degrees=True,
                    #                 frame=Frame.BASE),
                    # Pose.from_euler(pos=(0.2, -0.3, 0.3), euler=(0, -90, 0), degrees=True,
                    #                 frame=Frame.BASE),
                    ]

                pick_left_arm_poses1 = [
                    # Pose.from_euler(pos=(-0.37, 0.15, 0.01), euler=(-20, -15, 90), degrees=True,
                    #                 frame=Frame.TAG),
                    # Pose.from_euler(pos=(-0.37, 0.15, 0.01), euler=(-20, -15, 90), degrees=True,
                    #                 frame=Frame.TAG),
                    # Pose.from_euler(pos=(-0.37, 0.15, 0.01), euler=(-20, -15, 90), degrees=True,
                    #                 frame=Frame.TAG),
                    ]

                pick_right_arm_poses1 = [
                    Pose.from_euler(pos=(0.3, 0.05, 0.01), euler=(-20, -15, 90), degrees=True,
                                    frame=Frame.TAG),
                    Pose.from_euler(pos=(0.17, 0.05, 0.01), euler=(-20, -15, 90), degrees=True,
                                    frame=Frame.TAG),
                    Pose.from_euler(pos=(0.17, 0.15, 0.01), euler=(-20, -15, 90), degrees=True,
                                    frame=Frame.TAG),
                    ]

                pick_left_arm_poses2 = [
                    Pose.from_euler(pos=(0.40, 0.17, 0.2), euler=(0, -90, -5), degrees=True, frame=Frame.BASE)]

                pick_right_arm_poses2 = [
                    Pose.from_euler(pos=(0.40, -0.17, 0.2), euler=(0, -90, 5), degrees=True, frame=Frame.BASE)]


                # ================ 计算每个关键点的力控目标（wrench） ================ #
                # 计算夹持力参数
                box_mass = 0.5
                g = 9.8  # 重力加速度
                # 计算基础Z向力（考虑安全系数和经验比例）
                force_ratio_z = 1
                force_z = -abs(box_mass * g * force_ratio_z)
                lateral_force = 13
                # 判断是否为仿真模式
                left_force = lateral_force  # 左手侧向力（正值为夹紧方向）
                right_force = -lateral_force  # 右手侧向力（负值为夹紧方向）

                pick_left_arm_wrench = [
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                ]

                pick_right_arm_wrench = [
                ]

                pick_left_arm_wrench1 = [
                ]

                pick_right_arm_wrench1 = [
                    [0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0],
                ]

                pick_left_arm_wrench2 = [
                    [0, 0, 0, 0, 0, 0],
                ]

                pick_right_arm_wrench2 = [
                    [0, 0, 0, 0, 0, 0],
                ]

                arm_traj1 = (pick_left_arm_poses, pick_right_arm_poses)
                arm_traj2 = (pick_left_arm_poses1, pick_right_arm_poses1)
                arm_traj3 = (pick_left_arm_poses2, pick_right_arm_poses2)
                arm_wrench1 = (pick_left_arm_wrench, pick_right_arm_wrench)
                arm_wrench2 = (pick_left_arm_wrench1, pick_right_arm_wrench1)
                arm_wrench3 = (pick_left_arm_wrench2, pick_right_arm_wrench2)
                if self.mode == 'depservo_step1':
                    self.global_blackboard.ArmPoseAndWrench = [arm_traj1, arm_wrench1]
                elif self.mode == 'depservo_step2':
                    self.global_blackboard.ArmPoseAndWrench = [arm_traj2, arm_wrench2]
                elif self.mode == 'depservo_step3':
                    self.global_blackboard.ArmPoseAndWrench = [arm_traj3, arm_wrench3]
                else:
                    self.feedback_message = f"未知模式: {self.mode}"
                    return Status.FAILURE

                self.feedback_message = "calculate success."
                return Status.SUCCESS

            except Exception as e:
                self.feedback_message = f"抓取失败: {e}"
                self.success = False
                return Status.FAILURE

    def _depalletize_update(self):
        if self.mode.startswith('depalletize_'):
            print(f"self.mode = {self.mode}")
            self.robot_type = self.global_blackboard.Common_robot_type
            print(f"self.robot_type = {self.robot_type}")
            self.global_blackboard.register_key(key="TagLeftOrRight", access=py_trees.common.Access.READ)  # 可读写
            box_width = float(getattr(self.global_blackboard, "Common_BoxWidth"))
            box_length = float(getattr(self.global_blackboard, "Common_BoxLength"))
            box_mass = float(getattr(self.global_blackboard, "Common_BoxMass"))
            box_height = float(getattr(self.global_blackboard, "Common_BoxHeight"))
            force_ratio_z = float(getattr(self.global_blackboard, "GraspBox_ForceRatioZ"))
            lateral_force = float(getattr(self.global_blackboard, "GraspBox_LateralForce"))
            if self.robot_type == "kuavo_lb":
                box_width = 0.4
                box_left_tag = 0
                hand_pitch_degree = 45
                box_beneath_tag = -0.2
                box_behind_tag = 0.15
                # left arm pose when tag is on the left
                left_arm_roll, left_arm_pitch, left_arm_yaw  = -90, 0, 0              #0, 45, 90
                grasp_left_pose_left_tag=[
                    #左手预抓取点位
                    Pose.from_euler(pos=(0.2, 0.3, 0.3), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                                    frame=Frame.BASE),
                    Pose.from_euler(pos=(0.3, 0.2, 0.3), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                                    frame=Frame.BASE),
                    Pose.from_euler(pos=(0.4, 0.2, 0.3), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                                    frame=Frame.BASE),
                    #左手抓取
                    Pose.from_euler(pos=(-box_width / 2 - box_left_tag + 0.035, -box_beneath_tag + 0.2, -box_behind_tag),
                                euler=(0, hand_pitch_degree, 90), degrees=True, frame=Frame.TAG),
                    Pose.from_euler(pos=(-box_width / 2 - box_left_tag + 0.035, -box_beneath_tag + 0.1, -box_behind_tag),
                                euler=(0, hand_pitch_degree, 90), degrees=True, frame=Frame.TAG),
                    #左手拉出点位
                    Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag + 0.24, -box_behind_tag),
                                euler=(0, hand_pitch_degree, 90), degrees=True, frame=Frame.TAG),
                    Pose.from_euler(pos=(-box_width / 2 - box_left_tag - 0.1, -box_beneath_tag + 0.24, -box_behind_tag),
                                euler=(0, hand_pitch_degree, 90), degrees=True, frame=Frame.TAG),
                    #右手抓取点位
                    Pose.from_euler(pos=(-box_width / 2 - box_left_tag - 0.1, -box_beneath_tag + 0.2, -box_behind_tag),
                                euler=(0, hand_pitch_degree, 90), degrees=True, frame=Frame.TAG),
                    Pose.from_euler(pos=(-box_width / 2 - box_left_tag - 0.1, -box_beneath_tag + 0.2, -box_behind_tag),
                                euler=(0, hand_pitch_degree, 90), degrees=True, frame=Frame.TAG),
                    #抬升点位
                    Pose.from_euler(pos=(0.4, box_width / 2, 0.0), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                                    frame=Frame.BASE),
                    Pose.from_euler(pos=(0.4, box_width / 2, 0.0), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                                    frame=Frame.BASE),
                    Pose.from_euler(pos=(0.3, box_width / 2, 0.0), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                                    frame=Frame.BASE),
                ]

                # left arm pose when tag is on the right
                grasp_left_pose_right_tag=[
                    #左手预抓取点位
                    Pose.from_euler(pos=(-box_width / 2 - box_left_tag + 0.035, -box_beneath_tag + 0.2, -box_behind_tag),
                                euler=(0, hand_pitch_degree, 90), degrees=True, frame=Frame.TAG),
                    Pose.from_euler(pos=(-box_width / 2 - box_left_tag + 0.035, -box_beneath_tag + 0.1, -box_behind_tag),
                                euler=(0, hand_pitch_degree, 90), degrees=True, frame=Frame.TAG),

                    Pose.from_euler(pos=(-box_width / 2 - box_left_tag, -box_beneath_tag + 0.24, -box_behind_tag),
                                euler=(0, hand_pitch_degree, 90), degrees=True, frame=Frame.TAG),

                    Pose.from_euler(pos=(0.3, box_width / 2, 0.0), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                                    frame=Frame.BASE),
                ]

                right_arm_roll, right_arm_pitch, right_arm_yaw = 90, 0, 180  # 0, 45, 90
                # right arm pose when tag is on the left
                grasp_right_pose_left_tag=[
                    #右手预抓取点位
                    Pose.from_euler(pos=(0.2, -0.3, 0.3), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                                    frame=Frame.BASE),
                    Pose.from_euler(pos=(0.3, -0.2, 0.3), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                                    frame=Frame.BASE),
                    Pose.from_euler(pos=(0.4, -0.2, 0.3), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                                    frame=Frame.BASE),
                    #左手抓取
                    Pose.from_euler(pos=(0.4, -0.2, 0.3), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                                    frame=Frame.BASE),
                    Pose.from_euler(pos=(0.4, -0.2, 0.3), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                                    frame=Frame.BASE),
                    #左手拉出点位
                    Pose.from_euler(pos=(0.4, -0.2, 0.3), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                                    frame=Frame.BASE),
                    Pose.from_euler(pos=(0.4, -0.2, 0.3), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                                    frame=Frame.BASE),
                    #右手抓取点位
                    Pose.from_euler(pos=(box_width / 2 - box_left_tag - 0.16, -box_beneath_tag + 0.2, -box_behind_tag),
                                euler=(0, hand_pitch_degree, 90), degrees=True, frame=Frame.TAG),
                    Pose.from_euler(pos=(box_width / 2 - box_left_tag - 0.16, -box_beneath_tag + 0.06, -box_behind_tag),
                                euler=(0, hand_pitch_degree, 90), degrees=True, frame=Frame.TAG),
                    #抬升点位
                    Pose.from_euler(pos=(0.4, -box_width / 2, 0.0), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                                    frame=Frame.BASE),
                    Pose.from_euler(pos=(0.4, -box_width / 2, 0.0), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                                    frame=Frame.BASE),
                    Pose.from_euler(pos=(0.3, -box_width / 2, 0.0), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                                    frame=Frame.BASE),
                ]

                # right arm pose when tag is on the right
                grasp_right_pose_right_tag=[
                    #左手预抓取点位
                    Pose.from_euler(pos=(box_width / 2 - box_left_tag + 0.035, -box_beneath_tag + 0.2, -box_behind_tag),
                                euler=(0, hand_pitch_degree, 90), degrees=True, frame=Frame.TAG),
                    Pose.from_euler(pos=(box_width / 2 - box_left_tag + 0.035, -box_beneath_tag + 0.1, -box_behind_tag),
                                euler=(0, hand_pitch_degree, 90), degrees=True, frame=Frame.TAG),

                    Pose.from_euler(pos=(box_width / 2 - box_left_tag, -box_beneath_tag + 0.24, -box_behind_tag),
                                euler=(0, hand_pitch_degree, 90), degrees=True, frame=Frame.TAG),

                    Pose.from_euler(pos=(0.3, -box_width / 2, 0.0), euler=(0, -90 + hand_pitch_degree, 0), degrees=True,
                                    frame=Frame.BASE),
                ]


                # ================ 计算每个关键点的力控目标（wrench） ================ #
                # 计算夹持力参数
                g = 9.8  # 重力加速度

                # 计算基础Z向力（考虑安全系数和经验比例）
                force_z = -abs(box_mass * g * force_ratio_z)

                # 判断是否为仿真模式
                left_force = lateral_force  # 左手侧向力（正值为夹紧方向）
                right_force = -lateral_force  # 右手侧向力（负值为夹紧方向）

                grasp_left_arm_wrench = [
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                ]

                grasp_right_arm_wrench = [
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                ]

                tag_left_or_right = self.global_blackboard.TagLeftOrRight
                print(f"tag is on the {tag_left_or_right}")
                # tag_left_or_right = 'left'

                if tag_left_or_right == 'left':
                    arm_traj = (grasp_left_pose_left_tag, grasp_right_pose_left_tag)
                    arm_wrench = (grasp_left_arm_wrench, grasp_right_arm_wrench)
                    step1_param = [(arm_traj[0][:3], arm_traj[1][:3]), (arm_wrench[0][:3], arm_wrench[1][:3])]
                    step2_param = [(arm_traj[0][3:5], arm_traj[1][3:5]), (arm_wrench[0][3:5], arm_wrench[1][3:5])]
                    step3_param = [(arm_traj[0][5:7], arm_traj[1][5:7]), (arm_wrench[0][5:7], arm_wrench[1][5:7])]
                    step4_param = [(arm_traj[0][7:9], arm_traj[1][7:9]), (arm_wrench[0][7:9], arm_wrench[1][7:9])]
                    step5_param = [(arm_traj[0][9:], arm_traj[1][9:]), (arm_wrench[0][9:], arm_wrench[1][9:])]
                else:
                    arm_traj = (grasp_left_pose_right_tag, grasp_right_pose_right_tag)
                    wrench = [
                        [0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0]
                    ]
                    arm_wrench = (wrench, wrench)
                    step1_param = [(arm_traj[0][:2], arm_traj[1][:2]), (arm_wrench[0][:2], arm_wrench[1][:2])]
                    step2_param = [(arm_traj[0][1:2], arm_traj[1][1:2]), (arm_wrench[0][1:2], arm_wrench[1][1:2])]
                    step3_param = [(arm_traj[0][2:], arm_traj[1][2:]), (arm_wrench[0][2:], arm_wrench[1][2:])]

                if self.mode == 'depalletize_grasp_step1':
                    self.global_blackboard.ArmPoseAndWrench = step1_param
                    self.success = True
                elif self.mode == 'depalletize_grasp_step2':
                    self.global_blackboard.ArmPoseAndWrench = step2_param
                    self.success = True
                elif self.mode == 'depalletize_grasp_step3':
                    self.global_blackboard.ArmPoseAndWrench = step3_param
                    self.success = True
                elif self.mode == 'depalletize_grasp_step4':
                    self.global_blackboard.ArmPoseAndWrench = step4_param
                    self.success = True
                elif self.mode == 'depalletize_grasp_step5':
                    self.global_blackboard.ArmPoseAndWrench = step5_param
                    self.success = True
                elif self.mode == 'depalletize_place':
                    place_right_pose = [ Pose.from_euler(pos=(0.42, -box_length / 2, 0.20), euler=(0, -60, 0), degrees=True,
                                    frame=Frame.BASE)]
                    place_left_pose = [ Pose.from_euler(pos=(0.42, box_length / 2, 0.20), euler=(0, -60, 0), degrees=True,
                                    frame=Frame.BASE)]
                    place_left_wrench = [
                        [0, 0, 0, 0, 0, 0]
                    ]
                    place_right_wrench = [
                        [0, 0, 0, 0, 0, 0]
                    ]
                    arm_traj = (place_left_pose, place_right_pose)
                    place_arm_wrench = (place_left_wrench, place_right_wrench)
                    arm_pose_and_wrench = [(arm_traj[0][:], arm_traj[1][:]), (place_arm_wrench[0][:], place_arm_wrench[1][:])]
                    self.global_blackboard.ArmPoseAndWrench = arm_pose_and_wrench
                    self.success = True
                else:
                    self.success = False
                    self.feedback_message = "抓取失败: step解析错误"

                if self.success:
                    self.feedback_message = "calculate success."
                    self.logger.info(f"In t, {self.feedback_message}")
                    return Status.SUCCESS
                else:
                    self.logger.info(f"In MoveArmBaseTargetPoint, {self.feedback_message}")
                    return Status.FAILURE
            else:
                # left arm pose when tag is on the left
                left_arm_roll, left_arm_pitch, left_arm_yaw  = -90, 0, 0              #0, 45, 90
                grasp_left_pose_left_tag=[
                    #左手预抓取点位
                    Pose.from_euler(pos=(-box_length / 2 , box_height/2+0.20, -box_width/2), euler=(left_arm_roll, left_arm_pitch, left_arm_yaw), degrees=True,
                                    frame=Frame.TAG),
                    #左手抓取点位
                    Pose.from_euler(pos=(-box_length / 2 , box_height/2, -box_width/2), euler=(left_arm_roll, left_arm_pitch, left_arm_yaw), degrees=True,
                                    frame=Frame.TAG),
                    #close left claw
                    # lift a little.
                    Pose.from_euler(pos=(-box_length / 2, box_height/2 + 0.08, -box_width/2), euler=(left_arm_roll, left_arm_pitch, left_arm_yaw), degrees=True,
                                    frame=Frame.TAG),
                    #左手拖动点位
                    Pose.from_euler(pos=(-box_length / 2 -0.10, box_height/2 + 0.08, -box_width/2), euler=(left_arm_roll, left_arm_pitch, left_arm_yaw), degrees=True,
                                    frame=Frame.TAG),

                    Pose.from_euler(pos=(-box_length / 2 -0.10, box_height/2, -box_width/2), euler=(left_arm_roll, left_arm_pitch, left_arm_yaw), degrees=True,
                                    frame=Frame.TAG),
                    #wait point
                    Pose.from_euler(pos=(-box_length / 2 - 0.10 , box_height/2, -box_width/2), euler=(left_arm_roll, left_arm_pitch, left_arm_yaw), degrees=True,
                                    frame=Frame.TAG),

                    Pose.from_euler(pos=(-box_length / 2 - 0.1 , box_height/2 + 0.1, -box_width/2), euler=(left_arm_roll, left_arm_pitch, left_arm_yaw), degrees=True,
                                    frame=Frame.TAG),

                    Pose.from_euler(pos=(0.30, box_length/2, 0.20), euler=(0, -60, 0), degrees=True,
                                    frame=Frame.BASE)
                ]

                # left arm pose when tag is on the right
                grasp_left_pose_right_tag=[
                    #左手预抓取点位
                    Pose.from_euler(pos=(-box_length / 2 , box_height/2+0.20, -box_width/2), euler=(left_arm_roll, left_arm_pitch, left_arm_yaw), degrees=True,
                                    frame=Frame.TAG),
                    #左手抓取点位
                    Pose.from_euler(pos=(-box_length / 2 , box_height/2, -box_width/2), euler=(left_arm_roll, left_arm_pitch, left_arm_yaw), degrees=True,
                                    frame=Frame.TAG),

                    Pose.from_euler(pos=(-box_length / 2, box_height/2 + 0.1, -box_width/2), euler=(left_arm_roll, left_arm_pitch, left_arm_yaw), degrees=True,
                                    frame=Frame.TAG),

                    Pose.from_euler(pos=(0.30, box_length/2, 0.20), euler=(0, -60, 0), degrees=True,
                                    frame=Frame.BASE)
                ]

                right_arm_roll, right_arm_pitch, right_arm_yaw = 90, 0, 180  # 0, 45, 90
                # right arm pose when tag is on the left
                grasp_right_pose_left_tag=[
                    #右手预抓取点位
                    Pose.from_euler(pos=(box_length / 2 - 0.1 , box_height/2+0.20, -box_width/2), euler=(right_arm_roll, right_arm_pitch, right_arm_yaw), degrees=True,
                                    frame=Frame.TAG),
                    #右手等待点位
                    Pose.from_euler(pos=(box_length / 2 - 0.1 , box_height/2+0.20, -box_width/2), euler=(right_arm_roll, right_arm_pitch, right_arm_yaw), degrees=True,
                                    frame=Frame.TAG),
                    #右手等待点位
                    Pose.from_euler(pos=(box_length / 2 - 0.1, box_height/2 + 0.20, -box_width/2), euler=(right_arm_roll, right_arm_pitch, right_arm_yaw), degrees=True,
                                    frame=Frame.TAG),

                    Pose.from_euler(pos=(box_length / 2 - 0.1, box_height/2 + 0.20, -box_width/2), euler=(right_arm_roll, right_arm_pitch, right_arm_yaw), degrees=True,
                                    frame=Frame.TAG),

                    Pose.from_euler(pos=(box_length / 2 - 0.1, box_height/2 + 0.20, -box_width/2), euler=(right_arm_roll, right_arm_pitch, right_arm_yaw), degrees=True,
                                    frame=Frame.TAG),
                    #右手等待点位
                    Pose.from_euler(pos=(box_length / 2 - 0.1 , box_height/2, -box_width/2), euler=(right_arm_roll, right_arm_pitch, right_arm_yaw), degrees=True,
                                    frame=Frame.TAG),

                    Pose.from_euler(pos=(box_length / 2 - 0.1 , box_height/2 + 0.1, -box_width/2), euler=(right_arm_roll, right_arm_pitch, right_arm_yaw), degrees=True,
                                    frame=Frame.TAG),
                    #close right claw
                    Pose.from_euler(pos=(0.30, -box_length / 2, 0.20), euler=(0, -60, 0), degrees=True,
                                    frame=Frame.BASE)
                ]

                # right arm pose when tag is on the right
                grasp_right_pose_right_tag=[
                    #左手预抓取点位
                    Pose.from_euler(pos=(box_length / 2 , box_height/2+0.20, -box_width/2), euler=(right_arm_roll, right_arm_pitch, right_arm_yaw), degrees=True,
                                    frame=Frame.TAG),
                    #左手抓取点位
                    Pose.from_euler(pos=(box_length / 2 , box_height/2, -box_width/2), euler=(right_arm_roll, right_arm_pitch, right_arm_yaw), degrees=True,
                                    frame=Frame.TAG),

                    Pose.from_euler(pos=(box_length / 2, box_height/2 + 0.1, -box_width/2), euler=(right_arm_roll, right_arm_pitch, right_arm_yaw), degrees=True,
                                    frame=Frame.TAG),

                    Pose.from_euler(pos=(0.30, -box_length/2, 0.20), euler=(0, -60, 0), degrees=True,
                                    frame=Frame.BASE)
                ]


                # ================ 计算每个关键点的力控目标（wrench） ================ #
                # 计算夹持力参数
                g = 9.8  # 重力加速度

                # 计算基础Z向力（考虑安全系数和经验比例）
                force_z = -abs(box_mass * g * force_ratio_z)

                # 判断是否为仿真模式
                left_force = lateral_force  # 左手侧向力（正值为夹紧方向）
                right_force = -lateral_force  # 右手侧向力（负值为夹紧方向）

                grasp_left_arm_wrench = [
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第一关键点的扭矩
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第二关键点的扭矩
                    [0, 0, force_z, 0, 0, 0],  # 第三关键点的扭矩
                    [0.0, 0, force_z, 0.0, 0.0, 0.0],  # 第四关键点的扭矩
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第五关键点的扭矩
                    [0, 0, 0, 0, 0, 0],  # 第六关键点的扭矩
                    [0, 0, force_z, 0, 0, 0],  # 第六关键点的扭矩
                    [0, 0, force_z, 0, 0, 0]  # 第六关键点的扭矩
                ]

                grasp_right_arm_wrench = [
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第一关键点的扭矩
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第二关键点的扭矩
                    [0.0, 0, force_z, 0.0, 0.0, 0.0],  # 第三关键点的扭矩
                    [0.0, 0, force_z, 0.0, 0.0, 0.0],  # 第四关键点的扭矩
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第五关键点的扭矩
                    [0, 0, 0, 0, 0, 0],  # 第六关键点的扭矩
                    [0, 0, force_z, 0, 0, 0],  # 第六关键点的扭矩
                    [0, 0, force_z, 0, 0, 0]  # 第六关键点的扭矩
                ]

                tag_left_or_right = self.global_blackboard.TagLeftOrRight
                print(f"tag is on the {tag_left_or_right}")

                if tag_left_or_right == 'left':
                    arm_traj = (grasp_left_pose_left_tag, grasp_right_pose_left_tag)
                    arm_wrench = (grasp_left_arm_wrench, grasp_right_arm_wrench)
                    step1_param = [(arm_traj[0][:2], arm_traj[1][:2]), (arm_wrench[0][:2], arm_wrench[1][:2])]
                    step2_param = [(arm_traj[0][2:6], arm_traj[1][2:6]), (arm_wrench[0][2:6], arm_wrench[1][2:6])]
                    step3_param = [(arm_traj[0][6:], arm_traj[1][6:]), (arm_wrench[0][6:], arm_wrench[1][6:])]
                else:
                    arm_traj = (grasp_left_pose_right_tag, grasp_right_pose_right_tag)
                    wrench = [
                        [0, 0, 0, 0, 0, 0, 0],
                        [0, 0, force_z, 0, 0, 0],
                        [0, 0, force_z, 0, 0, 0],
                        [0, 0, force_z, 0, 0, 0]
                    ]
                    arm_wrench = (wrench, wrench)
                    step1_param = [(arm_traj[0][:2], arm_traj[1][:2]), (arm_wrench[0][:2], arm_wrench[1][:2])]
                    step2_param = [(arm_traj[0][1:2], arm_traj[1][1:2]), (arm_wrench[0][1:2], arm_wrench[1][1:2])]
                    step3_param = [(arm_traj[0][2:], arm_traj[1][2:]), (arm_wrench[0][2:], arm_wrench[1][2:])]

                if self.mode == 'depalletize_grasp_step1':
                    self.global_blackboard.ArmPoseAndWrench = step1_param
                    self.success = True
                elif self.mode == 'depalletize_grasp_step2':
                    self.global_blackboard.ArmPoseAndWrench = step2_param
                    self.success = True
                elif self.mode == 'depalletize_grasp_step3':
                    self.global_blackboard.ArmPoseAndWrench = step3_param
                    self.success = True
                elif self.mode == 'depalletize_place':
                    place_right_pose = [ Pose.from_euler(pos=(0.42, -box_length / 2, 0.20), euler=(0, -60, 0), degrees=True,
                                    frame=Frame.BASE)]
                    place_left_pose = [ Pose.from_euler(pos=(0.42, box_length / 2, 0.20), euler=(0, -60, 0), degrees=True,
                                    frame=Frame.BASE)]
                    place_left_wrench = [
                        [0, 0, 0, 0, 0, 0]
                    ]
                    place_right_wrench = [
                        [0, 0, 0, 0, 0, 0]
                    ]
                    arm_traj = (place_left_pose, place_right_pose)
                    place_arm_wrench = (place_left_wrench, place_right_wrench)
                    arm_pose_and_wrench = [(arm_traj[0][:], arm_traj[1][:]), (place_arm_wrench[0][:], place_arm_wrench[1][:])]
                    self.global_blackboard.ArmPoseAndWrench = arm_pose_and_wrench
                    self.success = True
                else:
                    self.success = False
                    self.feedback_message = "抓取失败: step解析错误"

                if self.success:
                    self.feedback_message = "calculate success."
                    self.logger.info(f"In t, {self.feedback_message}")
                    return Status.SUCCESS
                else:
                    self.logger.info(f"In MoveArmBaseTargetPoint, {self.feedback_message}")
                    return Status.FAILURE

    def _smt_update(self):
        print(f"self.mode = {self.mode}")
        if self.mode in self.smt_arm_trajectories:
            print(f"self.mode = {self.mode}, and is in self.smt_arm_trajectories")
            # 如果 mode 在 arm_trajectories 中，直接写入 ArmJointTrajectories
            poses = self.smt_arm_trajectories[self.mode]
            # 检查数据结构是否为单一目标点（如 [1, [...]])
            if isinstance(poses, list) and len(poses) == 2 and isinstance(poses[0], (int, float)):
                # 处理单一目标点的情况
                time, angles = poses
                target_poses = [[time, np.deg2rad(angles).tolist()]]
            else:
            # 处理多目标点的情况
                target_poses = [[time, np.deg2rad(angles).tolist()] for time, angles in poses]

            times = [pose[0] for pose in target_poses]
            q_frames = [pose[1] for pose in target_poses]

            self.global_blackboard.ArmJointTrajectories = {"times": times, "q_frames": q_frames}
            return Status.SUCCESS
        elif self.mode == "smt_pick_pose":
            return self._handle_pick_pose()
        elif self.mode == "smt_pick_pre_pose":
            return self._handle_pick_pre_pose()
        elif self.mode == "smt_pick_post_pose":
            return self._handle_pick_post_pose()
        elif self.mode == "smt_post_pose":
            return self._handle_post_pose()
        elif self.mode == "smt_place_pre_pose":
            return self._handle_place_pre_pose()
        elif self.mode == "smt_place_post_pose":
            return self._handle_place_post_pose()
        else:
            self.logger.error(f"未知模式: {self.mode}")
            return Status.FAILURE

    def _box_update(self):
        if self.mode.startswith('box_grasp'):
            try:
                box_length = float(getattr(self.global_blackboard, "Common_BoxLength"))
                box_width = float(getattr(self.global_blackboard, "Common_BoxWidth"))
                box_height = float(getattr(self.global_blackboard, "Common_BoxHeight"))
                force_ratio_z = float(getattr(self.global_blackboard, "GraspBox_ForceRatioZ"))
                lateral_force = float(getattr(self.global_blackboard, "GraspBox_LateralForce"))
                box_mass = float(getattr(self.global_blackboard, "Common_BoxMass"))

                height_compensation = 0.0
                pick_left_arm_poses = [
                    # 1. 预抓取点位
                    Pose.from_euler(pos=( - 0.3, -0.05, -0.02), euler=(0, -0, 90), degrees=True,
                                    frame=Frame.TAG),
                    # 2. 并拢点位
                    Pose.from_euler(pos=(-0.17, -0.05, 0.01), euler=(-20, -15,90), degrees=True,
                                    frame=Frame.TAG),

                    Pose.from_euler(pos=(-0.17, 0.10, 0.01), euler=(-20, -15, 90), degrees=True,
                                    frame=Frame.TAG),
                    ]

                pick_right_arm_poses = [
                    # 1. 预抓取点位
                    Pose.from_euler(pos=(0.3, -0.05, -0.02), euler=(0, -0, 90), degrees=True,
                                    frame=Frame.TAG),
                    # 2. 并拢点位
                    Pose.from_euler(pos=(0.17, -0.05, 0.01), euler=(20, 15, 90), degrees=True,
                                    frame=Frame.TAG),
                    # 2. 并拢点位
                    Pose.from_euler(pos=(0.17, 0.10, 0.01), euler=(20, 15, 90), degrees=True,
                                    frame=Frame.TAG),
                    ]

                pick_left_arm_poses1 = [
                    # 4. 收臂点位
                    Pose.from_euler(pos=(0.40, 0.17, 0.2), euler=(0, -90, -5), degrees=True, frame=Frame.BASE)]

                pick_right_arm_poses1 = [
                    # 4. 收臂点位
                    Pose.from_euler(pos=(0.40, -0.17, 0.2), euler=(0, -90, 5), degrees=True, frame=Frame.BASE)]


                # ================ 计算每个关键点的力控目标（wrench） ================ #
                # 计算夹持力参数
                g = 9.8  # 重力加速度
                # 计算基础Z向力（考虑安全系数和经验比例）
                force_ratio_z = 1
                force_z = -abs(box_mass * g * force_ratio_z)
                lateral_force = 13
                # 判断是否为仿真模式
                left_force = lateral_force  # 左手侧向力（正值为夹紧方向）
                right_force = -lateral_force  # 右手侧向力（负值为夹紧方向）

                pick_left_arm_wrench = [
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第一关键点的扭矩
                    [0.0, left_force, force_z, 0.0, 0.0, 0.0],  # 第二关键点的扭矩
                    [0.0, left_force, force_z, 0.0, 0.0, 0.0],  # 第二关键点的扭矩
                ]

                pick_right_arm_wrench = [
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 第一关键点的扭矩
                    [0.0, right_force, force_z, 0.0, 0.0, 0.0],  # 第二关键点的扭矩
                    [0.0, right_force, force_z, 0.0, 0.0, 0.0],  # 第二关键点的扭矩
                ]

                pick_left_arm_wrench1 = [
                    [0, left_force,  force_z, 0, 0, 0]   # 第四关键点的扭矩
                ]

                pick_right_arm_wrench1 = [
                    [0, right_force, force_z, 0, 0, 0]   # 第四关键点的扭矩
                ]

                arm_traj1 = (pick_left_arm_poses, pick_right_arm_poses)
                arm_traj2 = (pick_left_arm_poses1, pick_right_arm_poses1)
                arm_wrench1 = (pick_left_arm_wrench, pick_right_arm_wrench)  # 手臂扭矩数据
                arm_wrench2 = (pick_left_arm_wrench1, pick_right_arm_wrench1)
                if self.mode == 'box_grasp_step1':
                    self.global_blackboard.ArmPoseAndWrench = [arm_traj1, arm_wrench1]
                elif self.mode == 'box_grasp_step2':
                    self.global_blackboard.ArmPoseAndWrench = [arm_traj2, arm_wrench2]
                else:
                    self.feedback_message = f"未知模式: {self.mode}"
                    return Status.FAILURE

                self.feedback_message = "calculate success."
                return Status.SUCCESS

            except Exception as e:
                self.feedback_message = f"抓取失败: {e}"
                self.success = False
                return Status.FAILURE

        if self.mode.startswith('box_place'):
            try:
                box_length = float(getattr(self.global_blackboard, "Common_BoxLength"))
                place_left_arm_poses = [
                    Pose.from_euler(pos=(0.40, 0.17, 0.2), euler=(0, -90, -5), degrees=True, frame=Frame.BASE),
                    # 1. 打开点位
                    Pose.from_euler(pos=(- 0.17, -0.2, 0.0), euler=(-20, -15, 90), degrees=True,
                                    frame=Frame.TAG),
                    Pose.from_euler(pos=(- 0.3, -0.2, 0.0), euler=(-20, -15, 90), degrees=True,
                                    frame=Frame.TAG),
                    # Pose.from_euler(pos=(0.6, 0.24, 0.13), euler=(0, -90, 0), degrees=True,
                    #                 frame=Frame.BASE),
                ]
                place_right_arm_poses = [
                    Pose.from_euler(pos=(0.40, -0.17, 0.2), euler=(0, -90, -5), degrees=True, frame=Frame.BASE),
                    # 1. 打开点位
                    Pose.from_euler(pos=(0.17, -0.2, 0.0), euler=(20, 15, 90), degrees=True,
                                    frame=Frame.TAG),
                    Pose.from_euler(pos=(0.3, -0.2, 0.0), euler=(20, 15, 90), degrees=True,
                                    frame=Frame.TAG),
                    # Pose.from_euler(pos=(0.6, -0.24, 0.13), euler=(0, -90, 0), degrees=True,
                    #                 frame=Frame.BASE),
                ]

                place_left_arm_wrench = [
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                ]

                place_right_arm_wrench = [
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                ]

                arm_traj = (place_left_arm_poses, place_right_arm_poses)
                arm_wrench = (place_left_arm_wrench, place_right_arm_wrench)  # 手臂扭矩数据
                self.global_blackboard.ArmPoseAndWrench = [arm_traj, arm_wrench]

                self.executed = True
                self.success = True
                return Status.SUCCESS

            except Exception as e:
                self.feedback_message = f"calculate false: {e}"
                self.success = False
                return Status.FAILURE

        # 如果 mode 不是 box_grasp 或 box_place，返回失败
        self.logger.error(f"未知的 box mode: {self.mode}")
        return Status.FAILURE

    def _handle_pick_pose(self):
        """处理 pick_pose 模式"""
        pick_offsets = self._build_pick_offsets(
            pre_pos=self.smt_pick_pose_params["pre_pos"],
            pre_euler_deg=self.smt_pick_pose_params["pre_euler_deg"],
            grasp_pos=self.smt_pick_pose_params["grasp_pos"],
            grasp_euler_deg=self.smt_pick_pose_params["grasp_euler_deg"]
        )
        tag_id = int(self.global_blackboard.TargetSMTID)
        tag_list = self.global_blackboard.AllTagInfoOfBase

        self.logger.info(f"pick_offsets 抓取偏移= {pick_offsets}")

        cur_tag_pose = None
        for tag_id_in_list, tag_pose in zip(tag_list.id, tag_list.pose):
            if tag_id_in_list == tag_id:
                cur_tag_pose = tag_pose
                break

        if cur_tag_pose is None:
            self.logger.error(f"未找到目标 Tag ID: {tag_id}")
            return Status.FAILURE

        self.global_blackboard.TargetTag = Tag(
            id=tag_id,
            pose=Pose.from_euler(
                pos=(cur_tag_pose.position.x, cur_tag_pose.position.y, cur_tag_pose.position.z),
                euler=(90, 0, -90),
                frame=Frame.BASE,
                degrees=True
            )
        )

        pick_left_arm_poses = [
            self._build_pose_from_tag(cur_tag_pose.position, pick_offsets['pre_grasp']['pos_offset'], pick_offsets['pre_grasp']['euler_deg']),
            self._build_pose_from_tag(cur_tag_pose.position, pick_offsets['grasp']['pos_offset'], pick_offsets['grasp']['euler_deg']),
        ]

        pick_right_arm_poses = [
            Pose.from_euler(pos=(0.004345, -0.292643, -0.270229), euler=(0.0374, -14.5817, 0.0205), degrees=True, frame=Frame.BASE),
            Pose.from_euler(pos=(0.004345, -0.292643, -0.270229), euler=(0.0374, -14.5817, 0.0205), degrees=True, frame=Frame.BASE),
        ]

        pick_left_arm_wrench =  [[0.0] * 6, [0.0] * 6]
        pick_right_arm_wrench = [[0.0] * 6, [0.0] * 6]

        self.global_blackboard.ArmPoseAndWrench = [
            (pick_left_arm_poses, pick_right_arm_poses),
            (pick_left_arm_wrench, pick_right_arm_wrench)
        ]
        return Status.SUCCESS

    def _handle_pick_pre_pose(self):
        """处理 pick_pre_pose 模式"""
        pick_offsets = self._build_pick_offsets(
            pre_pos=self.smt_pick_pose_params["pre_pos"],
            pre_euler_deg=self.smt_pick_pose_params["pre_euler_deg"],
            grasp_pos=self.smt_pick_pose_params["grasp_pos"],
            grasp_euler_deg=self.smt_pick_pose_params["grasp_euler_deg"]
        )
        tag_id = int(self.global_blackboard.TargetSMTID)
        tag_list = self.global_blackboard.AllTagInfoOfBase

        self.logger.info(f"pick_offsets 抓取偏移= {pick_offsets}")

        cur_tag_pose = None
        for tag_id_in_list, tag_pose in zip(tag_list.id, tag_list.pose):
            if tag_id_in_list == tag_id:
                cur_tag_pose = tag_pose
                break

        if cur_tag_pose is None:
            self.logger.error(f"未找到目标 Tag ID: {tag_id}")
            return Status.FAILURE

        self.global_blackboard.TargetTag = Tag(
            id=tag_id,
            pose=Pose.from_euler(
                pos=(cur_tag_pose.position.x, cur_tag_pose.position.y, cur_tag_pose.position.z),
                euler=(90, 0, -90),
                frame=Frame.BASE,
                degrees=True
            )
        )

        pick_left_arm_poses = [
            self._build_pose_from_tag(cur_tag_pose.position, pick_offsets['pre_grasp']['pos_offset'], pick_offsets['pre_grasp']['euler_deg']),
        ]

        self.logger.info(f" 预先抓取的位置是: {cur_tag_pose.position} \n")
        self.logger.info(f" 预先抓取的偏移是: { pick_offsets['pre_grasp']['pos_offset']} \n")
        self.logger.info(f" 抓取左手坐标pose是: { pick_left_arm_poses } \n")

        pick_right_arm_poses = [
            Pose.from_euler(pos=(0.004345, -0.292643, -0.270229), euler=(0.0374, -14.5817, 0.0205), degrees=True, frame=Frame.BASE),
        ]

        pick_left_arm_wrench =  [[0.0] * 6]
        pick_right_arm_wrench = [[0.0] * 6]

        self.global_blackboard.ArmPoseAndWrench = [
            (pick_left_arm_poses, pick_right_arm_poses),
            (pick_left_arm_wrench, pick_right_arm_wrench)
        ]
        return Status.SUCCESS

    def _handle_pick_post_pose(self):
        """处理 pick_pose 模式"""
        pick_offsets = self._build_pick_offsets(
            pre_pos=self.smt_pick_pose_params["pre_pos"],
            pre_euler_deg=self.smt_pick_pose_params["pre_euler_deg"],
            grasp_pos=self.smt_pick_pose_params["grasp_pos"],
            grasp_euler_deg=self.smt_pick_pose_params["grasp_euler_deg"]
        )
        tag_id = int(self.global_blackboard.TargetSMTID)
        tag_list = self.global_blackboard.AllTagInfoOfBase

        self.logger.info(f"pick_offsets 抓取偏移= {pick_offsets}")

        cur_tag_pose = None
        for tag_id_in_list, tag_pose in zip(tag_list.id, tag_list.pose):
            if tag_id_in_list == tag_id:
                cur_tag_pose = tag_pose
                break

        if cur_tag_pose is None:
            self.logger.error(f"未找到目标 Tag ID: {tag_id}")
            return Status.FAILURE

        self.global_blackboard.TargetTag = Tag(
            id=tag_id,
            pose=Pose.from_euler(
                pos=(cur_tag_pose.position.x, cur_tag_pose.position.y, cur_tag_pose.position.z),
                euler=(90, 0, -90),
                frame=Frame.BASE,
                degrees=True
            )
        )

        pick_left_arm_poses = [
            self._build_pose_from_tag(cur_tag_pose.position, pick_offsets['grasp']['pos_offset'], pick_offsets['grasp']['euler_deg']),
        ]

        pick_right_arm_poses = [
            Pose.from_euler(pos=(0.004345, -0.292643, -0.270229), euler=(0.0374, -14.5817, 0.0205), degrees=True, frame=Frame.BASE),
        ]

        self.logger.info(f" 检测后抓取的位置是: {cur_tag_pose.position} \n")
        self.logger.info(f" 检测后抓取的偏移是: { pick_offsets['grasp']['pos_offset']} \n")
        self.logger.info(f" 抓取左手坐标pose是: { pick_left_arm_poses } \n")

        pick_left_arm_wrench =  [[0.0] * 6]
        pick_right_arm_wrench = [[0.0] * 6]

        self.global_blackboard.ArmPoseAndWrench = [
            (pick_left_arm_poses, pick_right_arm_poses),
            (pick_left_arm_wrench, pick_right_arm_wrench)
        ]
        return Status.SUCCESS

    def _handle_post_pose(self):
        """处理 post_pose 模式"""
        # 类似 pick_pose 的逻辑，处理抓取后的位姿
        post_offsets = self._build_post_grasp_offsets(
            withdraw_pos_offset=self.smt_post_pose_params["withdraw_pos_offset"],
            withdraw_euler_deg=self.smt_post_pose_params["withdraw_euler_deg"],
            observe_pos_abs=self.smt_post_pose_params["observe_pos_abs"],
            observe_euler_deg=self.smt_post_pose_params["observe_euler_deg"],
        )
        tag_id = int(self.global_blackboard.TargetSMTID)
        tag_list = self.global_blackboard.AllTagInfoOfBase
        print(f"tag_list = {tag_list}")
        print(f"tag_id = {tag_id}, type of tag_id {type(tag_id)}")

        cur_tag_pose = None

        # 方法1：使用zip组合id和pose
        for tag_id_in_list, tag_pose in zip(tag_list.id, tag_list.pose):
            if tag_id_in_list == tag_id:
                cur_tag_pose = tag_pose
                break

        print(f"cur_tag_pose: {cur_tag_pose}")
        self.global_blackboard.TargetTag = Tag(
            id=tag_id,
            pose=Pose.from_euler(
                pos=(cur_tag_pose.position.x, cur_tag_pose.position.y, cur_tag_pose.position.z),
                euler=(90, 0, -90),
                frame=Frame.BASE,  # 假设感知到的tag位姿在odom坐标系下
                degrees=True
            )
        )

        pick_left_arm_poses = [
            self._build_pose_from_tag(cur_tag_pose.position, post_offsets['withdraw']['pos_offset'], post_offsets['withdraw']['euler_deg']),
            Pose.from_euler(pos=post_offsets['observe']['pos_abs'], euler=post_offsets['observe']['euler_deg'], degrees=True,
                        frame=Frame.BASE),
        ]
        pick_right_arm_poses = [
            Pose.from_euler(pos=(0.004345, -0.292643,  -0.270229), euler=(0.0374, -14.5817, 0.0205), degrees=True,
                            frame=Frame.BASE),
            Pose.from_euler(pos=(0.004345, -0.292643,  -0.270229), euler=(0.0374, -14.5817, 0.0205), degrees=True,
                            frame=Frame.BASE),
        ]

        pick_left_arm_wrench = [
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        pick_right_arm_wrench = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]

        arm_traj = (pick_left_arm_poses, pick_right_arm_poses)
        arm_wrench = (pick_left_arm_wrench, pick_right_arm_wrench)

        self.global_blackboard.ArmPoseAndWrench = [arm_traj, arm_wrench]
        return Status.SUCCESS

    def _handle_place_pre_pose(self):
        # 用于给一个假的 tag
        self.global_blackboard.TargetTag = Tag(
            id=0,
            pose=Pose.from_euler(
                pos=(0.3, 0.2, 0.2),
                euler=(90, 0, -90),
                frame=Frame.BASE,
                degrees=True
            )
        )

        pick_left_arm_poses = [
                Pose.from_euler(pos=(0.5, 0.2,  -0.25), euler=(90, 0.0, -90), degrees=True,
                            frame=Frame.BASE),
            ]
        pick_right_arm_poses = [
            Pose.from_euler(pos=(0.004345, -0.292643,  -0.270229), euler=(0.0374, -14.5817, 0.0205), degrees=True,
                            frame=Frame.BASE),
        ]

        pick_left_arm_wrench = [
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        pick_right_arm_wrench = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        arm_traj = (pick_left_arm_poses, pick_right_arm_poses)
        arm_wrench = (pick_left_arm_wrench, pick_right_arm_wrench)

        self.global_blackboard.ArmPoseAndWrench = [arm_traj, arm_wrench]
        return Status.SUCCESS

    def _handle_place_post_pose(self):

        self.global_blackboard.TargetTag = Tag(
            id=0,
            pose=Pose.from_euler(
                pos=(0.3, 0.2, 0.2),
                euler=(90, 0, -90),
                frame=Frame.BASE,
                degrees=True
            )
        )

        pick_left_arm_poses = [
                Pose.from_euler(pos=(0.004345, 0.292643,  -0.270229), euler=(0.0374, 14.5817, 0.0205), degrees=True,
                            frame=Frame.BASE),
            ]
        pick_right_arm_poses = [

            Pose.from_euler(pos=(0.004345, -0.292643,  -0.270229), euler=(0.0374, -14.5817, 0.0205), degrees=True,
                            frame=Frame.BASE),
        ]

        pick_left_arm_wrench = [
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        pick_right_arm_wrench = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        arm_traj = (pick_left_arm_poses, pick_right_arm_poses)
        arm_wrench = (pick_left_arm_wrench, pick_right_arm_wrench)

        self.global_blackboard.ArmPoseAndWrench = [arm_traj, arm_wrench]
        return Status.SUCCESS

    def _read_board_value(self, key):
        """从 board.json 中读取指定键的值"""
        self.global_blackboard.register_key(key=key, access=py_trees.common.Access.READ)
        value = self.global_blackboard.get(key)
        if value is None:
            raise KeyError(f"Key '{key}' not found in board.json")
        return [float(x) for x in value.split(",")]

    def _build_pose_from_tag(self, tag_position, pos_offset, euler_deg):
        """根据 tag 的位置与给定偏移/欧拉角构造 Pose（Frame.BASE）"""
        return Pose.from_euler(
            pos=(
                tag_position.x + pos_offset[0],
                tag_position.y + pos_offset[1],
                tag_position.z + pos_offset[2],
            ),
            euler=euler_deg,
            degrees=True,
            frame=Frame.BASE,
        )

    def _build_pick_offsets(self, pre_pos, pre_euler_deg, grasp_pos, grasp_euler_deg):
        """构造抓取阶段偏移结构"""
        return {
            'pre_grasp': {
                'pos_offset': pre_pos,
                'euler_deg': pre_euler_deg,
            },
            'grasp': {
                'pos_offset': grasp_pos,
                'euler_deg': grasp_euler_deg,
            },
        }

    def _build_post_grasp_offsets(
            self,
        withdraw_pos_offset: Tuple[float, float, float],
        withdraw_euler_deg: Tuple[float, float, float],
        observe_pos_abs: Tuple[float, float, float],
        observe_euler_deg: Tuple[float, float, float],
    ) -> Dict[str, Dict[str, Tuple[float, float, float]]]:
        """统一构造抓取后的撤手与观察位姿结构。"""
        return {
            'withdraw': {
                'pos_offset': withdraw_pos_offset,
                'euler_deg': withdraw_euler_deg,
            },
            'observe': {
                'pos_abs': observe_pos_abs,
                'euler_deg': observe_euler_deg,
            },
        }
