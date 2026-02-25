import sys
import os
import time
import math
from pydrake.all import (
    DiagramBuilder,
    Parser,
    PiecewisePolynomial,
    FixedOffsetFrame,
    RigidTransform,
    InverseKinematics,
    Solve,
    RotationMatrix,
    RollPitchYaw,
    SnoptSolver,
    JacobianWrtVariable,
)
from pydrake.all import StartMeshcat, AddMultibodyPlantSceneGraph, MeshcatVisualizer

import numpy as np

current_dir = os.path.dirname(os.path.abspath(__file__))
tools_dir = os.path.abspath(os.path.join(current_dir, '..', 'tools'))
sys.path.append(tools_dir)
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate.utils import ArmIdx, IkTypeIdx


class TorsoIK:
    def __init__(
            self,
            plant,
            frame_name_list,
            constraint_tol=1.0e-8,
            solver_tol=1.0e-6,
            iterations_limit=1000,
            ctrl_arm_idx=ArmIdx.LEFT,
            as_mc_ik=True,
            # elbow_frame_name_list=None,
    ):
        self.__plant = plant
        self.__plant_context = self.__plant.CreateDefaultContext()
        self.__constraint_tol = constraint_tol
        self.__iterations_limit = iterations_limit
        self.__solver_tol = solver_tol
        self.__frames = [self.__plant.GetFrameByName(name) for name in frame_name_list]
        self.__ctrl_arm_idx = ctrl_arm_idx
        self.__as_mc_ik = as_mc_ik
        self.__nq = self.__plant.num_positions()
        self.__is_roban_dof = self.__nq == 8
        if not self.__is_roban_dof:
            self.__one_stage_ee_frames = [self.__plant.GetFrameByName("zarm_l6_link"),
                                          self.__plant.GetFrameByName("zarm_r6_link")]
        else:
            self.__one_stage_ee_frames = []
        print(f"[TorsoIK] nq: {self.__nq}")
        # self.__elbow_frames = [self.__plant.GetFrameByName(name) for name in elbow_frame_name_list] if elbow_frame_name_list is not None else None

    def set_as_mc_ik(self, as_mc_ik):
        print(f"[TorsoIK] set_as_mc_ik: {as_mc_ik}")
        self.__as_mc_ik = as_mc_ik

    def solve(self, pose_list, q0=[], left_shoulder_rpy=None, right_shoulder_rpy=None, last_solution=None):
        self.__IK = InverseKinematics(plant=self.__plant, with_joint_limits=True)
        snopt = SnoptSolver().solver_id()
        self.__IK.prog().SetSolverOption(
            snopt, "Major Optimality Tolerance", self.__solver_tol
        )
        self.__IK.prog().SetSolverOption(
            snopt, "Major Iterations Limit", self.__iterations_limit
        )
        for i, frame in enumerate(self.__frames):
            if not pose_list[i][0] is None:
                # print(f"ori: {i}")
                if not self.__is_roban_dof or i == 0:  # 只有单手大于6自由度才进行姿态约束，躯干一直进行姿态约束
                # if i == 0:  # 躯干一直进行姿态约束
                    self.__IK.AddOrientationConstraint(
                        self.__plant.world_frame(),
                        RotationMatrix(RollPitchYaw(pose_list[i][0])),
                        frame,
                        RotationMatrix(RollPitchYaw(0, 0, 0)),
                        self.__constraint_tol,
                    )
                elif not self.__is_roban_dof: # 只有单手大于6自由度才使用姿态代价
                    self.__IK.AddOrientationCost(
                        frameAbar=self.__plant.world_frame(),
                        R_AbarA=RotationMatrix(RollPitchYaw(pose_list[i][0])),
                        frameBbar=frame,
                        R_BbarB=RotationMatrix(RollPitchYaw(0, 0, 3.1415)), # urdf中末端执行器frame有旋转
                        # theta_bound=self.__constraint_tol
                        c=0.0,
                    )
            if not pose_list[i][1] is None:
                # print(f"pos: {i}")
                if self.__as_mc_ik and i != 0:
                    cost_matrix = 10 * np.eye(3)
                    if i > 2:  # 3,4 -> elbow
                        cost_matrix = (10 * np.eye(3)) if self.__is_roban_dof else (1 * np.eye(3))
                    self.__IK.AddPositionCost(
                        frameA=self.__plant.world_frame(),
                        p_AP=pose_list[i][1],
                        frameB=frame,
                        p_BQ=np.zeros(3),
                        C=cost_matrix,
                    )
                if (not self.__as_mc_ik and i <= 2) or (self.__as_mc_ik and i == 0):  # torso
                    self.__IK.AddPositionConstraint(frameB=frame,
                                                    p_BQ=np.zeros(3),
                                                    frameA=self.__plant.world_frame(),
                                                    p_AQ_lower=np.array(pose_list[i][1]) - self.__constraint_tol,
                                                    p_AQ_upper=np.array(pose_list[i][1]) + self.__constraint_tol)

        if last_solution is not None:
            W_vec = np.array([0.1] * self.__plant.num_positions())
            if self.__is_roban_dof:  # yaw调整可以很快
                W_vec[2] = 0.0
                W_vec[6] = 0.0
            # W_vec[7:10] *= 10 # shoulder joints
            # W_vec[14:17] *= 10
            W_prev_solution = np.diag(W_vec)
            self.__IK.prog().AddQuadraticErrorCost(W_prev_solution, last_solution, self.__IK.q())

        # if self.__ctrl_arm_idx == ArmIdx.LEFT or self.__ctrl_arm_idx == ArmIdx.BOTH:
        #     self.__IK.prog().AddQuadraticErrorCost(W, q_normal, self.__IK.q()[7:14])
        # if self.__ctrl_arm_idx == ArmIdx.RIGHT or self.__ctrl_arm_idx == ArmIdx.BOTH:
        #     self.__IK.prog().AddQuadraticErrorCost(W, q_normal_right, self.__IK.q()[-7:])
        # self.__IK.prog().AddQuadraticErrorCost(np.diag([0.01]*14), q0, self.__IK.q())
        result = Solve(self.__IK.prog(), q0)
        if result.is_success():
            return [True, result.GetSolution()]
        return [False, []]

    def solve_two_stage_ik(self, pose_list, q0=[], left_shoulder_rpy=None, right_shoulder_rpy=None, last_solution=None,
                           ctrl_arm_idx=ArmIdx.LEFT):
        """
        两阶段IK求解：先解算前4个自由度（肩部和肘部），再解算最后3个手腕自由度
        适用于单手7自由度手臂，且支持双手(BOTH)
        改动：肘部采用软约束（位置代价），不使用硬约束；不再使用基于索引的线性冻结，改用整体保持代价
        """
        if self.__is_roban_dof:
            return self.solve(pose_list, q0, left_shoulder_rpy, right_shoulder_rpy, last_solution)

        # 第一阶段：解算躯干和前4个自由度（肩部和肘部），不对手腕施加硬冻结，依靠近似保持代价抑制大幅变化
        stage1_IK = InverseKinematics(plant=self.__plant, with_joint_limits=True)
        snopt = SnoptSolver().solver_id()
        stage1_IK.prog().SetSolverOption(
            snopt, "Major Optimality Tolerance", self.__solver_tol
        )
        stage1_IK.prog().SetSolverOption(
            snopt, "Major Iterations Limit", self.__iterations_limit
        )

        # 添加躯干约束
        if not pose_list[0][0] is None:
            stage1_IK.AddOrientationConstraint(
                self.__plant.world_frame(),
                RotationMatrix(RollPitchYaw(pose_list[0][0])),
                self.__frames[0],
                RotationMatrix(RollPitchYaw(0, 0, 0)),
                self.__constraint_tol,
            )
        if not pose_list[0][1] is None:
            stage1_IK.AddPositionConstraint(
                frameB=self.__frames[0],
                p_BQ=np.zeros(3),
                frameA=self.__plant.world_frame(),
                p_AQ_lower=np.array(pose_list[0][1]) - self.__constraint_tol,
                p_AQ_upper=np.array(pose_list[0][1]) + self.__constraint_tol
            )

        # 为手与肘添加软约束（位置代价）
        # 左臂
        if (ctrl_arm_idx == ArmIdx.LEFT or ctrl_arm_idx == ArmIdx.BOTH):
            if not pose_list[1][1] is None:
                stage1_IK.AddPositionCost(
                    frameA=self.__plant.world_frame(),
                    p_AP=pose_list[1][1],
                    frameB=self.__one_stage_ee_frames[0],
                    p_BQ=np.zeros(3),
                    C=10.0 * np.eye(3),  # 10.0
                )
            if not pose_list[3] is None and not pose_list[3][1] is None:
                stage1_IK.AddPositionCost(
                    frameA=self.__plant.world_frame(),
                    p_AP=pose_list[3][1],
                    frameB=self.__frames[3],
                    p_BQ=np.zeros(3),
                    C=10.0 * np.eye(3), # 10.0
                )
        # 右臂
        if (ctrl_arm_idx == ArmIdx.RIGHT or ctrl_arm_idx == ArmIdx.BOTH):
            if not pose_list[2][1] is None:
                stage1_IK.AddPositionCost(
                    frameA=self.__plant.world_frame(),
                    p_AP=pose_list[2][1],
                    frameB=self.__one_stage_ee_frames[1],
                    p_BQ=np.zeros(3),
                    C=10.0 * np.eye(3),
                )
            if not pose_list[4] is None and not pose_list[4][1] is None:
                stage1_IK.AddPositionCost(
                    frameA=self.__plant.world_frame(),
                    p_AP=pose_list[4][1],
                    frameB=self.__frames[4],
                    p_BQ=np.zeros(3),
                    C=10.0 * np.eye(3),
                )

        # 添加前一次解的约束（平滑项）
        nq = self.__plant.num_positions()
        # 选择参考解：优先last_solution，其次q0
        use_last = last_solution is not None and len(last_solution) == nq and np.linalg.norm(last_solution) > 1e-6
        q_ref_stage1 = last_solution if use_last else (q0 if len(q0) == nq else np.zeros(nq))

        print(f'============= q_ref_stage1 {q_ref_stage1} ==========')
        # 阶段1权重：默认平滑0.2，被控制手的手腕权重更大（近似冻结）
        W_vec = np.array([0.2] * nq, dtype=float)
        left_wrist_idx = [4, 5, 6]
        right_wrist_idx = [11, 12, 13]
        if ctrl_arm_idx == ArmIdx.LEFT or ctrl_arm_idx == ArmIdx.BOTH:
            for i in left_wrist_idx:
                if i < nq:
                    W_vec[i] = 5.0
        if ctrl_arm_idx == ArmIdx.RIGHT or ctrl_arm_idx == ArmIdx.BOTH:
            for i in right_wrist_idx:
                if i < nq:
                    W_vec[i] = 5.0
        W_prev_solution = np.diag(W_vec)
        stage1_IK.prog().AddQuadraticErrorCost(W_prev_solution, q_ref_stage1, stage1_IK.q())

        # 求解第一阶段
        result1 = Solve(stage1_IK.prog(), q0)
        if not result1.is_success():
            print("[TorsoIK] 第一阶段IK求解失败")
            return [False, []]

        q_stage1 = result1.GetSolution()
        print(f"[TorsoIK] 第一阶段IK解算成功，q_stage1: {q_stage1}")
        # 第二阶段：解算手腕3个自由度（通过强保持代价抑制非手腕关节变化），为手添加位置+姿态约束，并再次约束躯干
        stage2_IK = InverseKinematics(plant=self.__plant, with_joint_limits=True)
        stage2_IK.prog().SetSolverOption(
            snopt, "Major Optimality Tolerance", self.__solver_tol
        )
        stage2_IK.prog().SetSolverOption(
            snopt, "Major Iterations Limit", self.__iterations_limit
        )

        # 躯干约束（与第一阶段一致）
        if not pose_list[0][0] is None:
            stage2_IK.AddOrientationConstraint(
                self.__plant.world_frame(),
                RotationMatrix(RollPitchYaw(pose_list[0][0])),
                self.__frames[0],
                RotationMatrix(RollPitchYaw(0, 0, 0)),
                self.__constraint_tol,
            )
        if not pose_list[0][1] is None:
            stage2_IK.AddPositionConstraint(
                frameB=self.__frames[0],
                p_BQ=np.zeros(3),
                frameA=self.__plant.world_frame(),
                p_AQ_lower=np.array(pose_list[0][1]) - self.__constraint_tol,
                p_AQ_upper=np.array(pose_list[0][1]) + self.__constraint_tol
            )

        # 左手末端位姿约束
        if (ctrl_arm_idx == ArmIdx.LEFT or ctrl_arm_idx == ArmIdx.BOTH):
            # if not pose_list[1][1] is None:
            #     # 使用位置代价代替硬约束
            #     stage2_IK.AddPositionCost(
            #         frameA=self.__plant.world_frame(),
            #         p_AP=pose_list[1][1],
            #         frameB=self.__frames[1],
            #         p_BQ=np.zeros(3),
            #         C=50.0 * np.eye(3),
            #     )
            # 添加姿态硬约束
            if not pose_list[1][0] is None:
                stage2_IK.AddOrientationConstraint(
                    self.__plant.world_frame(),
                    RotationMatrix(RollPitchYaw(pose_list[1][0])),
                    self.__frames[1],
                    RotationMatrix(RollPitchYaw(0, 0, 0.0)),
                    self.__constraint_tol,
                )

        # 右手末端位姿约束
        if (ctrl_arm_idx == ArmIdx.RIGHT or ctrl_arm_idx == ArmIdx.BOTH):
            # if not pose_list[2][1] is None:
            #     stage2_IK.AddPositionCost(
            #         frameA=self.__plant.world_frame(),
            #         p_AP=pose_list[2][1],
            #         frameB=self.__frames[2],
            #         p_BQ=np.zeros(3),
            #         C=50.0 * np.eye(3),
            #     )
            # 添加姿态硬约束
            if not pose_list[2][0] is None:
                stage2_IK.AddOrientationConstraint(
                    self.__plant.world_frame(),
                    RotationMatrix(RollPitchYaw(pose_list[2][0])),
                    self.__frames[2],
                    RotationMatrix(RollPitchYaw(0, 0, 0.0)),
                    self.__constraint_tol,
                )

        # 添加保持q_stage1的代价：除被控制手的手腕外，其余关节权重大，手腕权重小，促使仅手腕去满足末端姿态
        W_hold_vec = np.array([100.0] * nq, dtype=float)
        # 放松被控制手腕关节
        if ctrl_arm_idx == ArmIdx.LEFT or ctrl_arm_idx == ArmIdx.BOTH:
            for i in left_wrist_idx:
                if i < nq:
                    W_hold_vec[i] = 0.05  # 0.05
        if ctrl_arm_idx == ArmIdx.RIGHT or ctrl_arm_idx == ArmIdx.BOTH:
            for i in right_wrist_idx:
                if i < nq:
                    W_hold_vec[i] = 0.05
        W_hold = np.diag(W_hold_vec)
        stage2_IK.prog().AddQuadraticErrorCost(W_hold, q_stage1, stage2_IK.q())

        # 求解第二阶段
        result2 = Solve(stage2_IK.prog(), q_stage1)
        if not result2.is_success():
            print("[TorsoIK] 第二阶段IK求解失败")
            return [False, []]

        q_final = result2.GetSolution()
        q_final[:4] = q_stage1[:4]  # 保持前4个自由度不变
        return [True, q_final]


class ArmIk:
    """
    eef_z_bias: float, default 0.0(-> arm length=0.58m), the z-axis offset of the end-effector frame
        i.e. if u want to make length to 0.6m, set eef_z_bias to 0.02(0.58+0.02=0.6)
    """

    def __init__(
            self,
            model_file,
            end_frames_name,
            meshcat,
            constraint_tol=1e-4,
            solver_tol=1.0e-4,
            iterations_limit=1000,
            eef_z_bias=0.0,
            shoulder_frame_names=["l_arm_pitch", "r_arm_pitch"],
            ctrl_arm_idx=ArmIdx.LEFT,
            as_mc_ik=True,
    ):
        builder = DiagramBuilder()
        self.__plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
        parser = Parser(self.__plant)
        robot = parser.AddModelFromFile(model_file)
        # custom added
        eef_frame_name_list = ["frame_eef_left", "frame_eef_right"]
        self.shoulder_frame_names = shoulder_frame_names
        eef_frame_left = self.__plant.GetFrameByName(end_frames_name[1])
        eef_frame_right = self.__plant.GetFrameByName(end_frames_name[2])
        p = np.array([0, 0, eef_z_bias])
        self.frame_eef_left_custom = self.__plant.AddFrame(
            FixedOffsetFrame(eef_frame_name_list[0], eef_frame_left, RigidTransform(p))
        )
        self.frame_eef_right_custom = self.__plant.AddFrame(
            FixedOffsetFrame(eef_frame_name_list[1], eef_frame_right, RigidTransform(p))
        )
        # 假设躯干坐标系的名称为 "torso_frame"
        torso_frame = self.__plant.GetFrameByName(end_frames_name[0])

        # 焊接躯干坐标系到世界坐标系
        self.__plant.WeldFrames(self.__plant.world_frame(), torso_frame)
        self.__plant.Finalize()
        self.__meshcat = meshcat

        if self.__meshcat is not None:
            self.__visualizer = MeshcatVisualizer.AddToBuilder(
                builder, scene_graph, meshcat
            )
        self.__diagram = builder.Build()
        self.__diagram_context = self.__diagram.CreateDefaultContext()

        self.__plant_context = self.__plant.GetMyContextFromRoot(self.__diagram_context)
        self.__q0 = self.__plant.GetPositions(self.__plant_context)
        self.__v0 = self.__plant.GetVelocities(self.__plant_context)
        self.__r0 = self.__plant.CalcCenterOfMassPositionInWorld(self.__plant_context)
        self.__nq = self.__plant.num_positions()

        self.__base_link_name = end_frames_name[0]
        self.__left_eef_name = eef_frame_name_list[0]
        self.__right_eef_name = eef_frame_name_list[1]
        end_frames_name[1] = self.__left_eef_name
        end_frames_name[2] = self.__right_eef_name

        # 保存控制手臂索引
        self.__ctrl_arm_idx = ctrl_arm_idx

        self.__IK = TorsoIK(
            self.__plant,
            end_frames_name,
            constraint_tol,
            solver_tol,
            iterations_limit=iterations_limit,
            ctrl_arm_idx=ctrl_arm_idx,
            as_mc_ik=as_mc_ik,
        )
        # 添加记录上一次求得的逆解的属性，初始化为全零向量
        self.__last_solution = np.zeros(self.__plant.num_positions())
        # 添加是否使用两阶段IK的标志
        self.__use_two_stage_ik = True
        print("initializing arm ik")

    def set_as_mc_ik(self, as_mc_ik):
        self.__IK.set_as_mc_ik(as_mc_ik)
        # print(f"[ArmIk] set as_mc_ik to {as_mc_ik}")

    def set_use_two_stage_ik(self, use_two_stage_ik):
        """
        设置是否使用两阶段IK求解
        """
        self.__use_two_stage_ik = use_two_stage_ik
        print(f"[ArmIk] set use_two_stage_ik to {use_two_stage_ik}")

    def get_use_two_stage_ik(self):
        """
        获取是否使用两阶段IK求解
        """
        return self.__use_two_stage_ik

    def type(self):
        return IkTypeIdx.TorsoIK

    def q0(self):
        return self.__q0

    def init_state(self, torso_yaw_deg, torso_height):
        self.__torso_yaw_rad = math.radians(torso_yaw_deg)
        self.__torso_height = torso_height
        self.__q0[6] = torso_height

    def computeIK(self, q0, l_hand_pose, r_hand_pose, l_hand_RPY=None, r_hand_RPY=None, l_elbow_pos=None,
                  r_elbow_pos=None, left_shoulder_rpy=None, right_shoulder_rpy=None):
        torsoR = [0.0, self.__torso_yaw_rad, 0.0]
        r = [0.0, 0.0, self.__torso_height]
        # print(f"l_elbow_pos: {l_elbow_pos}, r_elbow_pos: {r_elbow_pos}")
        pose_list = [
            [torsoR, r],
            [l_hand_RPY, l_hand_pose],
            [r_hand_RPY, r_hand_pose],
            [None, l_elbow_pos],
            [None, r_elbow_pos],
        ]

        # 根据设置选择IK方式
        if self.__use_two_stage_ik:
            # 使用两阶段IK
            is_success, q = self.__IK.solve_two_stage_ik(
                pose_list,
                q0=q0,
                left_shoulder_rpy=left_shoulder_rpy,
                right_shoulder_rpy=right_shoulder_rpy,
                last_solution=self.__last_solution,
                ctrl_arm_idx=self.__ctrl_arm_idx
            )
        else:
            # 使用标准IK
            is_success, q = self.__IK.solve(pose_list, q0=q0, left_shoulder_rpy=left_shoulder_rpy,
                                            right_shoulder_rpy=right_shoulder_rpy, last_solution=self.__last_solution)

        if not is_success:
            # print(f"pose: {pose_list[0][0]}, {pose_list[0][1]}")
            # print(f"lhand: {pose_list[1][0]}, {pose_list[1][1]}")
            # print(f"rhand: {pose_list[2][0]}, {pose_list[2][1]}")
            # raise RuntimeError("Failed to IK0!")
            return None
        else:
            self.__last_solution = q
            return q

    def computeTwoStageIK(self, q0, l_hand_pose, r_hand_pose, l_hand_RPY=None, r_hand_RPY=None, l_elbow_pos=None,
                          r_elbow_pos=None, left_shoulder_rpy=None, right_shoulder_rpy=None, ctrl_arm_idx=ArmIdx.LEFT):
        """
        使用两阶段IK求解：先解算前4个自由度（肩部和肘部），再解算最后3个手腕自由度
        适用于单手7自由度手臂
        """
        torsoR = [0.0, self.__torso_yaw_rad, 0.0]
        r = [0.0, 0.0, self.__torso_height]
        pose_list = [
            [torsoR, r],
            [l_hand_RPY, l_hand_pose],
            [r_hand_RPY, r_hand_pose],
            [None, l_elbow_pos],
            [None, r_elbow_pos],
        ]

        is_success, q = self.__IK.solve_two_stage_ik(
            pose_list,
            q0=q0,
            left_shoulder_rpy=left_shoulder_rpy,
            right_shoulder_rpy=right_shoulder_rpy,
            last_solution=self.__last_solution,
            ctrl_arm_idx=ctrl_arm_idx
        )

        if not is_success:
            print(f"[ArmIk] 两阶段IK求解失败")
            return None
        else:
            self.__last_solution = q
            return q

    def start_recording(self):
        if self.__meshcat is None:
            return
        self.__visualizer.StartRecording()

    def stop_andpublish_recording(self):
        if self.__meshcat is None:
            return
        self.__visualizer.StopRecording()
        self.__visualizer.PublishRecording()

    def visualize_animation(self, q_list, start_time=0.0, duration=1.1):
        t_sol = np.arange(start_time, start_time + duration, 1)
        q_sol = np.array(q_list).T
        # print(f"q_sol: {q_sol.shape}, t_sol: {t_sol.shape}")
        q_pp = PiecewisePolynomial.FirstOrderHold(t_sol, q_sol)
        t0 = t_sol[0]
        tf = t_sol[-1]
        t = t0
        # self.__visualizer.StartRecording()
        while t < tf:
            q = q_pp.value(t)
            self.__plant.SetPositions(self.__plant_context, q)
            self.__diagram_context.SetTime(t)
            self.__diagram.ForcedPublish(self.__diagram_context)
            t += 0.01
        # self.__visualizer.StopRecording()
        # self.__visualizer.PublishRecording()
        # while True:
        time.sleep(0.1)

    def left_hand_jacobian(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        J_hand_in_world = self.__plant.CalcJacobianSpatialVelocity(
            self.__plant_context,
            JacobianWrtVariable.kV,
            self.__plant.GetFrameByName(self.__left_eef_name),
            [0, 0, 0],
            self.__plant.world_frame(),
            self.__plant.world_frame(),
        )
        # print(f'========= J_hand_in_world: {J_hand_in_world} =========')
        return J_hand_in_world[:, 0:7]

    def joint_limit_avoidance(self, joint_angles, joint_limits):
        # Define a simple potential function to avoid joint limits
        q_min = joint_limits[0, :]
        q_max = joint_limits[1, :]

        # Compute the potential function gradient (first derivative)
        grad = 2 * (joint_angles - q_min) + 2 * (joint_angles - q_max)

        return grad

    def null_space_projection(self, jacobian, joint_angles, joint_limits):
        # Compute the pseudoinverse of the Jacobian
        J_pseudo_inv = np.linalg.pinv(jacobian)

        # Compute the null space projection
        null_space = np.eye(jacobian.shape[1]) - J_pseudo_inv @ jacobian

        # Compute the gradient of the joint limit potential
        joint_limit_grad = self.joint_limit_avoidance(joint_angles, joint_limits)


        print(f'joint_limit_grad {joint_limit_grad}')
        # Combine task velocity and null space projection to avoid joint limits
        null_joint_velocity = null_space @ joint_limit_grad[:7]

        return null_joint_velocity

    def left_hand_tool_space_step(self,
                                  current_q,
                                  #   target_q,
                                  target_pos_W,
                                  target_rpy_W,
                                  Kp_pos=2.0,
                                  Kp_rot=2.0,
                                  damping=1e-4,
                                  dt=0.01):
        """
        对左臂做一小步 tool-space 控制：
        给定当前关节角 q 和目标末端位姿 (target_pos_W, target_rpy_W)，
        计算一小步积分后的 q_next，以及当前误差。

        参数
        ----
        q : np.ndarray, shape (nq,)
            当前整机关节角（和 plant.num_positions 一致）
        target_pos_W : array-like, shape (3,)
            目标末端位置（在 world/torso 坐标系下）
        target_rpy_W : array-like, shape (3,)
            目标末端 RPY（绕 world/torso 轴）
        Kp_pos : float
            位置 P 增益
        Kp_rot : float
            姿态 P 增益
        damping : float
            阻尼伪逆里的 λ
        dt : float
            单步积分时间

        返回
        ----
        q_next : np.ndarray, shape (nq,)
            单步更新后的关节角
        info : dict
            包含当前误差等调试信息
        """
        # q = target_q
        # q = np.asarray(q).copy()
        nq = self.__plant.num_positions()
        # assert q.shape[0] == nq

        # 1. 设定当前关节到 plant context
        self.__plant.SetPositions(self.__plant_context, current_q)

        # 2. 当前末端位姿（在 world 中）
        frame_E = self.__plant.GetFrameByName(self.__left_eef_name)
        X_WE = frame_E.CalcPoseInWorld(self.__plant_context)
        p_W_current = X_WE.translation()
        R_WE = X_WE.rotation()

        # 3. 目标末端位姿
        target_pos_W = np.asarray(target_pos_W).reshape(3, )
        target_rpy_W = np.asarray(target_rpy_W).reshape(3, )
        R_WE_des = RotationMatrix(RollPitchYaw(*target_rpy_W))
        p_W_des = target_pos_W

        # 4. 位置误差 (world)
        p_err_W = p_W_des - p_W_current  # 3

        print(f'========= p_W_des: {p_W_des}, p_W_current: {p_W_current}, p_err_W: {p_err_W} =========')
        print(f'========= target_rpy_W: {target_rpy_W} =========, R_WE_des: {R_WE_des} =========')

        # 5. 姿态误差：R_err = R_des * R_current^T
        R_err = R_WE_des @ R_WE.transpose()
        aa = R_err.ToAngleAxis()
        w_err_W = aa.angle() * aa.axis()  # 3

        # 6. 期望末端空间速度 V_des = [w_des; v_des]
        w_des = Kp_rot * w_err_W
        v_des_trans = Kp_pos * p_err_W
        V_des = np.hstack([w_des, v_des_trans])  # (6,)

        # 7.
        J_hand = self.left_hand_jacobian(current_q)  # (6, 7)

        joint_limits_dict = {
            'joint_1': (-3.14159265358979, 1.5707963267949),
            'joint_2': (-0.349065850398866, 2.0943951023932),
            'joint_3': (-1.5707963267949, 1.5707963267949),
            'joint_4': (-2.61799387799149, 0.0),
            'joint_5': (-1.5707963267949, 1.5707963267949),
            'joint_6': (-1.30899693899575, 0.698131700797732),
            'joint_7': (-0.698131700797732, 0.698131700797732),
        }

        # set joint limits here
        joint_limits = np.array(
            [[0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
             [7., 7., 7., 7., 7., 7., 7., 7., 7., 7., 7., 7., 7., 7.]]
        )

        for i in range(7):
            joint_name = f'joint_{i+1}'
            # if joint_name in joint_limits_dict:
            joint_limits[0, i] = joint_limits_dict[joint_name][0]
            joint_limits[1, i] = joint_limits_dict[joint_name][1]

        # 8. 阻尼伪逆：v_arm = J^T (J J^T + λI)^-1 V_des
        J = J_hand
        Jt = J.T  # (7, 6)
        JJt = J @ Jt  # (6, 6)
        lamI = damping * np.eye(6)
        v_arm = (
                Jt @ np.linalg.solve(JJt + lamI, V_des)
                + self.null_space_projection(J_hand, current_q, joint_limits)  # (7,)
        )

        # 9. 构造整机关节速度向量（只动左臂 7 个关节，其他为 0）
        v_full = np.zeros(nq)
        # 你的 left_hand_jacobian 里用的是 J[:, 6:13]
        v_full[:7] = v_arm

        # 10. 简单欧拉积分得到下一步关节角
        q_next = current_q + dt * v_full

        print(f'========= v_full: {v_full} =========')
        info = {
            "p_err_norm": np.linalg.norm(p_err_W),
            "w_err_norm": np.linalg.norm(w_err_W),
            "p_err": p_err_W,
            "w_err": w_err_W,
            "V_des": V_des,
            "v_arm": v_arm,
            "v_full": v_full,
        }

        print(f'=========== current_q: {current_q} ==========')
        print(f'=========== q_next: {q_next} ==========')
        return q_next, info

    def left_hand_tool_space_step_old(self,
                                  current_q,
                                  #   target_q,
                                  target_pos_W,
                                  target_rpy_W,
                                  Kp_pos=2.0,
                                  Kp_rot=2.0,
                                  damping=1e-4,
                                  dt=0.01):
        """
        对左臂做一小步 tool-space 控制：
        给定当前关节角 q 和目标末端位姿 (target_pos_W, target_rpy_W)，
        计算一小步积分后的 q_next，以及当前误差。

        参数
        ----
        q : np.ndarray, shape (nq,)
            当前整机关节角（和 plant.num_positions 一致）
        target_pos_W : array-like, shape (3,)
            目标末端位置（在 world/torso 坐标系下）
        target_rpy_W : array-like, shape (3,)
            目标末端 RPY（绕 world/torso 轴）
        Kp_pos : float
            位置 P 增益
        Kp_rot : float
            姿态 P 增益
        damping : float
            阻尼伪逆里的 λ
        dt : float
            单步积分时间

        返回
        ----
        q_next : np.ndarray, shape (nq,)
            单步更新后的关节角
        info : dict
            包含当前误差等调试信息
        """
        # q = target_q
        # q = np.asarray(q).copy()
        nq = self.__plant.num_positions()
        # assert q.shape[0] == nq

        # 1. 设定当前关节到 plant context
        self.__plant.SetPositions(self.__plant_context, current_q)

        # 2. 当前末端位姿（在 world 中）
        frame_E = self.__plant.GetFrameByName(self.__left_eef_name)
        X_WE = frame_E.CalcPoseInWorld(self.__plant_context)
        p_W_current = X_WE.translation()
        R_WE = X_WE.rotation()

        # 3. 目标末端位姿
        target_pos_W = np.asarray(target_pos_W).reshape(3, )
        target_rpy_W = np.asarray(target_rpy_W).reshape(3, )
        R_WE_des = RotationMatrix(RollPitchYaw(*target_rpy_W))
        p_W_des = target_pos_W

        # 4. 位置误差 (world)
        p_err_W = p_W_des - p_W_current  # 3

        p_err_W = np.clip(p_err_W, -0.1, 0.1)  # 限幅，防止过大震荡

        print(f'========= p_W_des: {p_W_des}, p_W_current: {p_W_current}, p_err_W: {p_err_W} =========')
        print(f'========= target_rpy_W: {target_rpy_W} =========, R_WE_des: {R_WE_des} =========')

        # 5. 姿态误差：R_err = R_des * R_current^T
        R_err = R_WE_des @ R_WE.transpose()
        aa = R_err.ToAngleAxis()
        w_err_W = aa.angle() * aa.axis()  # 3

        # 6. 期望末端空间速度 V_des = [w_des; v_des]
        w_des = Kp_rot * w_err_W
        v_des_trans = Kp_pos * p_err_W
        V_des = np.hstack([w_des, v_des_trans])  # (6,)

        # 7. 左手雅可比（只取左臂 7 个关节）
        J_hand = self.left_hand_jacobian(current_q)  # (6, 7)

        # 8. 阻尼伪逆：v_arm = J^T (J J^T + λI)^-1 V_des
        J = J_hand
        Jt = J.T  # (7, 6)
        JJt = J @ Jt  # (6, 6)
        lamI = damping * np.eye(6)
        v_arm = Jt @ np.linalg.solve(JJt + lamI, V_des)  # (7,)

        # 9. 构造整机关节速度向量（只动左臂 7 个关节，其他为 0）
        v_full = np.zeros(nq)
        # 你的 left_hand_jacobian 里用的是 J[:, 6:13]
        v_full[:7] = v_arm

        # 10. 简单欧拉积分得到下一步关节角
        q_next = current_q + dt * v_full

        print(f'========= v_full: {v_full} =========')
        info = {
            "p_err_norm": np.linalg.norm(p_err_W),
            "w_err_norm": np.linalg.norm(w_err_W),
            "p_err": p_err_W,
            "w_err": w_err_W,
            "V_des": V_des,
            "v_arm": v_arm,
            "v_full": v_full,
        }

        print(f'=========== current_q: {current_q} ==========')
        print(f'=========== q_next: {q_next} ==========')
        return q_next, info

    def right_hand_jacobian(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        J_hand_in_world = self.__plant.CalcJacobianSpatialVelocity(
            self.__plant_context,
            JacobianWrtVariable.kV,
            self.__plant.GetFrameByName(self.__right_eef_name),
            [0, 0, 0],
            self.__plant.world_frame(),
            self.__plant.world_frame(),
        )
        return J_hand_in_world[:, -7:]

    def left_hand_pose(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        l_hand_in_base = self.__plant.GetFrameByName(self.__left_eef_name).CalcPose(
            self.__plant_context, self.__plant.GetFrameByName(self.__base_link_name)
        )
        # print("left hand position in base:", l_hand_in_base.translation())
        return (l_hand_in_base.translation(), l_hand_in_base.rotation().ToRollPitchYaw().vector())

    def right_hand_pose(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        r_hand_in_base = self.__plant.GetFrameByName(self.__right_eef_name).CalcPose(
            self.__plant_context, self.__plant.GetFrameByName(self.__base_link_name)
        )
        # print("right hand position in base:", r_hand_in_base.translation())
        return (r_hand_in_base.translation(), r_hand_in_base.rotation().ToRollPitchYaw().vector())

    def get_arm_length(self):
        shoulder_frame_left = self.__plant.GetFrameByName(self.shoulder_frame_names[0])
        X_shoulder_left = shoulder_frame_left.CalcPoseInWorld(self.__plant_context)
        X_eef_left = self.__plant.GetFrameByName(self.__left_eef_name).CalcPoseInWorld(
            self.__plant_context
        )
        dis = X_shoulder_left.translation() - X_eef_left.translation()
        # print(f"left dis: {dis}")
        length_left = np.linalg.norm(dis)

        shoulder_frame_right = self.__plant.GetFrameByName(self.shoulder_frame_names[1])
        X_shoulder_right = shoulder_frame_right.CalcPoseInWorld(self.__plant_context)
        X_eef_right = self.__plant.GetFrameByName(
            self.__right_eef_name
        ).CalcPoseInWorld(self.__plant_context)
        dis = X_shoulder_right.translation() - X_eef_right.translation()
        # print(f"right dis: {dis}")
        length_right = np.linalg.norm(dis)

        return length_left, length_right

    def get_two_frame_dis_vec(self, frame_a_name, frame_b_name):
        frame_a = self.__plant.GetFrameByName(frame_a_name)
        X_a = frame_a.CalcPoseInWorld(self.__plant_context)
        X_b = self.__plant.GetFrameByName(frame_b_name).CalcPoseInWorld(
            self.__plant_context
        )
        dis = X_a.translation() - X_b.translation()
        return dis

    def get_two_frame_dis(self, frame_a_name, frame_b_name):
        dis = self.get_two_frame_dis_vec(frame_a_name, frame_b_name)
        # print(f"left dis: {dis}")
        length = np.linalg.norm(dis)

        return length