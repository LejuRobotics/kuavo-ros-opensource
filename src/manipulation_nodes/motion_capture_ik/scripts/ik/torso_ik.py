import sys
import os
import time
import math

import numpy as np
import pydrake

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
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
    InitializeAutoDiff,
    JacobianWrtVariable,
    ExtractGradient,
    ExtractValue,
    ComPositionConstraint,
    CentroidalMomentumConstraint,
)
from pydrake.all import StartMeshcat, AddMultibodyPlantSceneGraph, MeshcatVisualizer

# from scipy.spatial.transform import Rotation as R
import numpy as np
from functools import partial
from ik.urdf_utils import prepare_model_file

current_dir = os.path.dirname(os.path.abspath(__file__))
tools_dir = os.path.abspath(os.path.join(current_dir, '..', 'tools'))
sys.path.append(tools_dir)
from tools.utils import get_package_path, ArmIdx, IkTypeIdx
from drake_trans import quaternion_to_RPY, rpy_to_matrix


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
            self.__one_stage_ee_frames = [self.__plant.GetFrameByName("zarm_l6_link"), self.__plant.GetFrameByName("zarm_r6_link")]
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
                if not self.__is_roban_dof or i == 0: # 只有单手大于6自由度才进行姿态约束，躯干一直进行姿态约束
                    self.__IK.AddOrientationConstraint(
                        self.__plant.world_frame(),
                        RotationMatrix(RollPitchYaw(pose_list[i][0])),
                        frame,
                        RotationMatrix(RollPitchYaw(0, 0, 0)),
                        self.__constraint_tol,
                    )
            if not pose_list[i][1] is None:
                # print(f"pos: {i}")
                if self.__as_mc_ik and i != 0:
                    cost_matrix = 10 * np.eye(3)
                    if i > 2: # 3,4 -> elbow
                        cost_matrix = (10 * np.eye(3)) if self.__is_roban_dof else (1 * np.eye(3))
                    self.__IK.AddPositionCost(
                        frameA=self.__plant.world_frame(),
                        p_AP=pose_list[i][1],
                        frameB=frame,
                        p_BQ=np.zeros(3),
                        C=cost_matrix,
                    )
                if(not self.__as_mc_ik and i<=2) or (self.__as_mc_ik and i==0): # torso
                    self.__IK.AddPositionConstraint(frameB=frame,
                                                    p_BQ=np.zeros(3),
                                                    frameA=self.__plant.world_frame(),
                                                    p_AQ_lower=np.array(pose_list[i][1])-self.__constraint_tol,
                                                    p_AQ_upper=np.array(pose_list[i][1])+self.__constraint_tol)

        q_normal = np.array([-0.3928, 0.2608, 0.0143, -0.3927, 0.2970, -0.8364, 0.0589])
        q_normal_right = np.array(
            [0.3928, 0.2608, 0.0143, -0.3927, 0.2970, -0.8364, 0.0589]
        )
        W = np.diag([0.1])
        bias = 5.0 * np.pi / 180.0
        # if left_shoulder_rpy is not None: # left shoulder
        #     # self.__IK.prog().AddQuadraticErrorCost(W, np.array([[left_shoulder_rpy[1]]]), np.array([[self.__IK.q()[7+0]]])) # joint-1
        #     # self.__IK.prog().AddLinearConstraint(self.__IK.q()[7+0], lb=left_shoulder_rpy[1]-bias, ub=left_shoulder_rpy[1]+bias) # joint-1
        #     # self.__IK.prog().AddQuadraticErrorCost(W, np.array([[left_shoulder_rpy[0]]]), np.array([[self.__IK.q()[7+1]]])) # joint-2
        #     # self.__IK.prog().AddLinearConstraint(self.__IK.q()[7+1], lb=left_shoulder_rpy[0]-bias, ub=left_shoulder_rpy[0]+bias) # joint-2
        #     # self.__IK.prog().AddQuadraticErrorCost(W, np.array([[left_shoulder_rpy[2]]]), np.array([[self.__IK.q()[7+2]]])) # joint-3
        #     self.__IK.prog().AddLinearConstraint(self.__IK.q()[7+2], lb=left_shoulder_rpy[2]-bias, ub=left_shoulder_rpy[2]+bias) # joint-3
        # if right_shoulder_rpy is not None:
        #     # self.__IK.prog().AddQuadraticErrorCost(W, np.array([[right_shoulder_rpy[1]]]), np.array([[self.__IK.q()[14+0]]]))
        #     # self.__IK.prog().AddLinearConstraint(self.__IK.q()[14+0], lb=right_shoulder_rpy[1]-bias, ub=right_shoulder_rpy[1]+bias)
        #     # self.__IK.prog().AddQuadraticErrorCost(W, np.array([[right_shoulder_rpy[0]]]), np.array([[self.__IK.q()[14+1]]]))
        #     # self.__IK.prog().AddLinearConstraint(self.__IK.q()[14+1], lb=right_shoulder_rpy[0]-bias, ub=right_shoulder_rpy[0]+bias)
        #     # self.__IK.prog().AddQuadraticErrorCost(W, np.array([[right_shoulder_rpy[2]]]), np.array([[self.__IK.q()[14+2]]]))
        #     self.__IK.prog().AddLinearConstraint(self.__IK.q()[14+2], lb=right_shoulder_rpy[2]-bias, ub=right_shoulder_rpy[2]+bias)

        if last_solution is not None:
            W_vec = np.array([0.1] * self.__plant.num_positions())
            if self.__is_roban_dof: # yaw调整可以很快
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

    def solve_two_stage_ik(self, pose_list, q0=[], left_shoulder_rpy=None, right_shoulder_rpy=None, last_solution=None, ctrl_arm_idx=ArmIdx.LEFT):
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
                p_AQ_lower=np.array(pose_list[0][1])-self.__constraint_tol,
                p_AQ_upper=np.array(pose_list[0][1])+self.__constraint_tol
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
                    C=10.0 * np.eye(3),
                )
            if not pose_list[3] is None and not pose_list[3][1] is None:
                stage1_IK.AddPositionCost(
                    frameA=self.__plant.world_frame(),
                    p_AP=pose_list[3][1],
                    frameB=self.__frames[3],
                    p_BQ=np.zeros(3),
                    C=10.0 * np.eye(3),
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
                p_AQ_lower=np.array(pose_list[0][1])-self.__constraint_tol,
                p_AQ_upper=np.array(pose_list[0][1])+self.__constraint_tol
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
                    W_hold_vec[i] = 0.05
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
        
        # 用第一阶段的前4个关节位置覆盖最终结果，确保肩部和肘部位置不变
        # 左臂: 关节索引 0-3 (相对于左臂起始位置)
        # 右臂: 关节索引 7-10 (相对于右臂起始位置)
        if ctrl_arm_idx == ArmIdx.LEFT or ctrl_arm_idx == ArmIdx.BOTH:
            # 左臂前4个关节 (假设从索引0开始)
            q_final[0:4] = q_stage1[0:4]
        if ctrl_arm_idx == ArmIdx.RIGHT or ctrl_arm_idx == ArmIdx.BOTH:
            # 右臂前4个关节 (假设从索引7开始)
            q_final[7:11] = q_stage1[7:11]
        
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
        # 添加kuavo_assets包路径，用于解析URDF中的package://kuavo_assets/...路径
        kuavo_assets_path = get_package_path("kuavo_assets")
        print(f"kuavo_assets_path: {kuavo_assets_path}")
        if kuavo_assets_path:
            parser.package_map().Add("kuavo_assets", kuavo_assets_path)

        # Prepare model file (possibly create a temporary URDF with some joints set to fixed)
        joints_to_force_fixed = ["waist_yaw_joint", "waist_pitch_joint", "leg_joint", "knee_joint"]
        model_to_load, temp_path = prepare_model_file(model_file, joints_to_force_fixed)

        robot = parser.AddModelFromFile(model_to_load)
        print(f"[ArmIk] Loaded model: {model_to_load}")

        # cleanup temp file if created
        if temp_path is not None:
            try:
                os.remove(temp_path)
            except Exception:
                pass

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
        # 绝对式 IK：在 Finalize 前收紧左右肩 pitch(zarm_l1/r1) upper，不改磁盘 URDF。
        # 用于缓解肩关节过大上抬导致卡住（kuavodevlab#3448）。
        self._apply_shoulder_pitch_upper_limit()
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

        # 将解析出的固定关节（索引和值）传给 TorsoIK
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
        self.__use_two_stage_ik = False
        print("initializing arm ik")

    def _apply_shoulder_pitch_upper_limit(self):
        """Clamp zarm_l1/r1 upper via rosparam without modifying URDF on disk.

        Params:
          /quest3/limit_shoulder_pitch_upper (bool, default True)
          /quest3/shoulder_pitch_upper_limit (float, default 0.3)
        """
        enable = True
        upper_limit = 0.3
        try:
            import rospy
            enable = bool(rospy.get_param("/quest3/limit_shoulder_pitch_upper", True))
            upper_limit = float(rospy.get_param("/quest3/shoulder_pitch_upper_limit", 0.3))
        except Exception:
            pass

        if not enable:
            print("[ArmIk] shoulder pitch upper limit override disabled")
            return

        for joint_name in ("zarm_l1_joint", "zarm_r1_joint"):
            try:
                joint = self.__plant.GetMutableJointByName(joint_name)
                lower = np.array(joint.position_lower_limits(), dtype=float).reshape(-1)
                upper = np.array(joint.position_upper_limits(), dtype=float).reshape(-1)
                old_upper = float(upper[0])
                upper[0] = min(old_upper, upper_limit)
                joint.set_position_limits(lower, upper)
                print(
                    f"[ArmIk] {joint_name} upper limit: {old_upper:.4f} -> {float(upper[0]):.4f} "
                    f"(rosparam /quest3/shoulder_pitch_upper_limit={upper_limit})"
                )
            except Exception as e:
                print(f"[ArmIk] skip shoulder pitch upper limit for {joint_name}: {e}")

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

    def reset_last_solution(self, q=None):
        """
        重置上一次的IK解
        
        Args:
            q: 如果提供，设置为新的last_solution；否则重置为q0()
        """
        if q is not None and len(q) == self.__plant.num_positions():
            self.__last_solution = np.array(q).copy()
        else:
            self.__last_solution = self.__q0.copy()
        print(f"[ArmIk] Reset last_solution to {'provided value' if q is not None else 'q0'}")

    def init_state(self, torso_yaw_deg, torso_height):
        self.__torso_yaw_rad = math.radians(torso_yaw_deg)
        self.__torso_height = torso_height
        self.__q0[6] = torso_height

    def computeIK(self, q0, l_hand_pose, r_hand_pose, l_hand_RPY=None, r_hand_RPY=None, l_elbow_pos=None, r_elbow_pos=None, left_shoulder_rpy=None, right_shoulder_rpy=None):
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
            is_success, q = self.__IK.solve(pose_list, q0=q0, left_shoulder_rpy=left_shoulder_rpy, right_shoulder_rpy=right_shoulder_rpy, last_solution=self.__last_solution)
        
        if not is_success:
            # print(f"pose: {pose_list[0][0]}, {pose_list[0][1]}")
            # print(f"lhand: {pose_list[1][0]}, {pose_list[1][1]}")
            # print(f"rhand: {pose_list[2][0]}, {pose_list[2][1]}")
            # raise RuntimeError("Failed to IK0!")
            return None
        else:
            self.__last_solution = q
            return q

    def computeTwoStageIK(self, q0, l_hand_pose, r_hand_pose, l_hand_RPY=None, r_hand_RPY=None, l_elbow_pos=None, r_elbow_pos=None, left_shoulder_rpy=None, right_shoulder_rpy=None, ctrl_arm_idx=ArmIdx.LEFT):
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
        return J_hand_in_world[:, 6:13]

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

if __name__ == "__main__":
    # test = np.load("./rosbag_joint.npy")  # drake RPY版本
    # test = np.load("./rosbag_s.npy") # 四元数版本（x,y,z,w）
    np.set_printoptions(linewidth=240)
    np.set_printoptions(threshold=2000)
    np.set_printoptions(precision=4)
    np.set_printoptions(suppress=True)

    meshcat = StartMeshcat()
    version = 4
    eef_z_bias = -0.0
    kuavo_pkg_path = get_package_path("motion_capture_ik")
    print(f"kuavo_pkg_path: {kuavo_pkg_path}")
    kuavo_assests_path = get_package_path("kuavo_assets")
    robot_version = os.environ.get('ROBOT_VERSION', '40')
    model_file = kuavo_assests_path + f"/models/biped_s{robot_version}/urdf/drake/biped_v3_arm.urdf"
    
    # model_file = kuavo_pkg_path + "/models/biped_gen4.0/urdf/biped_v3_arm.urdf"
    end_frames_name = ["torso", "l_hand_roll", "r_hand_roll", "l_forearm_pitch", "r_forearm_pitch"]
    if version == 3:
        # model_file = kuavo_pkg_path + "/models/biped_gen3.4/urdf/biped_v3_arm.urdf"
        eef_z_bias = -0.098
        end_frames_name = ["torso", "l_hand_pitch", "r_hand_pitch"]

    arm_ik = ArmIk(model_file, end_frames_name, meshcat, 1e-3, 1e-3, 1000, eef_z_bias=eef_z_bias)
    # end_frames_name = [base_link_name, "l_hand_eef", "r_hand_eef"]

    # arm_ik = ArmIk(model_file, end_frames_name, meshcat, 1e-3, 1e-3, 1000, -0.098)
    torso_yaw_deg = 0.0
    torso_height = 0.0
    arm_ik.init_state(torso_yaw_deg, torso_height)
    arm_length_l, arm_length_r = arm_ik.get_arm_length()
    print(f"Arm length left: {arm_length_l:.3f} m, right: {arm_length_r:.3f} m")
    q0 = arm_ik.q0()
    q_list = [q0]
    last_q = q0
    arm_ik.start_recording()
    t = 0.0
    l_pose = arm_ik.left_hand_pose(q0)
    print(
        f"left_hand_pose: {l_pose[0]}, {l_pose[1]}"
    )

    for i in range(5):
        l_hand_pose = np.array([0.15, 0.21, -0.121])
        l_hand_RPY = np.array([0.4092, -0.3443, 0.5193])
        l_elbow_pos = np.array([0.15, 0.21, 0.121])
        r_elbow_pos = np.array([0.15, -0.21, 0.121])
        l_hand_RPY = None

        r_hand_RPY = None
        r_hand_pose = None  # [x, y, z]
        time_0 = time.time()
        q = arm_ik.computeIK(q0, l_hand_pose, r_hand_pose, l_hand_RPY, r_hand_RPY, l_elbow_pos, r_elbow_pos)
        time_cost = time.time() - time_0
        print(f"i: {i}, time cost: {1e3*time_cost:.3f} ms")
        if q is not None:
            q_list.append(q)
            q0 = q
            # animate trajectory
            arm_ik.visualize_animation([last_q, q], t)
            last_q = q
            t = t + 1.0
        else:
            print(f"Failed to IK in step {i}!")
        time.sleep(0.001)
    pos = arm_ik.left_hand_pose(last_q)
    print(
        f"end left_hand_pose: {pos[0]}, {pos[1]}"
    )
    arm_ik.stop_andpublish_recording()
    print("Program end, Press Ctrl + C to exit.")
    while True:
        time.sleep(0.01)
