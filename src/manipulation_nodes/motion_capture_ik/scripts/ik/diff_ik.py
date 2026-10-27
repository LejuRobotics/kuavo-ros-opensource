import time
import math

import numpy as np
from IPython.display import clear_output
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    JacobianWrtVariable,
    MathematicalProgram,
    MeshcatVisualizer,
    PiecewisePolynomial,
    FixedOffsetFrame,
    PiecewisePose,
    RigidTransform,
    RotationMatrix,
    Solve,
    StartMeshcat,
    Quaternion,
    RollPitchYaw,
    Parser,
)
import matplotlib.pyplot as plt

from scipy.spatial.transform import Rotation as R
import rospy
from sensor_msgs.msg import JointState
import os, sys

current_dir = os.path.dirname(os.path.abspath(__file__))
tools_dir = os.path.abspath(os.path.join(current_dir, '..', 'tools'))
sys.path.append(tools_dir)
from tools.utils import get_package_path, ArmIdx, IkTypeIdx, rotation_matrix_diff_in_angle_axis
from drake_parser_compat import add_urdf_model
from drake_trans import quaternion_to_matrix

class DiffIK:
    def __init__(self, model_file, end_frames_name, 
        arm_idx=ArmIdx.LEFT, 
        q_limit=None, 
        meshcat=None,          
        eef_z_bias=0.0, 
        shoulder_frame_names=["l_arm_pitch", "r_arm_pitch"],
        ):
        builder = DiagramBuilder()
        self.__plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
        parser = Parser(self.__plant)
        add_urdf_model(parser, model_file)
        # custom added
        eef_frame_name_list = ["frame_eef_left", "frame_eef_right"]
        self.shoulder_frame_names = shoulder_frame_names
        eef_frame_left = self.__plant.GetFrameByName(end_frames_name[1])
        eef_frame_right = self.__plant.GetFrameByName(end_frames_name[2])
        p = np.array([0, 0, -(0.2254 + eef_z_bias)])
        frame_eef_left_custom = self.__plant.AddFrame(
            FixedOffsetFrame(eef_frame_name_list[0], eef_frame_left, RigidTransform(p))
        )
        frame_eef_right_custom = self.__plant.AddFrame(
            FixedOffsetFrame(eef_frame_name_list[1], eef_frame_right, RigidTransform(p))
        )
        self.__plant.Finalize()
        self.__meshcat = meshcat
        self.__arm_idx = arm_idx

        if self.__meshcat is not None:
            self.__visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
        self.__diagram = builder.Build()
        self.__diagram_context = self.__diagram.CreateDefaultContext()

        self.__plant_context = self.__plant.GetMyContextFromRoot(self.__diagram_context)
        self.__q0 = self.__plant.GetPositions(self.__plant_context)
        self.__v0 = self.__plant.GetVelocities(self.__plant_context)
        self.__r0 = self.__plant.CalcCenterOfMassPositionInWorld(self.__plant_context)

        self.__base_link_name = end_frames_name[0]
        self.__left_eef_name = eef_frame_name_list[0] # custom added
        self.__right_eef_name = eef_frame_name_list[1]
        self.__q_lb = np.array(q_limit[0], dtype=float) if q_limit is not None else None
        self.__q_ub = np.array(q_limit[1], dtype=float) if q_limit is not None else None
        if self.__q_lb is not None:
            rad2deg = 180 / math.pi
            print("q lower_bound left: ", rad2deg * self.__q_lb[:7])
            print("q upper_bound left: ", rad2deg * self.__q_ub[:7])
            print("q lower_bound right:", rad2deg * self.__q_lb[-7:])
            print("q upper_bound right:", rad2deg * self.__q_ub[-7:])
        
    def type(self):
        return IkTypeIdx.DiffIK

    def q0(self):
        return self.__q0
    
        
    def left_hand_jacobian(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        J_hand_in_world = self.__plant.CalcJacobianSpatialVelocity(
            self.__plant_context, JacobianWrtVariable.kV,
            self.__plant.GetFrameByName(self.__left_eef_name), [0, 0, 0], self.__plant.world_frame(), self.__plant.world_frame())
        return J_hand_in_world[:, 6:13]

    def right_hand_jacobian(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        J_hand_in_world = self.__plant.CalcJacobianSpatialVelocity(
            self.__plant_context, JacobianWrtVariable.kV,
            self.__plant.GetFrameByName(self.__right_eef_name), [0, 0, 0], self.__plant.world_frame(), self.__plant.world_frame())
        return J_hand_in_world[:, -7:]

    def left_hand_pose(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        l_hand_in_base = self.__plant.GetFrameByName(self.__left_eef_name).CalcPose(self.__plant_context, self.__plant.GetFrameByName(self.__base_link_name))
        return (l_hand_in_base.translation(), l_hand_in_base.rotation().ToRollPitchYaw().vector())
    
    def right_hand_pose(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        r_hand_in_base = self.__plant.GetFrameByName(self.__right_eef_name).CalcPose(self.__plant_context, self.__plant.GetFrameByName(self.__base_link_name))
        return (r_hand_in_base.translation(), r_hand_in_base.rotation().ToRollPitchYaw().vector())

    def solve_left_hand(self, q, v, V_G_desired, h=0.01, v_max=3.0, vd_max=1.0):
        q_lb = self.__q_lb[:7] if self.__q_lb is not None else None
        q_ub = self.__q_ub[:7] if self.__q_ub is not None else None
        J_l = self.left_hand_jacobian(q)
        prog = self.MakeMathematicalProgram(q[7:14], v, J_l, V_G_desired, h, v_max, vd_max, q_lb=q_lb, q_ub=q_ub)
        result = Solve(prog)
        return np.asarray(result.GetSolution())

    def solve_right_hand(self, q, v, V_G_desired, h=0.01, v_max=3.0, vd_max=1.0):
        q_lb = self.__q_lb[-7:] if self.__q_lb is not None else None
        q_ub = self.__q_ub[-7:] if self.__q_ub is not None else None
        J_r = self.right_hand_jacobian(q)
        prog = self.MakeMathematicalProgram(q[-7:], v, J_r, V_G_desired, h, v_max, vd_max, q_lb=q_lb, q_ub=q_ub)
        result = Solve(prog)
        return np.asarray(result.GetSolution())

    '''
    This is a simple diff IK by QP for robots with 7 joints.
        @q: current joint angles
        @J_G: Jacobian matrix of the end-effector w.r.t. the joint angles
        @v_Gdesired: desired end-effector position and orientation (r, p, y, dx, dy, dz)
        @h: time step/(s)
    '''
    @staticmethod
    def MakeMathematicalProgram(q, v0, J_G, v_Gdesired, h, v_max, vd_max, q_lb=None, q_ub=None):
        """
        参数:
        q -- 初始关节角度
        v0 -- 初始关节速度
        J_G -- 雅可比矩阵
        v_Gdesired -- 期望速度向量
        h -- 时间步长
        v_max -- 速度最大值
        q_lb -- 状态下界（可选）
        q_ub -- 状态上界（可选）
        """
        prog = MathematicalProgram()
        v = prog.NewContinuousVariables(7, "v")
        # v_max = 3.0

        error = J_G @ v - np.asarray(v_Gdesired)
        Q = np.diag([1, 1, 1, 10, 10, 10])
        prog.AddCost(np.transpose(error) @ Q @ error)
        vd_error = v - v0
        vd_ratio = 0.1
        prog.AddCost(vd_ratio*vd_error.dot(vd_error))
        prog.AddBoundingBoxConstraint(-v_max, v_max, v)
        # TODO: add acceleration limit constraint
        # prog.AddBoundingBoxConstraint(v0 - vd_max*h, v0 + vd_max*h, v)
        if q_lb is not None and q_ub is not None:
            # q_lb <= q + h*v <= q_ub
            prog.AddBoundingBoxConstraint((q_lb - q)/h, (q_ub - q)/h, v)

        return prog

    def start_recording(self):
        if self.__meshcat is None:
            return
        self.__visualizer.StartRecording()

    def stop_andpublish_recording(self):
        if self.__meshcat is None:
            return
        print("Publish recording...")
        self.__visualizer.StopRecording()
        self.__visualizer.PublishRecording()

    def visualize_animation(self, q_list, start_time=0.0, duration=1.1):
        if self.__meshcat is None:
            return
        t_sol = np.array([start_time, start_time + duration])  
        q_sol = np.array(q_list).T
        q_pp = PiecewisePolynomial.FirstOrderHold(t_sol, q_sol)
        t0 = t_sol[0]
        tf = t_sol[-1]
        t = t0
        while t < tf:
            q = q_pp.value(t)
            self.__plant.SetPositions(self.__plant_context, q)
            self.__diagram_context.SetTime(t)
            self.__diagram.ForcedPublish(self.__diagram_context)
            t += duration/2.0
        # time.sleep(0.1)

    def get_arm_length(self):
        shoulder_frame_left = self.__plant.GetFrameByName(self.shoulder_frame_names[0])
        X_shoulder_left = shoulder_frame_left.CalcPoseInWorld(self.__plant_context)
        X_eef_left = self.__plant.GetFrameByName(self.__left_eef_name).CalcPoseInWorld(
            self.__plant_context
        )
        dis = X_shoulder_left.translation() - X_eef_left.translation()
        length_left = np.linalg.norm(dis)

        shoulder_frame_right = self.__plant.GetFrameByName(self.shoulder_frame_names[1])
        X_shoulder_right = shoulder_frame_right.CalcPoseInWorld(self.__plant_context)
        X_eef_right = self.__plant.GetFrameByName(
            self.__right_eef_name
        ).CalcPoseInWorld(self.__plant_context)
        dis = X_shoulder_right.translation() - X_eef_right.translation()
        length_right = np.linalg.norm(dis)

        return length_left, length_right


# pose: tuple(pos, ori(x,y,z,w))
def interpolate_pose(poseA, poseB, t0, dt, ds=0.01):
    pose1_pos, pose1_quat = np.array(poseA[0]), np.array(poseA[1])
    pose2_pos, pose2_quat = np.array(poseB[0]), np.array(poseB[1])

    norm = np.linalg.norm(pose2_pos - pose1_pos)
    mat1, mat2 = quaternion_to_matrix(pose1_quat), quaternion_to_matrix(pose2_quat)
    delta_theta, _ = rotation_matrix_diff_in_angle_axis(mat1, mat2)

    num_pos = int(norm / ds)
    num_ori = int(delta_theta/(0.5*np.pi/180.0))  # 0.5deg
    # print(f"num_pos: {num_pos}, num_ori: {num_ori}")
    num = max(num_pos, num_ori) # 取两者的最大值作为分段数
    num = max(2, min(3, num))
    # print(f"final num: {num}")
    X1 = RigidTransform(RotationMatrix(mat1), pose1_pos)
    X2 = RigidTransform(RotationMatrix(mat2), pose2_pos)
    return PiecewisePose.MakeLinear([t0, t0+num*dt], [X1, X2])

def draw_traj(traj_X_G):
    traj_p_G = traj_X_G.get_position_trajectory()
    traj_R_G = traj_X_G.get_orientation_trajectory()
    p_G = traj_p_G.vector_values(traj_p_G.get_segment_times())
    R_G = traj_R_G.vector_values(traj_R_G.get_segment_times())
    print(traj_p_G.get_segment_times())
    plt.plot(traj_p_G.get_segment_times(), p_G.T)
    t_interp = 0.4
    plt.plot(t_interp, traj_p_G.value(t_interp)[0], 'x')
    plt.plot(t_interp, traj_p_G.value(t_interp)[1], 'x')
    plt.plot(t_interp, traj_p_G.value(t_interp)[2], 'x')
    # plt.legend(["x", "y", "z"])
    plt.title("p_G")
    plt.show()

    plt.plot(traj_R_G.get_segment_times(), R_G.T)
    plt.legend(["qx", "qy", "qz", "qw"])
    plt.title("R_G")
    plt.show()

    traj_v_G = traj_p_G.MakeDerivative()
    v_G = traj_v_G.vector_values(traj_v_G.get_segment_times())
    plt.plot(traj_v_G.get_segment_times(), v_G.T)
    plt.legend(["vx", "vy", "vz"])
    plt.title("v_G")
    plt.show()

    traj_V_G = traj_X_G.MakeDerivative()
    V_G = traj_V_G.vector_values(traj_V_G.get_segment_times())
    plt.plot(traj_V_G.get_segment_times(), V_G.T)
    plt.legend(["vr", "vp", "vy", "vx", "vy", "vz"])
    plt.title("V_G")
    plt.show()

def draw_real_traj(x_record, pos_record, ori_record):
    pos_record = np.array(pos_record)
    plt.subplot(2, 3, 1)
    plt.plot(x_record, 1e3*pos_record[:, 0])
    plt.title('p_x')
    plt.ylabel('mm')
    plt.subplot(2, 3, 2)
    plt.plot(x_record, 1e3*pos_record[:, 1])
    plt.title('p_y')
    plt.ylabel('mm')
    plt.subplot(2, 3, 3)
    plt.plot(x_record, 1e3*pos_record[:, 2])
    plt.title('p_z')
    plt.ylabel('mm')

    ori_record = np.array(ori_record)
    RadToDeg = 180.0/math.pi
    plt.subplot(2, 3, 4)
    plt.plot(x_record, RadToDeg*ori_record[:, 0])
    plt.title('roll')
    plt.ylabel('deg')
    plt.subplot(2, 3, 5)
    plt.plot(x_record, RadToDeg*ori_record[:, 1])
    plt.title('pitch')
    plt.ylabel('deg')
    plt.subplot(2, 3, 6)
    plt.plot(x_record, RadToDeg*ori_record[:, 2])
    plt.title('yaw')
    plt.ylabel('deg')

    # plt.show()


if __name__ == "__main__":
    mc_pkg_path = get_package_path("motion_capture_ik")
    test = np.load(mc_pkg_path + "/data/rosbag_s.npy") # quat版本
    np.set_printoptions(linewidth=240)
    np.set_printoptions(threshold=2000)
    np.set_printoptions(precision=4)
    np.set_printoptions(suppress=True)

    meshcat = StartMeshcat()
    model_file = mc_pkg_path + "/robots/biped_s4/urdf/biped_s4.urdf"

    end_frames_name = ['torso','l_hand_roll','r_hand_roll']

    diff_ik = DiffIK(model_file, end_frames_name, meshcat)
    q0 = diff_ik.q0()
    last_q = q0
    diff_ik.start_recording()
    t = 0.0
    l_pose_0 = diff_ik.left_hand_pose(q0)
    q0[7:14] = [0.1084,  0.0478 , 0.1954 ,-0.0801 , 0.1966 ,-0.5861 , 0.0755]
    q_start = q0
    
    def get_pos(record_data, idx):
        return (record_data[idx, 0:3], record_data[idx, 3:7])

    diff_ik.start_recording()

        
    x_record = []
    pos_record = []
    ori_record = []
    for i in range(250):
        quat = test[i, 3:7]
        quat_drake = Quaternion(quat[3], quat[0], quat[1], quat[2])
        rpy = RollPitchYaw(quat_drake).vector()

        ori_record.append(rpy)
        pos_record.append(test[i, 0:3])
        x_record.append(i)
    draw_real_traj(x_record, pos_record, ori_record)

    x_record = []
    t_record = []
    pos_record = []
    ori_record = []
    controller_dt = 0.01
    for i in range(250-1): 
        traj_X_G = interpolate_pose(get_pos(test, i), get_pos(test, i+1), 0, controller_dt)
        pos_err = np.linalg.norm(get_pos(test, i)[0] - get_pos(test, i+1)[0])
        print(f"pos_err start: {pos_err}")
        # draw_traj(traj_X_G)

        time_0 = time.time()
        q_now, pos, rpy = control_to_pos(diff_ik, q_start, traj_X_G)
        q_start = q_now
        time_cost = time.time() - time_0
        print(f"time cost: {1e3*time_cost:.3f} ms")

        x_record.append(i)
        pos_record.append(pos)
        ori_record.append(rpy)
        t_record.append(1e3*time_cost)
        pos_err = np.linalg.norm(pos - get_pos(test, i+1)[0])
        print(f"pos_err end: {pos_err}")
    
    diff_ik.stop_andpublish_recording()
    print('Program end, Press Ctrl + C to exit.')
    draw_real_traj(x_record, pos_record, ori_record)
    plt.show()

    # plt.plot(x_record, np.array(t_record))
    # plt.title('time cost')
    # plt.ylabel('ms')
    
    while True:
        time.sleep(0.01)
