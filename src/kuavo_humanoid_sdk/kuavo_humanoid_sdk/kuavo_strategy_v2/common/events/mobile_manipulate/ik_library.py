import numpy as np
from scipy.spatial.transform import Rotation as R
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate.torso_ik_from_kuavo_ros_control import ArmIk
import os, sys
cur_dir = os.path.dirname(os.path.abspath(__file__))

VERBOSE = False


def transfer_l7_eef_to_l7(eef_pos, eef_quat_xyzw):
    """
    A: base_link
    B: zarm_l7_end_effector
    C: zarm_l7_link
    :return:
    """

    eef_in_l7 = [0, 0.03, -0.17]
    # eef_in_l7 = [0, 0.0, -0.0]


    # 已知量：
    p_A_B = np.array([eef_pos[0], eef_pos[1], eef_pos[2]])  # B 在 A 下的位置
    # q_A_B = np.array([eef_quat_xyzw[3], eef_quat_xyzw[0], eef_quat_xyzw[1], eef_quat_xyzw[2]])  # B 在 A 下的四元数 (xyzw)
    p_B_C = - np.array(eef_in_l7)  # C 在 B 下的位置（无旋转）

    # 1. 计算 R_A_B
    R_A_B = R.from_quat(eef_quat_xyzw).as_matrix()
    # 注意：scipy 用的是 [x, y, z, w] 顺序

    # 2. C 在 A 下的位置
    l7_pos = p_A_B + R_A_B @ p_B_C

    # 3. C 在 A 下的四元数（与 B 一样）
    l7_quat_xyzw = eef_quat_xyzw.copy()

    return l7_pos, l7_quat_xyzw

def transform(rot, p=np.zeros(3)):
    return np.block([[rot, p.reshape([3, 1])], [np.zeros(3), 1.]])


def transform_from_x_rotation(theta, p=np.zeros(3)):
    return transform(R.from_euler('x', theta).as_matrix(), p)


def transform_from_y_rotation(theta, p=np.zeros(3)):
    return transform(R.from_euler('y', theta).as_matrix(), p)


def transform_from_z_rotation(theta, p=np.zeros(3)):
    return transform(R.from_euler('z', theta).as_matrix(), p)


def wrap_to_half_pi(theta):
    while (theta >= np.pi / 2):
        theta -= np.pi
    while (theta < -np.pi / 2):
        theta += np.pi
    return theta


def wrap_to_pi(theta):
    while (theta >= np.pi):
        theta -= 2 * np.pi
    while (theta < -np.pi):
        theta += 2 * np.pi
    return theta


def clip_to_pi(angle):
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


offset_theta_4 = np.arctan2(0.02, 0.2837) + np.arctan2(0.02, 0.126 + 0.1075)
l_34 = np.sqrt(0.02 ** 2 + 0.2837 ** 2)
l_45 = np.sqrt(0.02 ** 2 + (0.126 + 0.1075) ** 2)

safe_acos = lambda theta: np.arccos(np.clip(theta, -1., 1.))
safe_asin = lambda theta: np.arcsin(np.clip(theta, -1., 1.))
safe_atan2 = lambda y, x: np.arctan2(y, x) if (x ** 2 + y ** 2) > 1e-6 else 0.

joint_limits = {
    'joint_1': (-3.14159265358979, 1.5707963267949),
    'joint_2': (-0.349065850398866, 2.0943951023932),
    'joint_3': (-1.5707963267949, 1.5707963267949),
    'joint_4': (-2.61799387799149, 0.0),
    'joint_5': (-1.5707963267949, 1.5707963267949),
    'joint_6': (-1.30899693899575, 0.698131700797732),
    'joint_7': (-0.698131700797732, 0.698131700797732),
    # 'joint_7': (-1.9198621771937625, 1.0471975511965976)
}


class IKAnalytical:

    @staticmethod
    def ik_leftarm_w_err(X_07):
        l = np.linalg.norm(X_07[:3, 3])
        x_07, y_07, z_07 = transform_from_y_rotation(0.6*np.pi / 4)[:3, :3] @ transform_from_z_rotation(np.pi / 4)[:3, :3] @ \
                        X_07[:3, 3]
        if (VERBOSE):
            if ((l_34 ** 2 + l_45 ** 2 - l ** 2) / (2 * l_34 * l_45) > 1.):
                print("target too short?")
            elif ((l_34 ** 2 + l_45 ** 2 - l ** 2) / (2 * l_34 * l_45) < -1.):
                print("target too long?")
        theta_4_virtual = np.pi - safe_acos((l_34 ** 2 + l_45 ** 2 - l ** 2) / (2 * l_34 * l_45))
        x_37 = l_45 * np.sin(theta_4_virtual)
        z_37 = -(l_34 + l_45 * np.cos(theta_4_virtual))
        theta_3_virtual = safe_atan2(x_07, np.linalg.norm([y_07, z_07])) - safe_atan2(x_37, -z_37)
        theta = theta_3_virtual + theta_4_virtual + np.arctan2(0.02, 0.126 + 0.1075)
        R_24 = transform_from_y_rotation(-theta)[:3, :3]
        theta_2_virtual = safe_atan2(y_07, -z_07)
        R_04 = transform_from_x_rotation(theta_2_virtual)[:3, :3] @ R_24
        R_04 = transform_from_z_rotation(-np.pi / 4)[:3, :3] @ transform_from_y_rotation(-0.6*np.pi / 4)[:3, :3] @ R_04
        R_47 = np.linalg.inv(R_04) @ X_07[:3, :3]
        theta_5, theta_6, theta_7 = R.from_matrix(R_47[:3, :3]).as_euler('ZXY')
        theta_4 = -(theta_4_virtual + offset_theta_4)
        R_34 = transform_from_y_rotation(theta_4)[:3, :3]
        R_03 = R_04 @ np.linalg.inv(R_34)

        theta_1 = safe_atan2(R_03[0, 2], R_03[2, 2])
        if (theta_1 > 0.75 * np.pi):
            theta_1 -= 2 * np.pi
        theta_2 = np.arcsin(-R_03[1, 2])
        R_01 = transform_from_y_rotation(theta_1)[:3, :3]
        R_12 = transform_from_x_rotation(theta_2)[:3, :3]
        R_23 = np.linalg.inv(R_12) @ np.linalg.inv(R_01) @ R_03
        theta_3 = safe_atan2(R_23[1, 0], R_23[0, 0])

        return np.array([theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7])

    @staticmethod
    def fk_leftarm(thetas):  # use mujoco forward instead?
        theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7 = thetas
        X_01 = transform_from_y_rotation(theta_1)
        X_12 = transform_from_x_rotation(theta_2)
        X_23 = transform_from_z_rotation(theta_3)
        X_34 = transform_from_y_rotation(theta_4, np.array([0.02, 0., -0.2837]))
        X_45 = transform_from_z_rotation(theta_5, np.array([-0.02, 0., -(0.126 + 0.1075)]))
        X_56 = transform_from_x_rotation(theta_6)
        X_67 = transform_from_y_rotation(theta_7, np.array([0., 0., -0.021]))
        X_07 = X_01 @ X_12 @ X_23 @ X_34 @ X_45 @ X_56 @ X_67
        return X_07

    @staticmethod
    def ik_leftarm_loop(X_07):
        err = np.zeros(3)
        err_norm = np.inf
        X_07_tmp = np.copy(X_07)
        N_ITER = 10
        for i in range(N_ITER):

            X_07_tmp[:3, 3] -= err
            thetas = IKAnalytical.ik_leftarm_w_err(X_07_tmp)
            err = IKAnalytical.fk_leftarm(thetas)[:3, 3] - X_07[:3, 3]
            err_norm0 = err_norm
            err_norm = np.linalg.norm(err)
            if (VERBOSE):
                with np.printoptions(precision=6):
                    print(
                        f"iter {i}, origin target:{X_07[:3, 3]}, passing target:{X_07_tmp[:3, 3]}, result:{IKAnalytical.fk_leftarm(thetas)[:3, 3]}, err:{err_norm:.6e}")
            if (np.abs(err_norm0 - err_norm) < 1e-6):
                break
        return thetas

    @staticmethod
    def compute(eef_pos,
                eef_quat_xyzw,
                eef_frame,
                model_type='45', # 45 or 46 or 60, default is 45
                limit=True,
                ):
        """
        :return:
        """
        assert eef_frame in ['zarm_l7_link', 'zarm_r7_link'], \
            f"IKAnalytical only supports 'zarm_l7_link', 'zarm_r7_link' frame, got {eef_frame}"

        if eef_frame in ['zarm_r7_link']:
            side = 'right'
        else:
            side = 'left'

        eef_pos = np.asarray(eef_pos)
        eef_quat_xyzw = np.asarray(eef_quat_xyzw)

        # print(f'IKAnalytical input: eef_pos = {eef_pos}, eef_quat_xyzw = {eef_quat_xyzw}, side = {side}')
        if side == 'right':
            # 把右手的eef_target_in_base转成左手的
            euler = R.from_quat(eef_quat_xyzw).as_euler('xyz')  # 外部系旋转
            euler_mirrored = [-euler[0], euler[1], -euler[2]]
            eef_quat_xyzw = R.from_euler('xyz', euler_mirrored).as_quat()
            eef_pos = np.asarray([eef_pos[0], -eef_pos[1], eef_pos[2]])
        #
        # if eef_frame in ['zarm_l7_end_effector', 'zarm_r7_end_effector']:
        #     # 转换到 l7_link 下
        #     print(f'>>>>>>>> Before transfer: eef_pos = {eef_pos}, eef_quat_xyzw = {eef_quat_xyzw}')
        #     eef_pos, eef_quat_xyzw = transfer_l7_eef_to_l7(eef_pos, eef_quat_xyzw)
        #     print(f'>>>>>>>> After transfer: eef_pos = {eef_pos}, eef_quat_xyzw = {eef_quat_xyzw}')

        # model_type -> l1_link_in_base
        l1_link_map = {
            '45': [-0.017, 0.293, 0.424],
            '46': [-0.0174999, 0.2927, 0.445],
            '60': [-0.018, 0.255, 0.192],
        }

        try:
            l1_link_in_base = l1_link_map[model_type]
            if VERBOSE:
                print(f'>>>>>>>> model_type: {model_type}')
                print(f'>>>>>>>> l1_link_in_base: {l1_link_in_base}')
        except KeyError:
            raise ValueError(f"Unsupported model_type {model_type}. Supported types: {list(l1_link_map.keys())}")

        target_rotmat_in_base = R.from_quat(eef_quat_xyzw).as_matrix()

        # print(f'xmat = {target_rotmat_in_base}')
        X_07 = transform(target_rotmat_in_base.reshape([3, 3]),
                         eef_pos - l1_link_in_base)

        thetas = IKAnalytical.ik_leftarm_loop(X_07)

        # print(f'IKAnalytical output thetas (before limit and side adjust): {thetas}')

        if limit:
            for i, joint_name in enumerate(
                    ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']):
                lower, upper = joint_limits[joint_name]
                thetas[i] = clip_to_pi(thetas[i])
                thetas[i] = np.clip(thetas[i], lower, upper)

        if side == 'right':
            # 把左手的thetas转成右手的. # 0, 3, 6 正向，其余反向
            thetas_converted = [
                thetas[0],
                -thetas[1],
                -thetas[2],
                thetas[3],
                -thetas[4],
                -thetas[5],
                thetas[6],
            ]
            thetas = thetas_converted
            # return np.asarray(thetas_converted)

        # print(f'IKAnalytical final output thetas: {thetas}')
        return np.asarray(thetas)[:7]


class IKNumerical:
    def __init__(self, urdf_file=os.path.join(cur_dir, 'drake_urdf/urdf/biped_v3_arm.urdf'), if_two_stgs=False):
        self.arm_ik = ArmIk(
            model_file=urdf_file,
            end_frames_name=["base_link", "zarm_l7_link", "zarm_l1_link"],
            meshcat=None,
            constraint_tol=1e-3,
            solver_tol=1.0e-4,
            iterations_limit=10000,
            eef_z_bias=0.0,
        )

        torso_yaw_deg = 0.0
        torso_height = 0.0
        self.arm_ik.init_state(torso_yaw_deg, torso_height)
        q0 = self.arm_ik.q0()
        self.arm_ik.start_recording()
        l_pose = self.arm_ik.left_hand_pose(q0)
        self.arm_ik.set_use_two_stage_ik(if_two_stgs)
        print(
            f"left_hand_pose: {l_pose[0]}, {l_pose[1]}"
        )

    def compute(self,
                eef_pos,
                eef_quat_xyzw,
                eef_frame,
                ref_q=None,
                limit=True
                ):
        l_hand_RPY = R.from_quat(eef_quat_xyzw).as_euler('xyz')  # 外部系旋转
        assert eef_frame in ['zarm_l7_link', 'zarm_r7_link'], \
            f"IKAnalytical only supports 'zarm_l7_link', 'zarm_r7_link' frame, got {eef_frame}"

        if eef_frame in ['zarm_r7_link']:
            side = 'right'
        else:
            side = 'left'

        if side == 'right':
            # 把右手的eef_target_in_base转成左手的
            euler_mirrored_ = [-l_hand_RPY[0], l_hand_RPY[1], -l_hand_RPY[2]]
            l_hand_RPY = euler_mirrored_
            eef_pos = [eef_pos[0], -eef_pos[1], eef_pos[2]]

        l_elbow_pos = None
        r_elbow_pos = None
        r_hand_RPY = None
        r_hand_pose = None  # [x, y, z]
        q0 = self.arm_ik.q0()

        print(f' >>>>>>>>>>>>>>>>>>>>>>>>. arm q0 {q0} <<<<<<<<<<<<<<<<<<<<<')

        q0_heuristic = [-0.60276536, 0.61077987, -1.10754203, -1.28068895, 1.12118157, 0.56953695,
                        0.34970026, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0
                        ]
        if ref_q is not None:
            q0 = q0_heuristic
            for i in range(len(ref_q)):
                if ref_q[i] is not None:
                    q0[i] = ref_q[i]
        q_now = self.arm_ik.computeIK(
            q0,
            l_hand_pose=eef_pos,
            r_hand_pose=r_hand_pose,
            l_hand_RPY=l_hand_RPY,
            r_hand_RPY=r_hand_RPY,
            l_elbow_pos=l_elbow_pos,
            r_elbow_pos=r_elbow_pos,
        )
        if limit:
            for i, joint_name in enumerate(
                    ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']):
                lower, upper = joint_limits[joint_name]
                q_now[i] = clip_to_pi(q_now[i])
                q_now[i] = np.clip(q_now[i], lower, upper)

        if side == 'right':
            # 把左手的thetas转成右手的. # 0, 3, 6 正向，其余反向
            thetas_converted = [
                q_now[0],
                -q_now[1],
                -q_now[2],
                q_now[3],
                -q_now[4],
                -q_now[5],
                q_now[6],
            ]
            q_now = thetas_converted
            # return np.asarray(thetas_converted)

        return np.asarray(q_now)[:7]
