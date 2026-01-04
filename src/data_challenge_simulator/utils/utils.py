import math
import os
from typing import List, Tuple, Optional, Literal
import xml.etree.ElementTree as ET
import random, time
from kuavo_humanoid_sdk import KuavoPose, KuavoRobotState, KuavoRobot
from scipy.spatial.transform import Rotation as R
import numpy as np
from utils.object_pos import ObjectPose
from kuavo_humanoid_sdk.interfaces.data_types import KuavoIKParams
class Utils:
    def __init__(self):
        pass

    @staticmethod
    def is_in_target_region(position, region_bounds):

        if position is None:
            return False

        for i in range(3):
            if not (region_bounds[i][0] <= position[i] <= region_bounds[i][1]):
                return False
        return True

    @staticmethod
    def interpolate_joint_trajectory(q_target: List[float], q_start: List[float] = None, num: int = 50) -> List[List[float]]:

        if q_start is None:
            q_start = [0.0] * len(q_target)

        q_list = []
        for i in range(num):
            q_tmp = [
                math.radians(q_start[j] + i / float(num) * (q_target[j] - q_start[j]))
                for j in range(len(q_target))
            ]
            q_list.append(q_tmp)
        return q_list

    @staticmethod
    def interpolate_joint_trajectory_polynomial(
            q_target: List[float], 
            q_start: List[float] = None, 
            num: int = 50,
            order: int = 3,
            start_velocity: Optional[List[float]] = None,
            end_velocity: Optional[List[float]] = None
        ) -> List[List[float]]:

        if q_start is None:
            q_start = [0.0] * len(q_target)
        
        if start_velocity is None:
            start_velocity = [0.0] * len(q_target)
        if end_velocity is None:
            end_velocity = [0.0] * len(q_target)
        
        if len(q_start) != len(q_target) or len(start_velocity) != len(q_target) or len(end_velocity) != len(q_target):
            raise ValueError("q_start, q_target, start_velocity, end_velocity 的长度必须相同")
        
        q_list = []
        
        for i in range(num):
            t = i / float(num - 1) if num > 1 else 0.0
            
            q_tmp = []
            for j in range(len(q_target)):
                q0 = q_start[j]
                q1 = q_target[j]
                v0 = start_velocity[j]
                v1 = end_velocity[j]
                delta_q = q1 - q0
                
                if order == 1:

                    q_interp = q0 + t * delta_q
                elif order == 3:

                    a = q0
                    b = v0
                    c = 3.0 * delta_q - 2.0 * v0 - v1
                    d = v0 + v1 - 2.0 * delta_q
                    q_interp = a + b * t + c * t * t + d * t * t * t
                elif order == 5:

                    a = q0
                    b = v0
                    c = 0.0  

                    d = 10.0 * delta_q - 6.0 * v0 - 4.0 * v1
                    e = -15.0 * delta_q + 8.0 * v0 + 7.0 * v1
                    f = 6.0 * delta_q - 3.0 * v0 - 3.0 * v1
                    q_interp = a + b * t + c * t * t + d * t * t * t + e * t * t * t * t + f * t * t * t * t * t
                else:
                    raise ValueError(f"不支持的多项式阶数: {order}，支持的值: 1, 3, 5")
                
                q_tmp.append(math.radians(q_interp))
            
            q_list.append(q_tmp)
        
        return q_list

    @staticmethod
    def compute_pose(
        robot: KuavoRobot,
        robot_state: KuavoRobotState,
        mode: Literal["left", "right", "both"],
        pos_left: Optional[List[float]] = None,
        quat_left: Optional[List[float]] = None,
        pos_right: Optional[List[float]] = None,
        quat_right: Optional[List[float]] = None,
    ) -> Tuple[List[float], List[float]]:

        base_pos = np.array(robot_state.robot_position())
        base_quat = np.array(robot_state.robot_orientation())
        base_rot_inv = R.from_quat(base_quat).inv()

        def convert_pose(pos_world: List[float], quat_world: List[float]) -> KuavoPose:

            pos_rel = np.array(pos_world) - base_pos
            pos_robot = base_rot_inv.apply(pos_rel)

            rot_world = R.from_quat(quat_world)
            rot_robot = base_rot_inv * rot_world
            quat_robot = rot_robot.as_quat().tolist()
            return KuavoPose(position=pos_robot.tolist(), orientation=quat_robot)

        left_pose = None
        right_pose = None

        if mode in ["left", "both"]:
            if pos_left is None or quat_left is None:
                raise ValueError("左手 pos / quat 缺失")
            left_pose = convert_pose(pos_left, quat_left)

        if mode in ["right", "both"]:
            if pos_right is None or quat_right is None:
                raise ValueError("右手 pos / quat 缺失")
            right_pose = convert_pose(pos_right, quat_right)

        q_curr = robot_state.arm_joint_state().position
        if mode == "left":
            fk_result = robot.arm_fk(q_curr)
            if fk_result is None or fk_result[0] is None or fk_result[1] is None:
                raise RuntimeError("FK 求解失败，无法获取当前手臂位置")
            right_pose = fk_result[1]
        elif mode == "right":

            fk_result = robot.arm_fk(q_curr)
            if fk_result is None or fk_result[0] is None or fk_result[1] is None:
                raise RuntimeError("FK 求解失败，无法获取当前手臂位置")
            left_pose = fk_result[0]

        if left_pose is None or right_pose is None:
            raise ValueError(f"IK 求解失败：left_pose 或 right_pose 为 None (mode={mode})")
        params = KuavoIKParams(major_optimality_tol=1e-3, major_feasibility_tol=1e-3, minor_feasibility_tol=1e-3, major_iterations_limit=100, oritation_constraint_tol=1e-3, pos_constraint_tol=1e-3, pos_cost_weight=0.0)
        q_result = robot.arm_ik(left_pose, right_pose, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], q_curr,params)

        if q_result is None:
            raise RuntimeError("IK 求解失败，请检查目标位姿是否合理")

        left_q = list(q_result[:7]) if mode in ["left", "both"] else []
        right_q = list(q_result[7:]) if mode in ["right", "both"] else []

        return left_q, right_q
    
    @staticmethod
    def robot_to_world(pos_robot, robot_state):

        base_pos = np.array(robot_state.robot_position())         
        base_quat = np.array(robot_state.robot_orientation())      
        base_rot = R.from_quat(base_quat)
        pos_world = base_pos + base_rot.apply(pos_robot)
        return pos_world.tolist()
    
    @staticmethod
    def wait_for_grasp_pose(
            robot, robot_state,
            y_grasp_left=None,   
            quat_left=None,      
            y_grasp_right=None,  
            quat_right=None,    
            v=0.1,               
            mode="left",       
            obj_name_left="box_grab", 
            obj_name_right="box_grab", 
            x_grasp_left=None,   
            z_grasp_left=None,   
            x_grasp_right=None,  
            z_grasp_right=None,  
            move_lead_time=0.3,   
            check_interval=0.01,  
            axis="y",           
            timeout: Optional[float] = None 
        ):

        from utils.utils import Utils

        obj_pos_reader = ObjectPose()
        
        if axis not in ["x", "y"]:
            raise ValueError(f"axis must be 'x' or 'y', got '{axis}'")

        start_time = time.time()

        def elapsed():
            return time.time() - start_time

        while True:

            if mode in ["left", "both"]:

                while obj_pos_reader.get_position(obj_name_left) is None:
                    if timeout is not None and elapsed() > timeout:
                        print(f"[wait_for_grasp_pose] 超时等待物体 {obj_name_left} 的位姿，elapsed={elapsed():.2f}s")

                        obj_pos_left = None
                        break
                    print(f"[wait_for_grasp_pose] 等待物体 {obj_name_left} 的位姿数据...")
                    time.sleep(0.01)
                obj_pos_left = obj_pos_reader.get_position(obj_name_left)
                
                if axis == "y":

                    coord_obj_left = obj_pos_left[1] 
                    coord_grasp_left = y_grasp_left 
                    coord_idx = 1  
                else: 
                    coord_obj_left = obj_pos_left[0]  
                    coord_grasp_left = x_grasp_left   
                    coord_idx = 0 

                x_left = x_grasp_left if x_grasp_left is not None else obj_pos_left[0]
                z_left = z_grasp_left if z_grasp_left is not None else obj_pos_left[2]
                y_left = y_grasp_left if y_grasp_left is not None else obj_pos_left[1]

                t_to_grasp_left = (coord_grasp_left - coord_obj_left) / v if coord_grasp_left is not None else float('inf')
            else:
                t_to_grasp_left = float('inf')


            if mode in ["right", "both"]:

                while obj_pos_reader.get_position(obj_name_right) is None:
                    if timeout is not None and elapsed() > timeout:
                        print(f"[wait_for_grasp_pose] 超时等待物体 {obj_name_right} 的位姿，elapsed={elapsed():.2f}s")
                        obj_pos_right = None
                        break
                    print(f"[wait_for_grasp_pose] 等待物体 {obj_name_right} 的位姿数据...")
                    time.sleep(0.01)
                obj_pos_right = obj_pos_reader.get_position(obj_name_right)
                
                if axis == "y":
                    coord_obj_right = obj_pos_right[1]
                    coord_grasp_right = y_grasp_right 
                    coord_idx = 1
                else:  # axis == "x"
                    coord_obj_right = obj_pos_right[0] 
                    coord_grasp_right = x_grasp_right 
                    coord_idx = 0 

                x_right = x_grasp_right if x_grasp_right is not None else obj_pos_right[0]
                z_right = z_grasp_right if z_grasp_right is not None else obj_pos_right[2]
                y_right = y_grasp_right if y_grasp_right is not None else obj_pos_right[1]

                t_to_grasp_right = (coord_grasp_right - coord_obj_right) / v if coord_grasp_right is not None else float('inf')
            else:
                t_to_grasp_right = float('inf')

            if timeout is not None and elapsed() > timeout:
                print(f"[wait_for_grasp_pose] 超时退出主等待循环，elapsed={elapsed():.2f}s")

                pos_left_est = None
                pos_right_est = None
                try:
                    if mode in ["left", "both"]:
                        if 'obj_pos_left' in locals() and obj_pos_left is not None:
                            x_left = x_grasp_left if x_grasp_left is not None else obj_pos_left[0]
                            z_left = z_grasp_left if z_grasp_left is not None else obj_pos_left[2]
                            y_left = y_grasp_left if y_grasp_left is not None else obj_pos_left[1]
                            pos_left_est = [x_left, y_left, z_left]
                    if mode in ["right", "both"]:
                        if 'obj_pos_right' in locals() and obj_pos_right is not None:
                            x_right = x_grasp_right if x_grasp_right is not None else obj_pos_right[0]
                            z_right = z_grasp_right if z_grasp_right is not None else obj_pos_right[2]
                            y_right = y_grasp_right if y_grasp_right is not None else obj_pos_right[1]
                            pos_right_est = [x_right, y_right, z_right]
                    left_q, right_q = Utils.compute_pose(
                        robot,
                        robot_state,
                        mode=mode,
                        pos_left=pos_left_est if mode in ['left', 'both'] else None,
                        quat_left=quat_left if mode in ['left', 'both'] else None,
                        pos_right=pos_right_est if mode in ['right', 'both'] else None,
                        quat_right=quat_right if mode in ['right', 'both'] else None,
                    )
                    return left_q, right_q
                except Exception as e:
                    print(f"[wait_for_grasp_pose] 超时后基于当前观测计算 IK 失败: {e}")
                    return None, None

            in_window = (
                (mode in ["left", "both"] and t_to_grasp_left <= move_lead_time) or
                (mode in ["right", "both"] and t_to_grasp_right <= move_lead_time)
            )
            if in_window:

                pos_left, pos_right = None, None
                if mode in ["left", "both"]:
                    if axis == "y":

                        y_pred_left = coord_obj_left + v * t_to_grasp_left
                        pos_left = [x_left, y_pred_left, z_left]
                    else:  # axis == "x"

                        x_pred_left = coord_obj_left + v * t_to_grasp_left
                        pos_left = [x_pred_left, y_left, z_left]
                        
                if mode in ["right", "both"]:
                    if axis == "y":

                        y_pred_right = coord_obj_right + v * t_to_grasp_right
                        pos_right = [x_right, y_pred_right, z_right]
                    else:  # axis == "x"

                        x_pred_right = coord_obj_right + v * t_to_grasp_right
                        pos_right = [x_pred_right, y_right, z_right]

                left_q, right_q = Utils.compute_pose(
                    robot,
                    robot_state,
                    mode=mode,
                    pos_left=pos_left if mode in ['left', 'both'] else None,
                    quat_left=quat_left if mode in ['left', 'both'] else None,
                    pos_right=pos_right if mode in ['right', 'both'] else None,
                    quat_right=quat_right if mode in ['right', 'both'] else None,
                )
                return left_q, right_q
            time.sleep(check_interval)

    @staticmethod
    def is_front_facing(
        quat_xyzw,
        *,
        body_front_axis='x',         
        front_world_dir='-x',   
        tol_deg=30.0
    ):

        def _axis_to_vec(axis_str: str) -> np.ndarray:
            axis_str = axis_str.strip().lower()
            table = {
                'x':  np.array([ 1., 0., 0.]),
                '-x': np.array([-1., 0., 0.]),
                'y':  np.array([ 0., 1., 0.]),
                '-y': np.array([ 0.,-1., 0.]),
                'z':  np.array([ 0., 0., 1.]),
                '-z': np.array([ 0., 0.,-1.]),
            }
            if axis_str not in table:
                raise ValueError("axis must be one of: x, -x, y, -y, z, -z")
            return table[axis_str]

        if isinstance(body_front_axis, str):
            n_body = _axis_to_vec(body_front_axis)
        else:
            raise ValueError("body_front_axis must be a string axis like 'x', '-y', etc.")

        if isinstance(front_world_dir, str):
            fw = _axis_to_vec(front_world_dir)
        else:
            fw = np.asarray(front_world_dir, dtype=float)
            if fw.shape != (3,):
                raise ValueError("front_world_dir vector must be shape (3,)")
            n = np.linalg.norm(fw)
            if n < 1e-12:
                raise ValueError("front_world_dir vector norm is zero")
            fw = fw / n

        q = np.asarray(quat_xyzw, dtype=float)
        if q.shape != (4,):
            raise ValueError("quat_xyzw must be length-4 (x, y, z, w)")
        nq = np.linalg.norm(q)
        if nq < 1e-12:

            q = np.array([0., 0., 0., 1.], dtype=float)
        else:
            q = q / nq

        Rw = R.from_quat(q)

        n_world = Rw.apply(n_body)
        n = np.linalg.norm(n_world)
        if n < 1e-12:

            return False, 180.0
        n_world = n_world / n

        cosang = float(np.clip(np.dot(n_world, fw), -1.0, 1.0))
        angle = math.degrees(math.acos(cosang))
        return (angle <= tol_deg), angle

    @staticmethod
    def _build_segment_rad(target_rad, start_rad=None):

        if start_rad is None:
            start_rad = [0.0] * len(target_rad)
        return [list(start_rad), list(target_rad)]

    @staticmethod
    def execute_trajectory_with_ruckig(
            robot: KuavoRobot,
            robot_state: KuavoRobotState,
            traj_ctrl,
            q_target,
            q_start: Optional[List[float]] = None,
            sleep_time: float = 0.01,
            joint_v_max: Optional[List[float]] = None,
            joint_a_max: Optional[List[float]] = None,
            joint_j_max: Optional[List[float]] = None,
            speed_scale: float = 3.0,
            vel_settle_thresh: float = 0.03,
            publish_frequency: float = 100.0
        ):

        try:
            from ruckig import Ruckig, InputParameter, OutputParameter, Result
            HAS_RUCKIG = True
        except ImportError:
            HAS_RUCKIG = False
            if not hasattr(Utils.execute_trajectory_with_ruckig, "_warned"):
                import rospy
                rospy.logwarn("Ruckig 未安装，将使用固定插值执行轨迹")
                Utils.execute_trajectory_with_ruckig._warned = True
        
        import rospy

        if not q_target:
            rospy.logerr("无效的输入：q_target 不能为空")
            return False

        if isinstance(q_target[0], (int, float)):

            if q_start is None:

                curr_state = robot_state.arm_joint_state()
                q_start = list(curr_state.position)
            q_list = Utils._build_segment_rad(q_target, q_start)
        elif isinstance(q_target[0], (list, tuple)):

            q_list = [list(q) for q in q_target]
        else:
            rospy.logerr(f"无效的输入类型：q_target[0] 的类型是 {type(q_target[0])}")
            return False

        if not q_list:
            return True

        if not HAS_RUCKIG:
            traj_ctrl.execute_trajectory(q_list, sleep_time=sleep_time)
            return True

        dof = len(q_list[0])
        if joint_v_max is None:
            joint_v_max = [2.5] * dof
        if joint_a_max is None:
            joint_a_max = [5.0] * dof
        if joint_j_max is None:
            joint_j_max = [50.0] * dof

        dt = 1.0 / publish_frequency
        otg = Ruckig(dof, dt)
        out = OutputParameter(dof)

        curr_state = robot_state.arm_joint_state()
        curr_pos = list(curr_state.position)
        measured_vel = list(getattr(curr_state, "velocity", [0.0] * dof) or [0.0] * dof)
        vel_norm = sum(v * v for v in measured_vel) ** 0.5
        if vel_norm < vel_settle_thresh:
            measured_vel = [0.0] * dof
        curr_vel = measured_vel
        curr_acc = [0.0] * dof
        min_v = 0.01 
        
        traj_ctrl.set_target_positions(curr_pos)
        time.sleep(dt * 2.0)
        
        for idx, waypoint in enumerate(q_list):
            if idx == 0:
                waypoint = curr_pos.copy()
            
            delta_max = max(abs(waypoint[j] - curr_pos[j]) for j in range(dof))
            if delta_max < 1e-3:
                continue

            if max(abs(waypoint[j] - curr_pos[j]) for j in range(dof)) < 1e-6:
                continue
            
            inp = InputParameter(dof)
            inp.current_position = curr_pos
            inp.current_velocity = curr_vel
            inp.current_acceleration = curr_acc
            inp.target_position = waypoint

            adaptive_vmax = []
            for j in range(dof):
                raw_delta = abs(waypoint[j] - curr_pos[j])
                desired_v = raw_delta / max(sleep_time, 1e-3)
                vmax = min(joint_v_max[j] * speed_scale, desired_v * 1.2 * speed_scale)
                adaptive_vmax.append(max(min_v, vmax))
            inp.target_velocity = [0.0] * dof 
            inp.target_acceleration = [0.0] * dof
            inp.max_velocity = adaptive_vmax
            inp.max_acceleration = joint_a_max
            inp.max_jerk = joint_j_max
            
            res = Result.Working
            safety = 0
            try:
                while res == Result.Working and safety < 5000:
                    res = otg.update(inp, out)
                    traj_ctrl.set_target_positions(out.new_position)
                    time.sleep(dt)

                    inp.current_position = out.new_position
                    inp.current_velocity = out.new_velocity
                    inp.current_acceleration = out.new_acceleration
                    safety += 1
            except Exception as e:
                rospy.logwarn(f"Ruckig 出错，退回固定插值执行此段: {e}")
                remaining = q_list[idx:]
                traj_ctrl.execute_trajectory(remaining, sleep_time=sleep_time)
                return True
            curr_pos = out.new_position
            curr_vel = out.new_velocity
            curr_acc = out.new_acceleration
        
        rospy.loginfo("Ruckig 轨迹执行完成")
        return True

    @staticmethod
    def augment_trajectory(
        q_list,
        base_sleep,
        noise_deg=3,
        via_noise_deg=3,
        stretch_range=(0.8, 1.2),
        per_step_random=False
    ):

        if not q_list:
            if per_step_random:
                return q_list, [base_sleep]
            return q_list, base_sleep
        n = len(q_list)

        def center_taper(idx):
            if n <= 1:
                return 0.0

            return max(0.0, 1.0 - abs(2.0 * idx / (n - 1) - 1.0))

        augmented = []
        for i, q in enumerate(q_list):
            weight = center_taper(i)
            jitter = [
                val + math.radians(random.uniform(-noise_deg, noise_deg)) * weight
                for val in q
            ]
            augmented.append(jitter)

        if n > 6 and random.random() < 0.6:
            mid_idx = random.randint(n // 4, 3 * n // 4)
            via = [
                augmented[mid_idx][j] + math.radians(random.uniform(-via_noise_deg, via_noise_deg))
                for j in range(len(augmented[mid_idx]))
            ]
            augmented = augmented[:mid_idx] + [via] + augmented[mid_idx:]

        if per_step_random:

            sleep_times = []
            for i in range(len(augmented)):
                sleep_times.append(base_sleep * random.uniform(stretch_range[0], stretch_range[1]))
            
            if random.random() < 0.5:

                pause_idx = random.randint(len(augmented) // 3, 2 * len(augmented) // 3)
                augmented.insert(pause_idx, augmented[pause_idx])

                sleep_times.insert(pause_idx, base_sleep * random.uniform(stretch_range[0], stretch_range[1]))
            
            return augmented, sleep_times
        else:

            sleep_time = base_sleep * random.uniform(stretch_range[0], stretch_range[1])
            if random.random() < 0.5:

                pause_idx = random.randint(len(augmented) // 3, 2 * len(augmented) // 3)
                augmented.insert(pause_idx, augmented[pause_idx])
                
            return augmented, sleep_time
    
    @staticmethod
    def euler_to_quat(roll, pitch, yaw):

        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return {'x': qx, 'y': qy, 'z': qz, 'w': qw}

    def sample_orientation_jitter(max_yaw_deg=180.0, max_pitch_deg=0, max_roll_deg=0):

        yaw = math.radians(random.uniform(-max_yaw_deg, max_yaw_deg))
        pitch = math.radians(random.uniform(-max_pitch_deg, max_pitch_deg))
        roll = math.radians(random.uniform(-max_roll_deg, max_roll_deg))
        return Utils.euler_to_quat(roll, pitch, yaw)