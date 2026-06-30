import sys
import os
import mink
import mujoco as mj
import numpy as np
import json
import pickle
from scipy.spatial.transform import Rotation as R
from motion_retarget import GeneralMotionRetargeting
from typing import Dict, Optional
import pathlib
from typing import Optional, Sequence
from params import ROBOT_XML_DICT, IK_CONFIG_DICT
from rich import print

# ---------------------------------------------------------------------------
# Pure-numpy quaternion helpers (wxyz convention) – avoid per-frame scipy cost
# ---------------------------------------------------------------------------
def _quat_wxyz_to_mat(q):
    """Convert quaternion (w,x,y,z) -> 3x3 rotation matrix.  Pure numpy."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    tx, ty, tz = 2.0*x, 2.0*y, 2.0*z
    twx, twy, twz = tx*w, ty*w, tz*w
    txx, txy, txz = tx*x, ty*x, tz*x
    tyy, tyz, tzz = ty*y, ty*z, tz*z
    return np.array([
        [1.0 - (tyy + tzz),       txy - twz,        txz + twy],
        [      txy + twz,   1.0 - (txx + tzz),       tyz - twx],
        [      txz - twy,         tyz + twx,   1.0 - (txx + tyy)],
    ], dtype=np.float64)


def _mat_to_quat_wxyz(M):
    """Convert 3x3 rotation matrix -> quaternion (w,x,y,z).  Pure numpy.
    Uses Shepperd's method for numerical stability."""
    tr = M[0, 0] + M[1, 1] + M[2, 2]
    if tr > 0.0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (M[2, 1] - M[1, 2]) * s
        y = (M[0, 2] - M[2, 0]) * s
        z = (M[1, 0] - M[0, 1]) * s
    elif M[0, 0] > M[1, 1] and M[0, 0] > M[2, 2]:
        s = 2.0 * np.sqrt(1.0 + M[0, 0] - M[1, 1] - M[2, 2])
        w = (M[2, 1] - M[1, 2]) / s
        x = 0.25 * s
        y = (M[0, 1] + M[1, 0]) / s
        z = (M[0, 2] + M[2, 0]) / s
    elif M[1, 1] > M[2, 2]:
        s = 2.0 * np.sqrt(1.0 + M[1, 1] - M[0, 0] - M[2, 2])
        w = (M[0, 2] - M[2, 0]) / s
        x = (M[0, 1] + M[1, 0]) / s
        y = 0.25 * s
        z = (M[1, 2] + M[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + M[2, 2] - M[0, 0] - M[1, 1])
        w = (M[1, 0] - M[0, 1]) / s
        x = (M[0, 2] + M[2, 0]) / s
        y = (M[1, 2] + M[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    if q[0] < 0.0:
        q = -q
    q /= np.linalg.norm(q)
    return q

# ---------------------------------------------------------------------------


def _avg_existing(vals: Sequence[Optional[float]]) -> Optional[float]:
    xs = [float(v) for v in vals if v is not None]
    return (sum(xs) / len(xs)) if xs else None

def get_body_z(d: "mj.MjData", body_name: str) -> Optional[float]:
    try:
        return float(d.body(body_name).xpos[2])
    except Exception:
        return None

def get_pose_z(pose: Optional[Dict[str, tuple]], key: str) -> Optional[float]:
    if not pose or key not in pose:
        return None
    p = pose[key][0]  # (pos, quat)
    if p is None:
        return None
    return float(p[2])

class VMR_bvh(GeneralMotionRetargeting):
    """ based on General Motion Retargeting (GMR).
    change the scale and offset logic of the original GMR function
    """
    def __init__(
        self,
        src_human: str,
        tgt_robot: str,
        actual_human_height: float = None,
        solver: str="daqp", # change from "quadprog" to "daqp".
        damping: float=5e-1, # change from 1e-1 to 1e-2.
        verbose: bool=False,
        use_velocity_limit: bool=False,
        contact_sequence: Dict[str, np.ndarray] = None,
        cali_pose: Dict[str, tuple] = None,
    ) -> None:
        # used for contact offset
        self.contact_sequence = contact_sequence
        self.previous_human_data = None

        # load the robot model
        self.xml_file = str(ROBOT_XML_DICT[tgt_robot])
        if verbose:
            print("Use robot model: ", self.xml_file)
        self.model = mj.MjModel.from_xml_path(self.xml_file)
        
        # Print DoF names in order
        # print("[GMR] Robot Degrees of Freedom (DoF) names and their order:")
        self.robot_dof_names = {}
        for i in range(self.model.nv):  # 'nv' is the number of DoFs
            dof_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_JOINT, self.model.dof_jntid[i])
            self.robot_dof_names[dof_name] = i
            if verbose:
                print(f"DoF {i}: {dof_name}")
            
            
        # print("[GMR] Robot Body names and their IDs:")
        self.robot_body_names = {}
        for i in range(self.model.nbody):  # 'nbody' is the number of bodies
            body_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_BODY, i)
            self.robot_body_names[body_name] = i
            if verbose:
                print(f"Body ID {i}: {body_name}")
        
        # print("[GMR] Robot Motor (Actuator) names and their IDs:")
        self.robot_motor_names = {}
        for i in range(self.model.nu):  # 'nu' is the number of actuators (motors)
            motor_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, i)
            self.robot_motor_names[motor_name] = i
            if verbose:
                print(f"Motor ID {i}: {motor_name}")

        # Load the IK config
        with open(IK_CONFIG_DICT[src_human][tgt_robot]) as f:
            ik_config = json.load(f)
        if verbose:
            print("Use IK config: ", IK_CONFIG_DICT[src_human][tgt_robot])
        
        # compute the scale ratio based on given human height and the assumption in the IK config
        if actual_human_height is not None:
            ratio = actual_human_height / ik_config["human_height_assumption"]
        else:
            ratio = 1.0
        ratio = 1.0
            
        # adjust the human scale table
        for key in ik_config["human_scale_table"].keys():
            ik_config["human_scale_table"][key] = ik_config["human_scale_table"][key] * ratio
    

        # used for retargeting
        self.ik_match_table1 = ik_config["ik_match_table1"]
        self.ik_match_table2 = ik_config["ik_match_table2"]
        self.human_root_name = ik_config["human_root_name"]
        self.robot_root_name = ik_config["robot_root_name"]
        self.use_ik_match_table1 = ik_config["use_ik_match_table1"]
        self.use_ik_match_table2 = ik_config["use_ik_match_table2"]
        self.human_scale_table = ik_config["human_scale_table"]
        self.ground = ik_config["ground_height"] * np.array([0, 0, 1])

        self.max_iter = 10

        self.solver = solver
        self.damping = damping

        self.human_body_to_task1 = {}
        self.human_body_to_task2 = {}
        self.pos_offsets1 = {}
        self.rot_offsets1 = {}
        self.pos_offsets2 = {}
        self.rot_offsets2 = {}

        self.task_errors1 = {}
        self.task_errors2 = {}

        self.ik_limits = [mink.ConfigurationLimit(self.model)]
        if use_velocity_limit:
            VELOCITY_LIMITS = {k: 3*np.pi for k in self.robot_motor_names.keys()}
            self.ik_limits.append(mink.VelocityLimit(self.model, VELOCITY_LIMITS)) 

        
        self.autocalibrate_offsets_from_bvh_default_pose(cali_pose, align_feet=True)
            
        self.setup_retarget_configuration()
        
        self.ground_offset = 0.0
        self.min_ground_clearance = 0.01  # 0.1 mm
        self.flag = 0
    
    
    def update_targets(self, human_data, offset_to_ground=False):
        # scale human data in local frame
        human_data = self.to_numpy(human_data)
        human_data = self.scale_human_data(human_data, self.human_root_name, self.human_scale_table)
        human_data = self.offset_human_data_robotframe(human_data, self.pos_offsets1, self.rot_offsets1)
        
        self.scaled_human_data = human_data

        if self.use_ik_match_table1:
            for body_name in self.human_body_to_task1.keys():
                task = self.human_body_to_task1[body_name]
                pos, rot = human_data[body_name]
                task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3(rot), pos))
        
        if self.use_ik_match_table2:
            for body_name in self.human_body_to_task2.keys():
                task = self.human_body_to_task2[body_name]
                pos, rot = human_data[body_name]
                task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3(rot), pos))
            
    
    # -----------------------------------------------------------------------
    # Automatic calibration: compute IK pos/rot offsets from a BJ BVH **default pose**
    # -----------------------------------------------------------------------
    def autocalibrate_offsets_from_bvh_default_pose(
        self,
        cali_pose: Dict[str, tuple],
        align_feet: bool = True,
        verbose: bool = True,
    ) -> None:
        """Overwrite the (pos_offset, rot_offset) in `ik_match_table1/2` and the
        runtime `pos_offsets*/rot_offsets*` dicts using a BJ BVH **default pose**.

        The produced offsets are in **robot tracker frame** :
            t_r2s = RR^T (p_s - p_r)
            R_s2r = RR^T RS
        """

        # Start from the model's current qpos (same as the solver's state)
        d = mj.MjData(self.model)
        try:
            d.qpos[:] = self.configuration.data.qpos.copy()
        except Exception:
            pass
        mj.mj_forward(self.model, d)

        cali_pose = self.scale_human_data(cali_pose, self.human_root_name, self.human_scale_table)

        # Optional: align robot base height so feet touch ground similarly in the default pose
        if align_feet:
            hz = _avg_existing([
                get_pose_z(cali_pose, "LeftToeBase"),
                get_pose_z(cali_pose, "RightToeBase"),
            ])
            rz = _avg_existing([
                get_body_z(d, "leg_l6_link"),
                get_body_z(d, "leg_r6_link"),
            ])
            if hz is not None and rz is not None:
                dz = float(hz - rz)
                d.qpos[2] += dz
                mj.mj_forward(self.model, d)
                if verbose:
                    print(f"[AutoCalib] base z += {dz:+.4f} (human_toe_z={hz:.4f}, robot_foot_z={rz:.4f})")
            elif verbose:
                print("[AutoCalib] skip feet alignment (missing toe/foot bodies).")

        def _update_table(table: dict, pos_offsets: dict, rot_offsets: dict, tag: str):
            for frame_name, entry in table.items():
                if not isinstance(entry, (list, tuple)) or len(entry) < 5:
                    continue
                human_name = entry[0]
                if human_name not in cali_pose:
                    continue
                try:
                    pr = d.body(frame_name).xpos.copy()
                    RR = d.body(frame_name).xmat.reshape(3, 3).copy()
                except Exception:
                    continue

                ps, qs_wxyz = cali_pose[human_name]
                RS = R.from_quat(qs_wxyz, scalar_first=True).as_matrix()

                t_r2s = RR.T @ (ps - pr)
                R_s2r = RR.T @ RS
                q_xyzw = R.from_matrix(R_s2r).as_quat()
                q_wxyz = q_xyzw[[3, 0, 1, 2]]

                # overwrite in-place (json-friendly)
                entry[3] = [float(x) for x in t_r2s.tolist()]
                entry[4] = [float(x) for x in q_wxyz.tolist()]
                # overwrite runtime dicts (used in offset_human_data_robotframe)
                pos_offsets[human_name] = t_r2s.astype(float)
                rot_offsets[human_name] = R.from_quat(q_wxyz, scalar_first=True)

            if verbose:
                print(f"[AutoCalib] updated {tag}: {len(pos_offsets)} bodies")

        # Update both tables and the runtime offset dicts
        
        _update_table(self.ik_match_table1, self.pos_offsets1, self.rot_offsets1, "ik_match_table1")
        _update_table(self.ik_match_table2, self.pos_offsets2, self.rot_offsets2, "ik_match_table2")

        # Pre-cache numpy matrices for offset_human_data_robotframe (avoid per-frame scipy)
        self._build_offset_cache()

    def _build_offset_cache(self):
        """Pre-compute R_s2r_inv matrices and pos_offset arrays for fast numpy path."""
        self._rot_offsets1_inv_mat = {}  # body_name -> (3,3) ndarray  = R_s2r^{-1} = R_s2r^T
        self._pos_offsets1_arr = {}      # body_name -> (3,) ndarray
        for body_name, rot_obj in self.rot_offsets1.items():
            self._rot_offsets1_inv_mat[body_name] = rot_obj.inv().as_matrix().astype(np.float64)
            self._pos_offsets1_arr[body_name] = np.asarray(self.pos_offsets1[body_name], dtype=np.float64)

        self._rot_offsets2_inv_mat = {}
        self._pos_offsets2_arr = {}
        for body_name, rot_obj in self.rot_offsets2.items():
            self._rot_offsets2_inv_mat[body_name] = rot_obj.inv().as_matrix().astype(np.float64)
            self._pos_offsets2_arr[body_name] = np.asarray(self.pos_offsets2[body_name], dtype=np.float64)

    def offset_human_data_robotframe(self, human_data, pos_offsets, rot_offsets):
        """
        Convert mocap tracker global pose -> robot frame global target pose.
        Pure-numpy implementation (no per-frame scipy Rotation objects).

        human_data[body] = (p_s, q_s) where q_s is global tracker orientation (wxyz)
        pos_offsets[body] = t_r2s in ROBOT frame (3,)
        rot_offsets[body] = R_s2r (Rotation), mapping tracker frame -> robot frame

        Uses pre-cached inverse matrices from _build_offset_cache().
        """
        # Select the correct pre-cached dicts
        if rot_offsets is self.rot_offsets1:
            inv_mat_cache = self._rot_offsets1_inv_mat
            pos_cache = self._pos_offsets1_arr
        else:
            inv_mat_cache = self._rot_offsets2_inv_mat
            pos_cache = self._pos_offsets2_arr

        out = {}
        for body_name, (pos_s, quat_s) in human_data.items():
            pos_s = np.asarray(pos_s, dtype=np.float64)

            # If some bodies don't have offsets, pass-through
            if body_name not in inv_mat_cache:
                out[body_name] = [pos_s, quat_s]
                continue

            # R_s from tracker quaternion (wxyz) -> matrix
            R_s_mat = _quat_wxyz_to_mat(quat_s)

            # R_r = R_s @ R_s2r_inv  (equivalent to R_s * R_s2r.inv())
            R_r_mat = R_s_mat @ inv_mat_cache[body_name]

            # quat_r (wxyz)
            quat_r = _mat_to_quat_wxyz(R_r_mat)

            # pos_r = pos_s - R_r @ t_r2s  (equivalent to pos_s - R_r.apply(t_r2s))
            t_r2s = pos_cache[body_name]
            pos_r = pos_s - R_r_mat @ t_r2s

            out[body_name] = [pos_r, quat_r]

        return out


