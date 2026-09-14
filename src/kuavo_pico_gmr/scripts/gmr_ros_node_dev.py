import argparse
import pathlib
from re import L
import time
from kuavo_gmr import GeneralMotionRetargeting as GMR
from kuavo_gmr import RobotMotionViewer
from kuavo_gmr.utils.pico import load_pico_file
from rich import print
from tqdm import tqdm
import os
import numpy as np
from copy import deepcopy


""" This is an abstract class that defines the input and output of the GMR ROS NODE """
class GMRROSNodeAbs:
    def __init__(self):
        #### PICO body names
        # refer to the definition of BodyTrackerRole in the PICO SDK 
        # https://developer-cn.picoxr.com/document/unity-openxr/body-tracking/
        self.pico_body_names = [
            'pelvis',
            'left_hip',
            'right_hip',
            'spine1',
            'left_knee',
            'right_knee',
            'spine2',
            'left_ankle',
            'right_ankle',
            'spine3',
            'left_foot',
            'right_foot',
            'neck',
            'left_collar',
            'right_collar',
            'head',
            'left_shoulder',
            'right_shoulder',
            'left_elbow',
            'right_elbow',
            'left_wrist',
            'right_wrist',
            'left_hand',
            'right_hand',
        ]

    def retarget(self, pico_frame: dict) -> dict:
        """
        Args: 
            pico_frame: dict, the input pico frame
                '{' pico_body_name: [translation: np.ndarray, orientation: np.ndarray] '}' 
        Returns:
            retarget_output: dict, the output of this node
        """
        #### check the format of the input pico_frame
        for bone_name in self.pico_body_names:
            if bone_name not in pico_frame.keys():
                raise ValueError(f"Missing bone name: {bone_name}")
            
            bone_data = pico_frame[bone_name] # List[np.ndarray], length: 2
            bone_translation = bone_data[0]   # np.ndarray, shape: (3,)
            bone_orientation = bone_data[1]   # np.ndarray, shape: (4,), NOTE quaternion, first: w, then x, y, z

            if len(bone_translation) != 3:
                raise ValueError(f"Invalid bone translation length: {len(bone_translation)}")
            if len(bone_orientation) != 4:
                raise ValueError(f"Invalid bone orientation length: {len(bone_orientation)}")


        #### define the output of this node
        retarget_output = {
            "base_translation": np.zeros(3),
            "base_orientation": np.ones(4) / 2,
            "base_velocity": np.zeros(6),
            "joint_position": np.zeros(26),
            "joint_velocity": np.zeros(26),
            "end_effector_position": np.zeros(12),
            "head_joints": np.zeros(2),
        }

        return retarget_output


""" This is the implementation of the GMR ROS NODE """
class GMRROSNode(GMRROSNodeAbs):
    def __init__(self):
        super().__init__()
        #### Initialize GMR retargeter
        self.retargeter = GMR(
            src_human="pico",
            tgt_robot="kuavo_s45",
            actual_human_height=1.54,
        )

        #### std output for GMR ROS NODE
        self.retarget_output = {
            "base_translation": np.zeros(3),
            "base_orientation": np.ones(4) / 2,
            "base_velocity": np.zeros(6),
            "joint_position": np.zeros(26),
            "joint_velocity": np.zeros(26),
            "end_effector_position": np.zeros(12),
            "head_joints": np.zeros(2),
        }

        self.nq = 35    # general position 3+4 + 28 (including 2 head Dof)
        self.last_timestamp_ms = None

        #### velocity limits
        self.lower_vel_limit = -3 * np.ones(self.nq-1) * np.pi
        self.upper_vel_limit =  3 * np.ones(self.nq-1) * np.pi

        #### store the last position and velocity results for velocity estimation
        self.last_position = np.zeros(self.nq)   # xyz, wxyz, dofpos
        self.filtered_velocity = np.zeros(self.nq-1) # base link rotation velocity is 3 dim
        self.alpha = 0.1   # hyper parameter for 1st-order low-pass filter
        self.filtered_velocity_init = False
        #### store the last pico frame for drift offset raw human motion data
        self.last_pico_frame = None

    def retarget(self, pico_frame: dict, timestamp_ms: int) -> dict:
        """
        Args:
            pico_frame: dict, the input pico frame
                '{' pico_body_name: [translation_xyz: np.ndarray, orientation_wxyz: np.ndarray] '}'
                pico_body_name: see self.pico_body_names in GMRROSNodeAbs
            timestamp_ms: int, the timestamp of the pico frame in milliseconds
        
        Returns:
            retarget_output: dict, the output of this node
                '{' 
                    "base_translation": np.ndarray, shape: (3,), # xyz
                    "base_orientation": np.ndarray, shape: (4,), # wxyz
                    "base_velocity": np.ndarray, shape: (6,),    # linear velocity, angular velocity
                    "joint_position": np.ndarray, shape: (26,),  # dofpos of robot's joints except for the two head Dofs
                    "joint_velocity": np.ndarray, shape: (26,),  # dofvel of robot's joints except for the two head Dofs
                    "end_effector_position": np.ndarray, shape: (12,) # translation of left foot, right foot, left hand, right hand
                '}'
                measure in m, m/s, rad, rad/s

        Sequence of the joint_position and joint_velocity:
            Motor ID 0: leg_l1_joint_motor
            Motor ID 1: leg_l2_joint_motor
            Motor ID 2: leg_l3_joint_motor
            Motor ID 3: leg_l4_joint_motor
            Motor ID 4: leg_l5_joint_motor
            Motor ID 5: leg_l6_joint_motor
            Motor ID 6: leg_r1_joint_motor
            Motor ID 7: leg_r2_joint_motor
            Motor ID 8: leg_r3_joint_motor
            Motor ID 9: leg_r4_joint_motor
            Motor ID 10: leg_r5_joint_motor
            Motor ID 11: leg_r6_joint_motor
            Motor ID 12: zarm_l1_joint_motor
            Motor ID 13: zarm_l2_joint_motor
            Motor ID 14: zarm_l3_joint_motor
            Motor ID 15: zarm_l4_joint_motor
            Motor ID 16: zarm_l5_joint_motor
            Motor ID 17: zarm_l6_joint_motor
            Motor ID 18: zarm_l7_joint_motor
            Motor ID 19: zarm_r1_joint_motor
            Motor ID 20: zarm_r2_joint_motor
            Motor ID 21: zarm_r3_joint_motor
            Motor ID 22: zarm_r4_joint_motor
            Motor ID 23: zarm_r5_joint_motor
            Motor ID 24: zarm_r6_joint_motor
            Motor ID 25: zarm_r7_joint_motor
        """
        if self.last_timestamp_ms is not None:
            dt = timestamp_ms - self.last_timestamp_ms
        else:
            dt = None

        self.last_timestamp_ms = timestamp_ms
        pico_frame = self.preprocess_pico_frame(pico_frame)

        qpos = self.retargeter.retarget(pico_frame)
        qvel = self.velocity_estimation(current_position=qpos, dt=dt) # nq-1

        #### compute end effector position
        self.retargeter.configuration.update(q=qpos) # run forward kinematics
        l_foot_se = self.retargeter.configuration.get_transform_frame_to_world(frame_name="leg_l6_link", frame_type="body")
        r_foot_se = self.retargeter.configuration.get_transform_frame_to_world(frame_name="leg_r6_link", frame_type="body")
        l_hand_se = self.retargeter.configuration.get_transform_frame_to_world(frame_name="zarm_l7_link", frame_type="body")
        r_hand_se = self.retargeter.configuration.get_transform_frame_to_world(frame_name="zarm_r7_link", frame_type="body")
        end_translation = np.concatenate([l_foot_se.translation(), r_foot_se.translation(), l_hand_se.translation(), r_hand_se.translation()])

        #### convert to standard output format
        self.retarget_output["base_translation"][:] = qpos[:3]
        self.retarget_output["base_orientation"][:] = qpos[3:7]
        self.retarget_output["base_velocity"][:] = qvel[:6]
        self.retarget_output["joint_position"][:] = qpos[7:-2]
        self.retarget_output["joint_velocity"][:] = qvel[6:-2]
        self.retarget_output["end_effector_position"][:] = end_translation
        self.retarget_output["head_joints"][:] = qpos[-2:]
        return self.retarget_output

    def preprocess_pico_frame(self, pico_frame: dict) -> dict:
        """
        This function aims to preprocess the pico frame to make it compatible with the GMR algorithm
        The operations include:
        1. Use the spine3 orientation as the pelvis orientation
        2. ... Add more operations here if implemented ...
        """
        # remove redundant bodies from pico_frame
        if "Bone_24" in pico_frame.keys(): del pico_frame["Bone_24"]
        # 0. offset the pico frame to reduce the global drift
        pico_frame = self.offset_pico_drift(pico_frame)
        # 1. Use the spine3 orientation as the pelvis orientation
        pico_frame["pelvis"][1] = pico_frame["spine3"][1] # use spine3 orientation as the pelvis orientation
        # 2. ... Add more operations here if implemented ...
        return pico_frame

    def velocity_estimation(self, current_position: np.ndarray, dt: float | None) -> np.ndarray:
        """
        Args:
            current_position: np.ndarray, shape: (nq,) # xyz of baselink, wxyz of baselink, dofpos of robot's joints
            dt: float, the time interval between the current and last frame in milliseconds
        Returns:
            velocity: np.ndarray, shape: (nq-1,)

        1. The velocity of the first frame is default to be zero
        2. The velocity is estimated by naive finite difference method
        """
        if not hasattr(self, "_last_pos_initialized"):
            self._last_pos_initialized = False

        if not self._last_pos_initialized:
            self.last_position[:] = current_position
            self._last_pos_initialized = True
            self.filtered_vel = np.zeros(self.nq-1)
            return np.zeros(self.nq-1)

        if dt is None:
            return np.zeros(self.nq-1)
        
        if dt <= 0: # abnormal delta time
            dt = 10 # assume an average of 10ms latency

        trans_vel = (current_position[:3] - self.last_position[:3]) / dt

        #region compute angular velocity
        def normalize_quat(q: np.ndarray) -> np.ndarray:
            norm = np.linalg.norm(q)
            if norm == 0:
                return np.array([1.0, 0.0, 0.0, 0.0])
            return q / norm

        def quat_conj(q: np.ndarray) -> np.ndarray:
            return np.array([q[0], -q[1], -q[2], -q[3]])

        def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
            w1, x1, y1, z1 = q1
            w2, x2, y2, z2 = q2
            return np.array([
                w1*w2 - x1*x2 - y1*y2 - z1*z2,
                w1*x2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2,
            ])

        q_curr = normalize_quat(current_position[3:7])
        q_last = normalize_quat(self.last_position[3:7])
        delta_q = quat_multiply(q_curr, quat_conj(q_last))
        delta_q = normalize_quat(delta_q)

        #### initial implementation
        # delta_q[0] = np.clip(delta_q[0], -1.0, 1.0)
        # angle = 2.0 * np.arccos(delta_q[0])
        # sin_half = np.sin(angle / 2.0)
        # if np.isclose(sin_half, 0.0):
        #     ang_vel = np.zeros(3)
        # else:
        #     axis = delta_q[1:] / sin_half
        #     ang_vel = axis * angle / dt
        
        #### more stable one
        vec = delta_q[1:]
        vec_norm = np.linalg.norm(vec)
        if vec_norm < 1e-6:
            ang_vel = np.zeros(3)
        else:
            angle = 2.0 * np.arctan2(vec_norm, delta_q[0])
            ang_vel = angle * (vec / vec_norm) / dt
        #endregion compute angular velocity

        joint_vel = (current_position[7:] - self.last_position[7:]) / dt
        current_velocity = np.concatenate([trans_vel, ang_vel, joint_vel])
        current_velocity *= 1000. # from /ms to /s
        current_velocity = np.clip(current_velocity, a_min=self.lower_vel_limit, a_max=self.upper_vel_limit)
        has_inf = np.isinf(current_velocity).any()
        assert not has_inf

        #### low pass filter
        self.last_position[:] = current_position
        if self.filtered_velocity_init:
            self.filtered_velocity[:] = self.alpha * current_velocity + (1-self.alpha) * self.filtered_velocity
        else:
            self.filtered_velocity[:] = current_velocity
            self.filtered_velocity_init = True

        return self.filtered_velocity

    def offset_pico_drift(self, pico_frame: dict) -> dict:
        """
        Args:
            pico_frame: dict, the input pico frame
        Returns:
            pico_frame: dict, the output pico frame
        """
        if self.last_pico_frame is None:
            lowest_body_name = self._detect_lowest_body(pico_frame)
            offset_height = pico_frame[lowest_body_name][0][2]
            for body_name in pico_frame.keys():
                pico_frame[body_name][0][2] -= offset_height
            self.last_pico_frame = pico_frame
            return pico_frame

        # detect the lowest body of the current pico frame
        lowest_body_name = self._detect_lowest_body(pico_frame)
        last_translation, _ = self.last_pico_frame[lowest_body_name]
        current_translation, _ = pico_frame[lowest_body_name]
        last_translation[2] = 0.0
        delta_translation = last_translation - current_translation
        
        # normed_distance = np.linalg.norm(delta_translation)
        # large drift is abnormal
        # if normed_distance > 0.2:
        #     print("Lowest body name: ", lowest_body_name)
        #     print(f"[WARNING] Normed distance: {normed_distance}")
        #     print("--------------------------------")
        # offest body translation of current frame
        
        # this will cause 0.00001s = 0.01ms latency
        for body_name in pico_frame.keys():
            translation, _ = pico_frame[body_name]
            pico_frame[body_name][0] = translation + delta_translation
        self.last_pico_frame = pico_frame

        return pico_frame

    def _detect_lowest_body(self, pico_frame: dict) -> str:
        #region set search region as full body
        # lowest_height = 10.
        # lowest_body_name = None
        # for body_name, body_pose in pico_frame.items():
        #     translation, _ = body_pose
        #     if translation[2] < lowest_height:
        #         lowest_height = translation[2]
        #         lowest_body_name = body_name
        #endregion 

        #region  set search region as {right_foot, left_foot}
        right_translation, _ = pico_frame["right_foot"]
        left_translation,  _ = pico_frame["left_foot"]
        if right_translation[2] < left_translation[2]:
            lowest_height = right_translation[2]
            lowest_body_name = "right_foot"
        else:
            lowest_height = left_translation[2]
            lowest_body_name = "left_foot"
        #endregion

        # assert lowest_height < 0.2
        return lowest_body_name


if __name__ == "__main__":
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    from utils.eval_class import EvalMetric
    eval_metric = EvalMetric()

    rate_limit = False  # whether to limit the rendering rate
    plot_debug = False  # whether to plot the debug information

    #############
    # ATTENTION #
    #############
    # the input for gmr_ros_node could refer to pico_data_frames[i], frame_timestamps[i]
    # pico_data_frames, _, frame_timestamps = load_pico_file("pico_data/pico_data_ori.csv")
    # pico_data_frames, _, frame_timestamps = load_pico_file("pico_data/eval_data.csv")
    pico_data_frames, _, frame_timestamps = load_pico_file("pico_data/ground_slope.csv")
    # pico_data_frames, _, frame_timestamps = load_pico_file("pico_data/time_stamp.csv")

    gmr_ros_node = GMRROSNode()
    #############
    # ATTENTION #
    #############

    robot_motion_viewer = RobotMotionViewer(
        robot_type="kuavo_s45",
        motion_fps=30, # no affect on the visualization if rate_limit is False
        transparent_robot=0,
        record_video=True,
        video_path="videos/refined_results.mp4")

    # FPS measurement variables
    fps_counter = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0  # Display FPS every 2 seconds
    pbar = tqdm(total=len(pico_data_frames), desc="Retargeting")
    i = 0
    output_buf = {key: [] for key in gmr_ros_node.retarget_output.keys()}

    #region retargeting and visualizing
    while True:
        fps_counter += 1
        current_time = time.time()
        if current_time - fps_start_time >= fps_display_interval:
            actual_fps = fps_counter / (current_time - fps_start_time)
            print(f"Actual rendering FPS: {actual_fps:.2f}")
            fps_counter = 0
            fps_start_time = current_time
            
        # Update progress bar
        pbar.update(1)
        # Update task targets.
        smplx_data = pico_data_frames[i]

        start = time.perf_counter()        
        #############
        # ATTENTION #
        #############
        # retarget_output is designed as mentioned in Feishu
        retarget_output = gmr_ros_node.retarget(pico_frame=smplx_data, timestamp_ms=frame_timestamps[i])
        #############
        # ATTENTION #
        #############
        end = time.perf_counter()
        elapsed = end - start
        eval_metric.add_latency(delta_t=elapsed)
        for key in output_buf.keys(): output_buf[key].append(retarget_output[key].copy())
        eval_metric.add_velocity(np.concatenate([retarget_output["base_velocity"], retarget_output["joint_velocity"]]))
        eval_metric.add_timestamp(stamp=frame_timestamps[i])
        eval_metric.add_lowest_height(retarget_output["end_effector_position"][2])
        # eval_metric.add_lowest_height(smplx_data[gmr_ros_node._detect_lowest_body(pico_frame=smplx_data)][0][2]) # the 

        # visualize scaled results (before IK optimisation) and retargeted results-qpos (after IK optimisation)
        robot_motion_viewer.step(
            root_pos=retarget_output["base_translation"],
            root_rot=retarget_output["base_orientation"],
            dof_pos=np.concatenate([retarget_output["joint_position"], retarget_output["head_joints"]]),
            human_motion_data=gmr_ros_node.retargeter.scaled_human_data,
            rate_limit=rate_limit,
            follow_camera=True,
            # human_pos_offset=np.array([0.0, 0.0, 0.0])
        )

        i = (i + 1) % len(pico_data_frames)
        if i == 0: break # only run one loop

    pbar.close()
    robot_motion_viewer.close()
    # eval_metric.plot_latency_hist()
    eval_metric.plot_velocities()
    # eval_metric.plot_timestamp()
    # eval_metric.plot_lowest_height()
    #endregion retargeting and visualizing

    if plot_debug:
        import matplotlib.pyplot as plt
        # Plot joint_pos
        joint_pos = np.array(output_buf["joint_position"]) # num_fps, 26
        plt.figure(figsize=(12, 6))
        for j in range(joint_pos.shape[1]):
            plt.plot(joint_pos[:, j], label=f'joint {j}')
        plt.title('Joint Positions')
        plt.xlabel('Frame')
        plt.ylabel('Position')
        plt.legend(loc='upper right', fontsize='x-small', ncol=2)
        plt.tight_layout()
        plt.show()

        # Plot joint_vel
        joint_vel = np.array(output_buf["joint_velocity"]) # num_fps, 26
        plt.figure(figsize=(12, 6))
        for j in range(joint_vel.shape[1]):
            plt.plot(joint_vel[:, j], label=f'joint {j}')
        plt.title('Joint Velocities')
        plt.xlabel('Frame')
        plt.ylabel('Velocity')
        plt.legend(loc='upper right', fontsize='x-small', ncol=2)
        plt.tight_layout()
        plt.show()

        # Plot base_pos
        base_pos = np.array(output_buf["base_translation"]) # num_fps, 3
        plt.figure(figsize=(10, 5))
        for j in range(base_pos.shape[1]):
            plt.plot(base_pos[:, j], label=f'base {j}')
        plt.title('Base Positions')
        plt.xlabel('Frame')
        plt.ylabel('Position')
        plt.legend(loc='upper right', fontsize='x-small', ncol=2)
        plt.tight_layout()
        plt.show()

        # plot base_rot
        base_rot = np.array(output_buf["base_orientation"]) # num_fps, 4
        plt.figure(figsize=(10, 5))
        for j in range(base_rot.shape[1]):
            plt.plot(base_rot[:, j], label=f'base {j}')
        plt.title('Base Orientations')
        plt.xlabel('Frame')
        plt.ylabel('Orientation')
        plt.legend(loc='upper right', fontsize='x-small', ncol=2)
        plt.tight_layout()
        plt.show()

        # Plot base_vel (6D: typically 3 linear, 3 angular)
        base_vel = np.array(output_buf["base_velocity"])   # num_fps, 6
        plt.figure(figsize=(10, 5))
        for j in range(base_vel.shape[1]):
            plt.plot(base_vel[:, j], label=f'base {j}')
        plt.title('Base Velocities')
        plt.xlabel('Frame')
        plt.ylabel('Velocity')
        plt.legend(loc='upper right', fontsize='x-small')
        plt.tight_layout()
        plt.show()

        # Plot end effector positions (12D: e.g., 4 limbs x 3D)
        end_effector_pos = np.array(output_buf["end_effector_position"]) # num_fps, 12
        plt.figure(figsize=(12, 6))
        for j in range(end_effector_pos.shape[1]):
            plt.plot(end_effector_pos[:, j], label=f'EE {j}')
        plt.title('End Effector Positions')
        plt.xlabel('Frame')
        plt.ylabel('Position')
        plt.legend(loc='upper right', fontsize='x-small', ncol=2)
        plt.tight_layout()
        plt.show()
