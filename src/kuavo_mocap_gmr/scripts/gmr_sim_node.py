#!/usr/bin/env python3
"""
GMR Simulation Node

Simulates OptiTrack data stream for testing the GMR pipeline without actual hardware.
Supports:
  1. Generating synthetic skeleton data (sine wave motion)
  2. Reading from recorded pickle files
  3. Reading from recorded BVH files

Usage:
    roslaunch kuavo_mocap_gmr gmr_sim.launch
    roslaunch kuavo_mocap_gmr gmr_sim.launch data_file:=/path/to/recording.pkl
"""

import sys
import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

# Setup paths
script_dir = os.path.dirname(os.path.abspath(__file__))
pkg_dir = os.path.dirname(script_dir)
gmr_core_path = os.path.join(pkg_dir, 'src', 'kuavo_mocap_gmr', 'gmr_core')
if os.path.isdir(gmr_core_path) and gmr_core_path not in sys.path:
    sys.path.insert(0, gmr_core_path)
    print(f"[GMR Sim] Added to path: {gmr_core_path}")

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header

# Import custom messages from kuavo_msgs
from kuavo_msgs.msg import (
    MocapPoseRetarget,
    RigidBodyFrame,
    SkeletonFrame,
    RigidBodyDescription,
)

# Import GMR modules
from GMRmodule_bvh import VMR_bvh
from robot_motion_viewer import RobotMotionViewer
from params import ROBOT_XML_DICT


class GMRSimNode:
    """Simulation node for GMR testing."""

    # OptiTrack Motive skeleton body names (matching actual data)
    # Core bodies used by GMR retargeting
    BODY_NAMES = [
        "Hips", "Spine", "Spine1", "Neck", "Head",
        "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand",
        "RightShoulder", "RightArm", "RightForeArm", "RightHand",
        "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase",
        "RightUpLeg", "RightLeg", "RightFoot", "RightToeBase",
    ]

    # Mapping from Motive names to standard names used by GMR
    # OptiTrack Motive uses: Chest, Chest2, LeftCollar, LeftShoulder, LeftElbow, LeftWrist, LeftHip, LeftKnee, LeftAnkle, LeftToe
    # GMR uses: Spine, Spine1, LeftShoulder, LeftArm, LeftForeArm, LeftHand, LeftUpLeg, LeftLeg, LeftFoot, LeftToeBase
    MOTIVE_TO_STANDARD = {
        "Hips": "Hips",
        "Chest": "Spine",
        "Chest2": "Spine1",
        "Neck": "Neck",
        "Head": "Head",
        "LeftCollar": "LeftShoulder",
        "LeftShoulder": "LeftArm",
        "LeftElbow": "LeftForeArm",
        "LeftWrist": "LeftHand",
        "RightCollar": "RightShoulder",
        "RightShoulder": "RightArm",
        "RightElbow": "RightForeArm",
        "RightWrist": "RightHand",
        "LeftHip": "LeftUpLeg",
        "LeftKnee": "LeftLeg",
        "LeftAnkle": "LeftFoot",
        "LeftToe": "LeftToeBase",
        "RightHip": "RightUpLeg",
        "RightKnee": "RightLeg",
        "RightAnkle": "RightFoot",
        "RightToe": "RightToeBase",
    }

    # T-pose offsets in Y-up coordinate system (from actual OptiTrack rb_desc)
    # These are relative offsets from parent joint
    TPOSE_OFFSETS_YUP = {
        "Hips": [0.0, 0.967, 0.0],  # Root position
        "Spine": [0.0, 0.081, 0.0],  # Chest
        "Spine1": [0.0, 0.208, 0.0],  # Chest2
        "Neck": [0.0, 0.228, 0.0],
        "Head": [0.0, 0.154, 0.02],
        "LeftShoulder": [0.04, 0.143, -0.01],  # LeftCollar
        "LeftArm": [0.173, 0.0, 0.0],  # LeftShoulder
        "LeftForeArm": [0.248, 0.0, 0.0],  # LeftElbow
        "LeftHand": [0.239, 0.0, 0.0],  # LeftWrist
        "RightShoulder": [-0.04, 0.143, -0.01],  # RightCollar
        "RightArm": [-0.173, 0.0, 0.0],  # RightShoulder
        "RightForeArm": [-0.248, 0.0, 0.0],  # RightElbow
        "RightHand": [-0.239, 0.0, 0.0],  # RightWrist
        "LeftUpLeg": [0.099, 0.0, 0.0],  # LeftHip
        "LeftLeg": [0.0, -0.441, 0.0],  # LeftKnee
        "LeftFoot": [0.0, -0.446, 0.0],  # LeftAnkle
        "LeftToeBase": [0.0, -0.064, 0.149],  # LeftToe
        "RightUpLeg": [-0.099, 0.0, 0.0],  # RightHip
        "RightLeg": [0.0, -0.441, 0.0],  # RightKnee
        "RightFoot": [0.0, -0.446, 0.0],  # RightAnkle
        "RightToeBase": [0.0, -0.064, 0.149],  # RightToe
    }

    # Coordinate transform: Y-up to Z-up
    # [x, y, z]_yup -> [x, z, y]_zup (swap y and z, but y becomes z)
    # Actually from the data: rotation_matrix = [[0,0,1],[1,0,0],[0,1,0]]
    # new_x = old_z, new_y = old_x, new_z = old_y
    @staticmethod
    def yup_to_zup(pos_yup):
        """Convert position from Y-up to Z-up coordinate system."""
        x, y, z = pos_yup
        return np.array([z, x, y])  # new: [old_z, old_x, old_y]

    def __init__(self):
        rospy.init_node('gmr_sim_node', anonymous=False)

        # Parameters
        self.robot = rospy.get_param('~robot', 'kuavo_s52')
        self.format = rospy.get_param('~format', 'lafan1')
        self.fps = rospy.get_param('~fps', 30.0)
        self.cali_arm_pose_deg = rospy.get_param('~cali_arm_pose_deg', 75.0)
        self.data_file = rospy.get_param('~data_file', '')
        self.enable_viewer = rospy.get_param('~enable_viewer', False)
        self.loop = rospy.get_param('~loop', True)
        self.human_height = rospy.get_param('~human_height', 1.75)

        # Publishers
        self.vmp_pub = rospy.Publisher('/gmr/vmp_input', MocapPoseRetarget, queue_size=10)
        self.frame_pub = rospy.Publisher('/gmr/skeleton_frame', SkeletonFrame, queue_size=10)
        self.rb_desc_pub = rospy.Publisher('/gmr/rigid_body_desc', RigidBodyDescription, queue_size=1, latch=True)

        # State
        self.retarget = None
        self.viewer = None
        self.frame_idx = 0
        self.motion_data = None
        self.prev_root_loc = None
        self.prev_root_rot_wxyz = None
        self.prev_dof_pos = None

        # End effector links
        self.ee_links = {
            'left_foot': 'leg_l6_link',
            'right_foot': 'leg_r6_link',
            'left_hand': 'zarm_l7_link',
            'right_hand': 'zarm_r7_link',
        }

        rospy.loginfo("GMR Simulation Node initialized")
        rospy.loginfo(f"  Robot: {self.robot}")
        rospy.loginfo(f"  FPS: {self.fps}")
        rospy.loginfo(f"  Data file: {self.data_file if self.data_file else 'None (synthetic)'}")

    def generate_tpose_rb_desc(self):
        """Generate T-pose rigid body descriptions in Y-up coordinate system."""
        rb_desc = {}
        # Parent ID map: maps body name to its parent's rigid body ID
        # IDs are 1-indexed matching OptiTrack convention
        # BODY_NAMES order: Hips(1), Spine(2), Spine1(3), Neck(4), Head(5),
        #   LeftShoulder(6), LeftArm(7), LeftForeArm(8), LeftHand(9),
        #   RightShoulder(10), RightArm(11), RightForeArm(12), RightHand(13),
        #   LeftUpLeg(14), LeftLeg(15), LeftFoot(16), LeftToeBase(17),
        #   RightUpLeg(18), RightLeg(19), RightFoot(20), RightToeBase(21)
        parent_id_map = {
            "Hips": 0,  # Root, no parent
            "Spine": 1, "Spine1": 2, "Neck": 3, "Head": 4,
            # Arms: Collar(LeftShoulder) attaches to Spine1(3), not Neck
            "LeftShoulder": 3, "LeftArm": 6, "LeftForeArm": 7, "LeftHand": 8,
            "RightShoulder": 3, "RightArm": 10, "RightForeArm": 11, "RightHand": 12,
            # Legs: attach to Hips(1)
            "LeftUpLeg": 1, "LeftLeg": 14, "LeftFoot": 15, "LeftToeBase": 16,
            "RightUpLeg": 1, "RightLeg": 18, "RightFoot": 19, "RightToeBase": 20,
        }

        for i, name in enumerate(self.BODY_NAMES):
            rid = i + 1
            rb_desc[rid] = {
                'name': name.encode('utf-8'),
                'parent_id': parent_id_map.get(name, 0),
                'offset': self.TPOSE_OFFSETS_YUP.get(name, [0, 0, 0]),
            }
        return rb_desc

    def publish_rb_desc(self, rb_desc):
        """Publish rigid body descriptions."""
        msg = RigidBodyDescription()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "optitrack"

        ids = sorted(rb_desc.keys())
        for rid in ids:
            info = rb_desc[rid]
            msg.ids.append(rid)
            name = info.get('name', b'')
            if isinstance(name, bytes):
                name = name.decode('utf-8', errors='ignore')
            msg.names.append(name)
            msg.parent_ids.append(int(info.get('parent_id', 0)))
            offset = info.get('offset', [0, 0, 0])
            msg.offsets.append(Point(x=offset[0], y=offset[1], z=offset[2]))

        self.rb_desc_pub.publish(msg)
        rospy.loginfo("Published rigid body descriptions (T-pose)")

    def generate_synthetic_frame(self, t):
        """Generate synthetic skeleton frame with simple motion.

        The frame output is in Z-up coordinate system (matching actual OptiTrack streaming data).
        Internally we compute in Y-up (matching rb_desc offsets) then convert.
        """
        frame = {}

        # Simple walking/swaying motion parameters
        sway = 0.02 * np.sin(2 * np.pi * 0.5 * t)
        bob = 0.01 * np.sin(2 * np.pi * 1.0 * t)
        forward = 0.0  # No forward motion for now

        # First compute all positions in Y-up coordinate system
        # Then convert to Z-up for the frame output
        pos_yup = {}

        # Hips position in Y-up: [x, height, z_forward]
        hips_yup = np.array([0.0, 0.967 + bob, forward])
        pos_yup["Hips"] = hips_yup

        # Build skeleton using Y-up offsets
        for name in self.BODY_NAMES[1:]:
            parent_name = self._get_parent_name(name)
            parent_pos = pos_yup.get(parent_name, hips_yup)

            # Get offset in Y-up coordinate system
            offset = np.array(self.TPOSE_OFFSETS_YUP.get(name, [0, 0, 0]))

            # Add some animated motion to arms (in Y-up: Z is forward/back)
            if "Arm" in name or "Hand" in name:
                arm_swing = 0.05 * np.sin(2 * np.pi * 0.5 * t)
                if "Left" in name:
                    offset[2] += arm_swing  # Z-forward swing
                else:
                    offset[2] -= arm_swing

            # Add leg swing (in Y-up: Z is forward/back)
            if "Leg" in name or "Foot" in name:
                leg_swing = 0.03 * np.sin(2 * np.pi * 0.5 * t)
                if "Left" in name:
                    offset[2] += leg_swing
                else:
                    offset[2] -= leg_swing

            pos_yup[name] = parent_pos + offset

        # Convert all positions to Z-up and build frame
        for name in self.BODY_NAMES:
            pos_zup = self.yup_to_zup(pos_yup[name])

            # Rotation: small sway for hips, identity for others
            if name == "Hips":
                # Sway rotation around Z-up axis (yaw)
                rot = R.from_euler('xyz', [0, 0, sway * 5], degrees=True).as_quat()
            else:
                rot = R.from_euler('xyz', [0, 0, 0], degrees=True).as_quat()

            rot_wxyz = np.array([rot[3], rot[0], rot[1], rot[2]])
            frame[name] = (pos_zup, rot_wxyz)

        return frame

    def _get_parent_name(self, name):
        """Get parent body name.

        Based on actual OptiTrack Motive skeleton hierarchy:
        - Hips -> Chest(Spine) -> Chest2(Spine1) -> Neck -> Head
        - Chest2(Spine1) -> LeftCollar(LeftShoulder) -> LeftShoulder(LeftArm) -> ...
        - Hips -> LeftHip(LeftUpLeg) -> LeftKnee(LeftLeg) -> ...
        """
        parent_map = {
            "Spine": "Hips", "Spine1": "Spine", "Neck": "Spine1", "Head": "Neck",
            # Arms: Collar attaches to Spine1 (Chest2), not Neck
            "LeftShoulder": "Spine1", "LeftArm": "LeftShoulder",
            "LeftForeArm": "LeftArm", "LeftHand": "LeftForeArm",
            "RightShoulder": "Spine1", "RightArm": "RightShoulder",
            "RightForeArm": "RightArm", "RightHand": "RightForeArm",
            # Legs: attach to Hips
            "LeftUpLeg": "Hips", "LeftLeg": "LeftUpLeg",
            "LeftFoot": "LeftLeg", "LeftToeBase": "LeftFoot",
            "RightUpLeg": "Hips", "RightLeg": "RightUpLeg",
            "RightFoot": "RightLeg", "RightToeBase": "RightFoot",
        }
        return parent_map.get(name, "Hips")

    def load_motion_data(self):
        """Load motion data from file if specified.

        Supported formats:
        - .pkl with 'optitrack_streaming' format: contains rb_desc and frames
        - .pkl with robot motion data
        - .bvh files
        """
        if not self.data_file or not os.path.exists(self.data_file):
            rospy.loginfo("No data file specified, using synthetic motion")
            return None

        ext = os.path.splitext(self.data_file)[1].lower()

        if ext == '.pkl':
            import pickle
            with open(self.data_file, 'rb') as f:
                data = pickle.load(f)

            # Check if it's optitrack_streaming format
            if isinstance(data, dict) and data.get('format') == 'optitrack_streaming':
                rospy.loginfo(f"Loaded OptiTrack streaming data from {self.data_file}")
                rospy.loginfo(f"  Contains {len(data.get('frames', []))} frames")
                rospy.loginfo(f"  FPS: {data.get('fps', 30.0)}")
                # Update FPS from file if available
                if 'fps' in data:
                    self.fps = data['fps']
                return data
            else:
                rospy.loginfo(f"Loaded motion data from {self.data_file}")
                return data
        elif ext == '.bvh':
            from utils.bvh_loader import load_bvh_file
            data = load_bvh_file(self.data_file, format=self.format)
            rospy.loginfo(f"Loaded BVH data from {self.data_file}")
            return data
        else:
            rospy.logwarn(f"Unsupported file format: {ext}")
            return None

    def get_frame_from_data(self, frame_idx):
        """Get a frame from loaded motion data.

        Returns frame dict: {body_name: (pos_array, quat_wxyz_array)}
        """
        if self.motion_data is None:
            return None

        # Handle optitrack_streaming format
        if isinstance(self.motion_data, dict) and self.motion_data.get('format') == 'optitrack_streaming':
            frames = self.motion_data.get('frames', [])
            if not frames:
                return None

            # Handle looping
            if self.loop:
                idx = frame_idx % len(frames)
            else:
                idx = min(frame_idx, len(frames) - 1)

            return frames[idx]

        return None

    def publish_skeleton_frame(self, frame):
        """Publish skeleton frame."""
        msg = SkeletonFrame()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "optitrack"

        for name, (pos, quat_wxyz) in frame.items():
            rb = RigidBodyFrame()
            rb.name = name
            rb.position = Point(x=pos[0], y=pos[1], z=pos[2])
            rb.orientation = Quaternion(
                x=quat_wxyz[1], y=quat_wxyz[2], z=quat_wxyz[3], w=quat_wxyz[0]
            )
            msg.rigid_bodies.append(rb)

        self.frame_pub.publish(msg)

    def publish_vmp_input(self, qpos, root_loc, root_rot_wxyz, dof_pos):
        """Publish VMP input data."""
        msg = MocapPoseRetarget()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"

        # Base link pose
        msg.base_link_pose = Pose()
        msg.base_link_pose.position = Point(x=root_loc[0], y=root_loc[1], z=root_loc[2])
        msg.base_link_pose.orientation = Quaternion(
            x=root_rot_wxyz[1], y=root_rot_wxyz[2], z=root_rot_wxyz[3], w=root_rot_wxyz[0]
        )

        # Velocities
        if self.prev_root_loc is not None:
            v_lin = (root_loc - self.prev_root_loc) * self.fps
            prev_quat_xyzw = self.prev_root_rot_wxyz[[1, 2, 3, 0]]
            curr_quat_xyzw = root_rot_wxyz[[1, 2, 3, 0]]
            dR = R.from_quat(curr_quat_xyzw) * R.from_quat(prev_quat_xyzw).inv()
            w_ang = dR.as_rotvec() * self.fps
            dof_vel = (dof_pos - self.prev_dof_pos) * self.fps
        else:
            v_lin = np.zeros(3)
            w_ang = np.zeros(3)
            dof_vel = np.zeros(27)

        msg.base_velocity = np.concatenate([v_lin, w_ang]).tolist()
        msg.joint_position = dof_pos.tolist()
        msg.joint_velocity = dof_vel.tolist()

        # End effector positions
        self.retarget.configuration.update(q=qpos)
        ee_positions = []
        for ee_name in ['left_foot', 'right_foot', 'left_hand', 'right_hand']:
            link_name = self.ee_links[ee_name]
            se = self.retarget.configuration.get_transform_frame_to_world(link_name, frame_type="body")
            pos = se.translation()
            ee_positions.append(Point(x=pos[0], y=pos[1], z=pos[2]))
        msg.end_effector_poses = ee_positions

        self.vmp_pub.publish(msg)

    def run(self):
        """Main loop."""
        # Load motion data first (to get rb_desc if available)
        self.motion_data = self.load_motion_data()

        # Get rb_desc: from loaded data or generate synthetic
        if (self.motion_data is not None and
            isinstance(self.motion_data, dict) and
            'rb_desc' in self.motion_data):
            rb_desc = self.motion_data['rb_desc']
            rospy.loginfo("Using rb_desc from loaded data file")
        else:
            rb_desc = self.generate_tpose_rb_desc()
            rospy.loginfo("Using synthetic rb_desc")

        self.publish_rb_desc(rb_desc)

        # Create calibration pose from rb_desc
        from utils.motivestreaming import cali_pose_from_rb_desc_
        cali_pose, human_height = cali_pose_from_rb_desc_(
            rb_desc=rb_desc,
            arm_pose_deg=self.cali_arm_pose_deg
        )
        if human_height is None:
            human_height = self.human_height

        rospy.loginfo(f"Calibration ready, human height: {human_height:.3f}m")

        # Initialize retargeting
        rospy.loginfo("Initializing motion retargeting...")
        self.retarget = VMR_bvh(
            src_human=f"bvh_{self.format}",
            tgt_robot=self.robot,
            actual_human_height=human_height,
            cali_pose=cali_pose,
        )

        # Initialize viewer if enabled
        if self.enable_viewer:
            self.viewer = RobotMotionViewer(robot_type=self.robot, motion_fps=self.fps)

        rospy.loginfo(f"Starting simulation at {self.fps} FPS...")
        rate = rospy.Rate(self.fps)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            # Get current time
            t = (rospy.Time.now() - start_time).to_sec()

            # Generate or read frame
            frame = self.get_frame_from_data(self.frame_idx)
            if frame is not None:
                # Using loaded motion data
                self.frame_idx += 1
            else:
                # Fallback to synthetic motion
                frame = self.generate_synthetic_frame(t)

            # Publish skeleton frame
            self.publish_skeleton_frame(frame)

            # Retarget
            qpos = self.retarget.retarget(frame)

            if not hasattr(self.retarget, "scaled_human_data"):
                rate.sleep()
                continue

            # Extract components
            qpos_arr = np.asarray(qpos, dtype=np.float64)
            root_loc = qpos_arr[:3]
            root_rot_wxyz = qpos_arr[3:7]
            dof_pos = qpos_arr[7:-2]  # 27 DOF, exclude head

            # Publish VMP input
            self.publish_vmp_input(qpos, root_loc, root_rot_wxyz, dof_pos)

            # Update viewer
            if self.viewer is not None:
                self.viewer.step(
                    root_pos=qpos[:3],
                    root_rot=qpos[3:7],
                    dof_pos=qpos[7:],
                    human_motion_data=self.retarget.scaled_human_data,
                    rate_limit=False,
                )

            # Update state
            self.prev_root_loc = root_loc.copy()
            self.prev_root_rot_wxyz = root_rot_wxyz.copy()
            self.prev_dof_pos = dof_pos.copy()

            rate.sleep()


def main():
    try:
        node = GMRSimNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
