import math
import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from drake_trans import *
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from noitom_hi5_hand_udp_python.msg import PoseInfo, PoseInfoList, JoySticks
from kuavo_msgs.msg import headBodyPose
import rospy
import time
import tf
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_from_matrix, quaternion_matrix
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from constanst import gesture_corresponding_hand_position
# to dev
hand_thumb_pub = rospy.Publisher('/hand_thumb', Float32MultiArray, queue_size=10)

# Array of bone names
bone_names = [
    "LeftArmUpper",
    "LeftArmLower",
    "RightArmUpper",
    "RightArmLower",
    "LeftHandPalm",
    "RightHandPalm",
    "LeftHandThumbMetacarpal",
    "LeftHandThumbProximal",
    "LeftHandThumbDistal",
    "LeftHandThumbTip",
    "LeftHandIndexTip",
    "LeftHandMiddleTip",
    "LeftHandRingTip",
    "LeftHandLittleTip",
    "RightHandThumbMetacarpal",
    "RightHandThumbProximal",
    "RightHandThumbDistal",
    "RightHandThumbTip",
    "RightHandIndexTip",
    "RightHandMiddleTip",
    "RightHandRingTip",
    "RightHandLittleTip",
    "Root",  # Added new bone
    "Chest"  # Added new bone
]

FINGER_PARTS = [
    "HandPalm",
    "HandThumbMetacarpal",
    "HandThumbProximal",
    "HandThumbDistal",
    "HandThumbTip",
    "HandIndexTip",
    "HandMiddleTip",
    "HandRingTip",
    "HandLittleTip"
]


# Create bone_name_to_index dictionary
bone_name_to_index = {name: index for index, name in enumerate(bone_names)}

# List of bone IDs in order
fullBodyBoneIds_leju_arms = [bone_name_to_index[name] for name in bone_names]

# Create a reverse mapping from index to bone name
index_to_bone_name = {index: bone_names[index] for index in range(len(bone_names))}

bias_chest_to_base_link = [0.0, 0, 0.23]

class HeadBodyPose:
    head_pitch = 0.0
    head_yaw = 0.0
    body_yaw = 0.0
    body_x = 0.0
    body_y = 0.0
    body_pitch = 6 * np.pi / 180.0
    body_height = 0.74


class Quest3ArmInfoTransformer:
    def __init__(self, model_path, vis_pub=True, predict_gesture=False, eef_visual_stl_files=None):
        self.predict_gesture = predict_gesture
        self.model_path = model_path
        self.vis_pub = vis_pub
        self.left_finger_joints = None # down dom to 6 dof
        self.right_finger_joints = None
        self.left_hand_pose = None # pos + quaternion
        self.right_hand_pose = None
        self.left_elbow_pos = None
        self.right_elbow_pos = None
        self.pose_info_list = None # PoseInfoList message
        self.timestamp_ms = 0.0
        self.is_high_confidence = True
        self.is_hand_tracking = False
        self.ok_gesture_counts = 0
        self.shot_gesture_counts = 0
        self.is_runing = False     
        self.control_torso = False   
        self.init_R_wC = rpy_to_matrix([np.pi/2, 0, np.pi/2]) # rotation matrix from world to chest
        self.init_R_wLS = None # rotation matrix from world to left shoulder
        self.init_R_wRS = None # rotation matrix from world to right shoulder
        self.left_shoulder_rpy_in_robot = [0.0, 0.0, 0.0] # expressed in robot frame
        self.right_shoulder_rpy_in_robot = [0.0, 0.0, 0.0]
        self.chest_axis_agl = [0.0, 0.0, 0.0]  # in robot frame
        self.upper_arm_length = 22.0
        self.lower_arm_length = 19.0
        self.base_shoulder_x_bias = 0.0
        self.base_shoulder_y_bias = 0.15
        self.base_shoulder_z_bias = 0.42
        self.shoulder_width = 0.29
        self.head_body_pose = HeadBodyPose()
        # Add new variables for arm length measurement
        self.measure_arm_length = True
        self.arm_length_num = 30
        self.left_upper_arm_lengths = []
        self.left_lower_arm_lengths = []
        self.right_upper_arm_lengths = []
        self.right_lower_arm_lengths = []
        self.avg_left_upper_arm_length = None
        self.avg_left_lower_arm_length = None
        self.avg_right_upper_arm_length = None
        self.avg_right_lower_arm_length = None
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        # self.marker_pub_human = rospy.Publisher("visualization_marker/result", Marker, queue_size=10)
        self.marker_pub_right = rospy.Publisher("visualization_marker_right", Marker, queue_size=10)
        # self.marker_pub_human_right = rospy.Publisher("visualization_marker_human_right", Marker, queue_size=10)
        self.marker_pub_elbow = rospy.Publisher("visualization_marker/elbow", Marker, queue_size=10)
        self.marker_pub_elbow_right = rospy.Publisher("visualization_marker_right/elbow", Marker, queue_size=10)
        self.marker_pub_shoulder = rospy.Publisher("visualization_marker/shoulder", Marker, queue_size=10)
        # self.marker_pub_shoulder_human = rospy.Publisher("visualization_marker/shoulder_human", Marker, queue_size=10)
        self.marker_pub_shoulder_right = rospy.Publisher("visualization_marker_right/shoulder", Marker, queue_size=10)
        # self.marker_pub_shoulder_human_right = rospy.Publisher("visualization_marker_right/shoulder_human", Marker, queue_size=10)
        self.marker_pub_shoulder_quest3 = rospy.Publisher("visualization_marker/shoulder_quest3", Marker, queue_size=10)
        self.marker_pub_shoulder_quest3_right = rospy.Publisher("visualization_marker_right/shoulder_quest3", Marker, queue_size=10)
        self.marker_pub_chest = rospy.Publisher("visualization_marker_chest", Marker, queue_size=10)
        self.joint_state_puber = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.shoulder_angle_puber = rospy.Publisher('/quest3_debug/shoulder_angle', Float32MultiArray, queue_size=10)
        self.chest_axis_puber = rospy.Publisher('/quest3_debug/chest_axis', Float32MultiArray, queue_size=10)
        self.head_body_pose_puber = rospy.Publisher('/kuavo_head_body_orientation', headBodyPose, queue_size=10)
        self.left_joystick = None
        self.right_joystick = None
        if eef_visual_stl_files is not None:
            self.eef_visual_stl_files = eef_visual_stl_files
        if self.predict_gesture:
            from hand_gesture_predictor import HandGesturePredictor
            hand_gesture_model_path = os.path.join(current_dir, "hand_gesture_model.onnx")
            self.hand_gesture_predictor = HandGesturePredictor(hand_gesture_model_path)
        self.listener = tf.TransformListener()
        if rospy.has_param("/quest3/upper_arm_length"):
            self.upper_arm_length = rospy.get_param("/quest3/upper_arm_length")/100.0
            print(f"get rosparams upper_arm_length: {self.upper_arm_length}")
        if rospy.has_param("/quest3/lower_arm_length"):
            self.lower_arm_length = rospy.get_param("/quest3/lower_arm_length")/100.0
            print(f"get rosparams lower_arm_length: {self.lower_arm_length}")
        if rospy.has_param("/quest3/base_shoulder_x_bias"):
            self.base_shoulder_x_bias = rospy.get_param("/quest3/base_shoulder_x_bias")
            print(f"get rosparams base_shoulder_x_bias: {self.base_shoulder_x_bias}")
        if rospy.has_param("/quest3/base_shoulder_y_bias"):
            self.base_shoulder_y_bias = rospy.get_param("/quest3/base_shoulder_y_bias")
            print(f"get rosparams base_shoulder_y_bias: {self.base_shoulder_y_bias}")
        if rospy.has_param("/quest3/base_shoulder_z_bias"):
            self.base_shoulder_z_bias = rospy.get_param("/quest3/base_shoulder_z_bias")
            print(f"get rosparams base_shoulder_z_bias: {self.base_shoulder_z_bias}")
        if rospy.has_param("/quest3/shoulder_width"):
            self.shoulder_width = rospy.get_param("/quest3/shoulder_width")/100.0
            print(f"get rosparams shoulder_width: {self.shoulder_width}")
        delta_z = self.base_shoulder_z_bias - 0.42
        bias_chest_to_base_link[2] += delta_z
              
    def read_joySticks_msg(self, msg):
        self.left_joystick = [msg.left_trigger, msg.left_grip]
        self.right_joystick = [msg.right_trigger, msg.right_grip]

    def read_msg(self, msg):
        """
        Read the PoseInfoList message and extract the left and right finger joint angles.
        """
        if self.left_joystick is None:
            print("No joystick message received yet")
            return
        self.pose_info_list = msg.poses
        self.timestamp_ms = msg.timestamp_ms
        self.is_high_confidence = msg.is_high_confidence
        self.is_hand_tracking = msg.is_hand_tracking
        # print("is_high_confidence: {}, is_hand_tracking: {}".format(self.is_high_confidence, self.is_hand_tracking))
        if self.is_hand_tracking:
            self.compute_finger_joints("Left")
            self.compute_finger_joints("Right")
        else:
            self.compute_finger_joints_joy("Left")
            self.compute_finger_joints_joy("Right")
        self.compute_hand_pose("Left")
        self.compute_hand_pose("Right")
        # gesture counts
        max_counts = 50
        if(self.is_runing_gesture()):
            self.ok_gesture_counts += 1
            self.shot_gesture_counts = 0
            if(self.ok_gesture_counts >= max_counts):
                self.is_runing = True
        elif(self.is_stop_gesture()):
            self.shot_gesture_counts += 1
            self.ok_gesture_counts = 0
            if(self.shot_gesture_counts >= max_counts):
                self.is_runing = False
        if self.is_runing and self.control_torso:
            self.pub_head_body_pose_msg(self.head_body_pose)


    def get_relative_finger_poses(self, side):
        """
        Get relative finger poses from TF and convert to array format directly
        Args:
            side: 'Left' or 'Right', indicating hand side
        Returns:
            frame_poses: numpy array of length 63 (9 transforms * (3 position + 4 quaternion))
        """
        try:
            frame_poses = np.zeros(63)
            parent_frame = side + "HandPalm"
            
            for i, part in enumerate(FINGER_PARTS):
                try:
                    lookup_start = time.time()
                    child_frame = side + part
                    position, rotation = self.listener.lookupTransform(
                        parent_frame, child_frame, rospy.Time(0)
                    )
                    
                    # Mirror position for right hand
                    if side == "Right":
                        position = [-p for p in position]
                    
                    # Directly fill the array with position and rotation
                    start_idx = i * 7
                    frame_poses[start_idx:start_idx+3] = position
                    frame_poses[start_idx+3:start_idx+7] = rotation
                    
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logerr(f"TF lookup failed for {child_frame}: {e}")
                    continue
            
            return frame_poses
            
        except Exception as e:
            rospy.logerr(f"Error in get_relative_finger_poses: {e}")
            return np.zeros(63)

    def compute_finger_joints(self, side):
        if self.predict_gesture:
            gesture, confidence, inference_time = self.hand_gesture_predictor.predict(self.get_relative_finger_poses(side))
            if gesture in gesture_corresponding_hand_position:
                hand_position = gesture_corresponding_hand_position[gesture]
                finger_joints = [1.7*hand_position[i]/100.0 for i in range(6)]
                if side == "Left":
                    self.left_finger_joints = finger_joints
                elif side == "Right":
                    self.right_finger_joints = finger_joints
                return finger_joints
        """
        y轴从手腕指向中指指尖, x轴从拇指位置指向无名指方向
        """
        if self.pose_info_list is None:
            return None
        T_hand = self.pose_info2_transform(self.pose_info_list[bone_name_to_index[side + "HandPalm"]])
        T_finger_thumb_tip = self.pose_info2_transform(self.pose_info_list[bone_name_to_index[side + "HandThumbTip"]])
        finger_index_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandIndexTip"]].orientation
        finger_middle_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandMiddleTip"]].orientation
        finger_ring_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandRingTip"]].orientation
        finger_little_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandLittleTip"]].orientation
        R0_inv = T_hand[:3, :3].T
        finger_index_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_index_tip_ori))
        finger_middle_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_middle_tip_ori))
        finger_ring_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_ring_tip_ori))
        finger_little_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_little_tip_ori))
        # print(f"index: {finger_index_tip_rpy[2]}, middle: {finger_middle_tip_rpy[2]}")
        # finger_joints = [finger_thumb_metacarpal_rpy[2], finger_thumb_proximal_rpy[2], finger_thumb_distal_rpy[2], finger_thumb_tip_rpy[2], 

        p_hf_w = T_finger_thumb_tip[:3, 3] - T_hand[:3, 3]
        p_hf_h = R0_inv @ p_hf_w
        x, z = 0.0, 0.0
        if side == "Left":
            # x:[-0.075, 0]->[open, close]->[0, 100]
            # z:[0.044, 0.08]->[close, open]->[100, 0]
            x_min, x_max = -0.07, -0.01
            z_min, z_max = 0.02, 0.08
            x = min(x_max, max(x_min, p_hf_h[0]))
            z = min(z_max, max(z_min, p_hf_h[2]))
            x = 100.0 * (x - x_min) / (x_max - x_min)
            z = 100.0 * (z - z_min) / (z_max - z_min)
            # print(f"x: {x}")
            
        if side == "Right":
            # x:[0, 0.075]->[close, open]->[100, 0]
            # z:[-0.08, -0.044]->[open, close]
            x_min, x_max = -0.01, 0.06
            z_min, z_max = -0.08, -0.05
            x = min(x_max, max(x_min, p_hf_h[0]))
            z = min(z_max, max(z_min, p_hf_h[2]))
            x = 100 - 100.0 * (x - x_min) / (x_max - x_min)
            z = 100 - 100.0 * (z - z_min) / (z_max - z_min)

        finger_joints = [1.7*x/100.0, 1.7*z/100.0, # TODO: fix thumb finger joints
                finger_index_tip_rpy[0], finger_middle_tip_rpy[0], finger_ring_tip_rpy[0], finger_little_tip_rpy[0]]
        for i in range(len(finger_joints)):
            finger_joints[i] = self.limit_finger_angle(finger_joints[i], back_agl=np.pi/2.0)
        if side == "Left":
            self.left_finger_joints = finger_joints        
            thumb_msg = Float32MultiArray()
            thumb_msg.data = p_hf_h
            hand_thumb_pub.publish(thumb_msg)
        elif side == "Right":
            self.right_finger_joints = finger_joints
        return finger_joints
 
    def compute_finger_joints_joy(self, side):
        """
        手柄版本的计算手指关节角度
        """
        if self.pose_info_list is None:
            return None
        hand_ori = self.pose_info_list[bone_name_to_index[side + "HandPalm"]].orientation
        finger_thumb_metacarpal_ori = self.pose_info_list[bone_name_to_index[side + "HandThumbMetacarpal"]].orientation
        finger_thumb_proximal_ori = self.pose_info_list[bone_name_to_index[side + "HandThumbProximal"]].orientation
        finger_thumb_distal_ori = self.pose_info_list[bone_name_to_index[side + "HandThumbDistal"]].orientation
        finger_thumb_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandThumbTip"]].orientation
        finger_index_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandIndexTip"]].orientation
        finger_middle_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandMiddleTip"]].orientation
        finger_ring_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandRingTip"]].orientation
        finger_little_tip_ori = self.pose_info_list[bone_name_to_index[side + "HandLittleTip"]].orientation
        R0_inv = self.quaternion_msg_to_matrix(hand_ori).T
        finger_thumb_metacarpal_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_thumb_metacarpal_ori))
        finger_thumb_proximal_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_thumb_proximal_ori))
        finger_thumb_distal_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_thumb_distal_ori))
        finger_thumb_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_thumb_tip_ori))
        finger_index_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_index_tip_ori))
        finger_middle_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_middle_tip_ori))
        finger_ring_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_ring_tip_ori))
        finger_little_tip_rpy = matrix_to_rpy(R0_inv @ self.quaternion_msg_to_matrix(finger_little_tip_ori))
        # print(f"index: {finger_index_tip_rpy[2]}, middle: {finger_middle_tip_rpy[2]}")
        # finger_joints = [finger_thumb_metacarpal_rpy[2], finger_thumb_proximal_rpy[2], finger_thumb_distal_rpy[2], finger_thumb_tip_rpy[2], 

        finger_joints = [0.0 for _ in range(6)]
        finger_joints[0] = self.limit_finger_angle(finger_index_tip_rpy[0])
        finger_joints[1] = 1.7
        finger_joints[2] = self.limit_finger_angle(finger_index_tip_rpy[0])
        finger_joints[3] = 0.0 if finger_middle_tip_rpy[0] > 0.0 else 1.7
        finger_joints[4] = 0.0 if finger_ring_tip_rpy[0] > 0.0 else 1.7
        finger_joints[5] = 0.0 if finger_little_tip_rpy[0] > 0.0 else 1.7

        if side == "Left":
            self.left_finger_joints = finger_joints
        elif side == "Right":
            self.right_finger_joints = finger_joints
        return finger_joints

    def get_finger_joints(self, side):
        if side == "Left":
            return self.left_finger_joints
        elif side == "Right":
            return self.right_finger_joints
        else:
            print("Invalid side: {}".format(side))
            return None

    @staticmethod
    def limit_finger_angle(angle, min_angle=0.0, max_angle=1.7, back_agl=1/4*np.pi):
        """
        finger joint angle might in the range of [-3.14, 3.14]
        Limit the finger joint angles to within the range [0.0, 1.7].
        back_agl: VR中手指可能存在反向的角度, 这里设置一个阈值, 超过这个阈值, 才认为是反向的, 否则认为是0
        """
        res = angle
        if res < 0.0:
            res += 2.0 * np.pi
        res = np.fmod(res, 2.0 * np.pi)
        if res < min_angle:
            res = min_angle
        if res > max_angle:
            if res < 2*np.pi - back_agl:
                res = max_angle
            else:  # res > 3/2*np.pi and res <= 2*np.pi
                res = 0.0
        return res

    def is_runing_gesture(self):
        """
        ok-ok:->Runing
        """
        is_runing = True
        is_runing &= self.ok_gesture_check(self.left_finger_joints)
        is_runing &= self.ok_gesture_check(self.right_finger_joints)

        # 兼容手柄的情况
        is_joyRun = True
        is_joyRun &= self.joy_ok_gesture_check(self.left_joystick)
        is_joyRun &= self.joy_ok_gesture_check(self.right_joystick)

        # Calculate average arm lengths when OK gesture is detected
        if (is_runing or is_joyRun) and self.measure_arm_length:
            if len(self.left_upper_arm_lengths) > 0:
                self.avg_left_upper_arm_length = sum(self.left_upper_arm_lengths) / len(self.left_upper_arm_lengths)
                self.avg_left_lower_arm_length = sum(self.left_lower_arm_lengths) / len(self.left_lower_arm_lengths)
            if len(self.right_upper_arm_lengths) > 0:
                self.avg_right_upper_arm_length = sum(self.right_upper_arm_lengths) / len(self.right_upper_arm_lengths)
                self.avg_right_lower_arm_length = sum(self.right_lower_arm_lengths) / len(self.right_lower_arm_lengths)
            self.measure_arm_length = False
            print(f"Arm length measurement completed. Left upper: {self.avg_left_upper_arm_length:.2f}, lower: {self.avg_left_lower_arm_length:.2f}, "
                  f"Right upper: {self.avg_right_upper_arm_length:.2f}, lower: {self.avg_right_lower_arm_length:.2f}")

        return is_runing or is_joyRun

    def is_stop_gesture(self):
        """
        shot-shot:->Stop
        """
        is_stop = True
        is_stop &= self.shot_gesture_check(self.left_finger_joints)
        is_stop &= self.shot_gesture_check(self.right_finger_joints)

        # 兼容手柄的情况
        is_joyStop = True
        is_joyStop &= self.joy_shot_gesture_check(self.left_joystick)
        is_joyStop &= self.joy_shot_gesture_check(self.right_joystick)

        if is_stop or is_joyStop:
            self.measure_arm_length = True

        return is_stop or is_joyStop

    @staticmethod
    def ok_gesture_check(finger_pos, min_agl=0.5, max_agl=1.0):
        """
        finger_pos: list of 6 float, [thumb1, thumb2, index, middle, ring, pinky]
        """
        is_ok = True
        is_ok &= (finger_pos[2] > max_agl)
        is_ok &= (finger_pos[3] < min_agl)
        is_ok &= (finger_pos[4] < min_agl)
        return is_ok    

    @staticmethod
    def shot_gesture_check(finger_pos, min_agl=0.5, max_agl=1.0):
        """
        finger_pos: list of 6 float, [thumb1, thumb2, index, middle, ring, pinky]
        """
        is_shot = True
        is_shot &= (finger_pos[2] < min_agl)
        is_shot &= (finger_pos[3] > max_agl)
        is_shot &= (finger_pos[4] > max_agl)
        return is_shot
    
    @staticmethod
    def joy_shot_gesture_check(stick_pos, min_agl=0.5, max_agl=0.8):
        """
        stick_pos : [trigger, grip]
        """
        is_shot = True
        is_shot &= (stick_pos[0] < min_agl)
        is_shot &= (stick_pos[1] > max_agl)
        return is_shot    

    @staticmethod
    def joy_ok_gesture_check(stick_pos):
        """
        stick_pos : [trigger, grip]
        """
        is_ok = True
        is_ok &= (stick_pos[0] > 0.5)
        return is_ok


    @staticmethod
    def quaternion_msg_to_matrix(ori):
        """
        Convert a quaternion to roll-pitch-yaw angles.
        """
        quat = [ori.x, ori.y, ori.z, ori.w]
        mat = quaternion_to_matrix(quat)
        return mat

    @staticmethod
    def quaternion_msg_to_RPY(ori):
        """
        Convert a quaternion to roll-pitch-yaw angles.
        """
        quat = [ori.x, ori.y, ori.z, ori.w]
        rpy = quaternion_to_RPY(quat)
        return rpy

    def get_hand_pose(self, side):
        if side == "Left":
            return self.left_hand_pose, self.left_elbow_pos
        elif side == "Right":
            return self.right_hand_pose, self.right_elbow_pos
        else:
            print("Invalid side: {}".format(side))
            return None, None

    def compute_shoudler_pose(self, R_wS, side):
        """
        R_wS: 3x3 rotation matrix from world to <side> shoulder
        """
        if side == "Left":
            R_01 = self.init_R_wLS.T @ R_wS
            # align to urdf model, the shoulder angle is negative
            self.left_shoulder_rpy_in_robot[0] = matrix_to_rpy(R_01)[2]
            self.left_shoulder_rpy_in_robot[1] = -matrix_to_rpy(R_01)[0]
            self.left_shoulder_rpy_in_robot[2] = -matrix_to_rpy(R_01)[1]
        elif side == "Right":
            R_01 = self.init_R_wRS.T @ R_wS
            # align to urdf model, the shoulder angle is positive
            self.right_shoulder_rpy_in_robot[0] = -matrix_to_rpy(R_01)[2]
            self.right_shoulder_rpy_in_robot[1] = -matrix_to_rpy(R_01)[0]
            self.right_shoulder_rpy_in_robot[2] = matrix_to_rpy(R_01)[1]
        msg = Float32MultiArray()
        msg.data = np.append(self.left_shoulder_rpy_in_robot, self.right_shoulder_rpy_in_robot)
        self.shoulder_angle_puber.publish(msg)

    def scale_arm_positions(self, shoulder_pos, elbow_pos, hand_pos, human_shoulder_pos, side):
        """
        Scale the arm positions from human scale to robot scale
        Args:
            shoulder_pos: robot shoulder position
            elbow_pos: robot elbow position
            hand_pos: robot hand position
            human_shoulder_pos: human shoulder position
            side: "Left" or "Right"
        Returns:
            scaled_elbow_pos: scaled elbow position
            scaled_hand_pos: scaled hand position
        """
        human_upper_arm_length = math.sqrt((elbow_pos[0] - shoulder_pos[0])**2 + (elbow_pos[1] - shoulder_pos[1])**2 + (elbow_pos[2] - shoulder_pos[2])**2)
        human_lower_arm_length = math.sqrt((hand_pos[0] - elbow_pos[0])**2 + (hand_pos[1] - elbow_pos[1])**2 + (hand_pos[2] - elbow_pos[2])**2)

        # Collect arm length data if measurement is active
        if self.measure_arm_length:
            if side == "Left":
                self.left_upper_arm_lengths.append(human_upper_arm_length)
                self.left_lower_arm_lengths.append(human_lower_arm_length)
                if len(self.left_upper_arm_lengths) > self.arm_length_num:
                    self.left_upper_arm_lengths.pop(0)
                    self.left_lower_arm_lengths.pop(0)
            else:
                self.right_upper_arm_lengths.append(human_upper_arm_length)
                self.right_lower_arm_lengths.append(human_lower_arm_length)
                if len(self.right_upper_arm_lengths) > self.arm_length_num:
                    self.right_upper_arm_lengths.pop(0)
                    self.right_lower_arm_lengths.pop(0) 
            # Use the latest measurements for scaling
            radi1 = self.upper_arm_length/human_upper_arm_length
            radi2 = self.lower_arm_length/human_lower_arm_length
        else:
            # Use average measurements for scaling
            if side == "Left":
                radi1 = self.upper_arm_length/self.avg_left_upper_arm_length
                # radi1 = self.upper_arm_length/self.left_upper_arm_lengths[-1]
                radi2 = (self.lower_arm_length + self.upper_arm_length)/(self.avg_left_lower_arm_length + self.avg_left_upper_arm_length)
                # radi2 = (self.lower_arm_length + self.upper_arm_length)/(self.left_lower_arm_lengths[-1] + self.left_upper_arm_lengths[-1])
            else:
                radi1 = self.upper_arm_length/self.avg_right_upper_arm_length
                # radi1 = self.upper_arm_length/self.right_upper_arm_lengths[-1]
                radi2 = (self.lower_arm_length + self.upper_arm_length)/(self.avg_right_lower_arm_length + self.avg_right_upper_arm_length)
                # radi2 = (self.lower_arm_length + self.upper_arm_length)/(self.right_lower_arm_lengths[-1] + self.right_upper_arm_lengths[-1])
        
        scaled_elbow_pos = np.zeros(3)
        scaled_hand_pos = np.zeros(3)
        for i in range(3):
            scaled_elbow_pos[i] = shoulder_pos[i] + radi1 * (elbow_pos[i] - human_shoulder_pos[i])
            # scaled_hand_pos[i] = scaled_elbow_pos[i] + (hand_pos[i] - (human_shoulder_pos[i] + (elbow_pos[i] - shoulder_pos[i]) / radi1))*radi2
            # scaled_hand_pos[i] = shoulder_pos[i] + radi2 * (hand_pos[i] - human_shoulder_pos[i])
            scaled_hand_pos[i] = radi2 * (hand_pos[i])


        return scaled_elbow_pos, scaled_hand_pos

    def compute_hand_pose(self, side):
        chest_pose = self.pose_info_list[bone_name_to_index["Chest"]]
        elbow_pose = self.pose_info_list[bone_name_to_index[side + "ArmLower"]]
        shoulder_pose = self.pose_info_list[bone_name_to_index[side + "ArmUpper"]]
        hand_pose = self.pose_info_list[bone_name_to_index[side + "HandPalm"]]

        vr_quat = [hand_pose.orientation.x, hand_pose.orientation.y, hand_pose.orientation.z, hand_pose.orientation.w]
        hand_quat_in_w = vr_quat2robot_quat(vr_quat, side, 15*np.pi/180.0 if not self.is_hand_tracking else 0.0) # [x, y, z, w]

        T_wChest = self.pose_info2_transform(chest_pose)
        T_wElbow = self.pose_info2_transform(elbow_pose)
        T_wHand = self.pose_info2_transform(hand_pose)
        axis, angle = matrix_to_axis_angle(self.init_R_wC.T @ T_wChest[:3, :3])
        chest_rpy = matrix_to_rpy(self.init_R_wC.T @ T_wChest[:3, :3])
        self.chest_axis_agl = [0, 0, axis[1]]
        self.head_body_pose.body_yaw = axis[1]
        R_wChest_rm_yaw = axis_angle_to_matrix(self.chest_axis_agl).T @ T_wChest[:3, :3]
        self.head_body_pose.body_pitch = matrix_to_rpy(self.init_R_wC.T @ R_wChest_rm_yaw)[0]
        self.head_body_pose.body_x = chest_pose.position.x
        self.head_body_pose.body_y = chest_pose.position.y
        self.head_body_pose.body_height = chest_pose.position.z

        hand_mat_cH = axis_angle_to_matrix(self.chest_axis_agl).T @ quaternion_to_matrix(hand_quat_in_w)
        hand_quat = matrix_to_quaternion(hand_mat_cH)
        chest_axis_msg = Float32MultiArray()
        chest_axis_msg.data = axis
        self.chest_axis_puber.publish(chest_axis_msg)
        T_wS = self.pose_info2_transform(shoulder_pose)
        if side == "Left" and self.init_R_wLS is None:
            self.init_R_wLS = T_wS[:3, :3]
        if side == "Right" and self.init_R_wRS is None:
            self.init_R_wRS = T_wS[:3, :3]
        self.compute_shoudler_pose(T_wS[:3, :3], side)

        T_ChestElbow = T_wElbow
        T_ChestHand = T_wHand

        hand_pos, _ = transform_to_pos_rpy(T_ChestHand)
        hand_pos[0] -= chest_pose.position.x
        hand_pos[1] -= chest_pose.position.y
        hand_pos[2] -= chest_pose.position.z
        hand_pos = axis_angle_to_matrix(self.chest_axis_agl).T @ hand_pos
        hand_pos[0] += bias_chest_to_base_link[0]
        hand_pos[1] += bias_chest_to_base_link[1]
        hand_pos[2] += bias_chest_to_base_link[2]
        hand_pos[0] = 0.1 if hand_pos[0] < 0.1 else hand_pos[0]
        
        elbow_pos, _ = transform_to_pos_rpy(T_ChestElbow)
        elbow_pos[0] -= chest_pose.position.x
        elbow_pos[1] -= chest_pose.position.y
        elbow_pos[2] -= chest_pose.position.z
        elbow_pos = axis_angle_to_matrix(self.chest_axis_agl).T @ elbow_pos
        elbow_pos[0] += bias_chest_to_base_link[0]
        elbow_pos[2] += bias_chest_to_base_link[2]

        shoulder_pos, _ = transform_to_pos_rpy(T_wS)
        shoulder_pos[0] -= chest_pose.position.x
        shoulder_pos[1] -= chest_pose.position.y
        shoulder_pos[2] -= chest_pose.position.z
        shoulder_pos = axis_angle_to_matrix(self.chest_axis_agl).T @ shoulder_pos
        shoulder_pos[0] += bias_chest_to_base_link[0]
        shoulder_pos[2] += bias_chest_to_base_link[2]
        

        human_shoulder_pos = list(shoulder_pos[:])
        if (side == "Right"):
            shoulder_pos[1] = -self.shoulder_width
        elif (side == "Left"):
            shoulder_pos[1] = self.shoulder_width
        # human_hand_pos = hand_pos[:].copy()

        # Scale arm positions
        elbow_pos, hand_pos = self.scale_arm_positions(shoulder_pos, elbow_pos, hand_pos, human_shoulder_pos, side)
        

        if self.vis_pub:
            marker = self.construct_point_marker(hand_pos, 0.08, 0.9, color=[1, 0, 0])
            elbow_marker = self.construct_point_marker(elbow_pos, 0.1, color=[0, 1, 0])
            shoulder_marker = self.construct_point_marker(shoulder_pos, 0.1, color=[0, 0, 1])
            # huaman_marker = self.construct_point_marker(human_hand_pos, 0.1, 0.8, color=[1, 1, 1])
            if side == "Left":
                self.marker_pub.publish(marker)
                self.marker_pub_elbow.publish(elbow_marker)
                self.marker_pub_shoulder.publish(shoulder_marker)
                # self.marker_pub_human.publish(huaman_marker)
            else:
                self.marker_pub_right.publish(marker)
                self.marker_pub_elbow_right.publish(elbow_marker)
                self.marker_pub_shoulder_right.publish(shoulder_marker)
            chest_pos = [chest_pose.position.x, chest_pose.position.y, chest_pose.position.z]
            chest_marker = self.construct_point_marker(chest_pos, 0.1, 0.8)
            self.marker_pub_chest.publish(chest_marker)
            
        if side == "Left":
            self.left_hand_pose = (hand_pos, hand_quat)
            self.left_elbow_pos = elbow_pos
        else:
            self.right_hand_pose = (hand_pos, hand_quat)
            self.right_elbow_pos = elbow_pos

    def check_if_vr_error(self):
        """
        Check if the VR system is error.
        """
        if self.left_hand_pose is None:
            return False
        left_hand_pos = self.left_hand_pose[0]
        right_hand_pos = self.right_hand_pose[0]
        error = True
        error &= (-0.1 < left_hand_pos[0] < 0.15)
        error &= (-0.05 < left_hand_pos[1] < 0.05)
        error &= (0.15 < left_hand_pos[2] < 0.4)
        error &= (-0.1 < right_hand_pos[0] < 0.15)
        error &= (-0.05 < right_hand_pos[1] < 0.05)
        error &= (0.15 < right_hand_pos[2] < 0.4)
        return error

    @staticmethod
    def pose_info2_transform(pose_info):
        """
        Convert a PoseInfo message to a 4x4 transformation matrix.
        """
        pos = [pose_info.position.x, pose_info.position.y, pose_info.position.z]
        quat = [pose_info.orientation.x, pose_info.orientation.y, pose_info.orientation.z, pose_info.orientation.w]
        rpy = quaternion_to_RPY(quat)
        T = pos_rpy_to_transform(pos, rpy)
        return T
    
    def construct_marker(self, arm_pose_p, arm_pose_q, r, g, b, side):
        if len(arm_pose_q) != 4 or len(arm_pose_p)!= 3:
            print("Invalid arm pose, cannot construct marker")
            return None
        marker = Marker()
        # marker.id = id
        marker.header.frame_id = (
            "base_link"  # set frame_id according to the actual situation
        )
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        if side == "Left":
            marker.mesh_resource = (
                "file://"
                + self.model_path +
                "/meshes/" + self.eef_visual_stl_files[0]
            )
        elif side == "Right":
            marker.mesh_resource = (
                "file://"
                + self.model_path
                + "/meshes/" + self.eef_visual_stl_files[1]
            )
        elif side == "Torso":
            marker.mesh_resource = (
                "file://"
                + self.model_path
                + "/meshes/base_link.STL"
            )
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.a = 0.3
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = arm_pose_p[0]
        marker.pose.position.y = arm_pose_p[1]
        marker.pose.position.z = arm_pose_p[2]
        marker.pose.orientation = Quaternion(*arm_pose_q)
        return marker

    def construct_point_marker(self, point, scale=0.05, alpha=0.3, color=[0, 0, 1]):
        if len(point) != 3:
            print("Invalid pos, cannot construct marker")
            return None
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.a = alpha
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        marker.pose.orientation.w = 1.0
        return marker 

    def pub_whole_body_joint_state_msg(self, arm_joint_angles, 
                                       left_finger_joints=None, right_finger_joints=None,
                                       torso_ypr=None, torso_height=None):
        """
        Build a JointState message for the whole body joints.
        """
        if len(arm_joint_angles) != 14:
            print("Invalid arm joint angles, cannot construct joint state message")
            return
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [
            # "leg_l1_joint",
            # "leg_l2_joint",
            # "leg_l3_joint",
            # "leg_l4_joint",
            # "leg_l5_joint",
            # "leg_l6_joint",
            # "leg_r1_joint",
            # "leg_r2_joint",
            # "leg_r3_joint",
            # "leg_r4_joint",
            # "leg_r5_joint",
            # "leg_r6_joint",
            "vj_z",
            "vj_yaw",
            "vj_pitch",
            "vj_roll",
            "zarm_l1_joint",
            "zarm_l2_joint",
            "zarm_l3_joint",
            "zarm_l4_joint",
            "zarm_l5_joint",
            "zarm_l6_joint",
            "zarm_l7_joint",
            "zarm_r1_joint",
            "zarm_r2_joint",
            "zarm_r3_joint",
            "zarm_r4_joint",
            "zarm_r5_joint",
            "zarm_r6_joint",
            "zarm_r7_joint",
            "l_thumb_proximal_yaw_joint",
            "l_thumb_distal_pitch_joint",
            "l_index_proximal_finger_joint",
            "l_index_distal_finger_joint",
            "l_middle_proximal_finger_joint",
            "l_middle_distal_finger_joint",
            "l_ring_proximal_finger_joint",
            "l_ring_distal_finger_joint",
            "l_pinky_proximal_finger_joint",
            "l_pinky_distal_finger_joint",
            "r_thumb_proximal_yaw_joint",
            "r_thumb_distal_pitch_joint",
            "r_index_proximal_finger_joint",
            "r_index_distal_finger_joint",
            "r_middle_proximal_finger_joint",
            "r_middle_distal_finger_joint",
            "r_ring_proximal_finger_joint",
            "r_ring_distal_finger_joint",
            "r_pinky_proximal_finger_joint",
            "r_pinky_distal_finger_joint"
            ]
        # print(f"Dim: {len(msg.name)}")
        for i in range(len(msg.name)):
            msg.position.append(0.0)
        msg.position[0] = torso_height
        if torso_ypr is not None:
            for i in range(3):
                msg.position[1+i] = torso_ypr[i]
        vj_n = 4
        for i in range(14):  # arm
            msg.position[vj_n + i] = arm_joint_angles[i]
        map = [0, 1, 2, 4, 6, 8]  # 6
        if left_finger_joints is not None and right_finger_joints is not None:
            for i in range(6):  # left hand finger
                msg.position[vj_n + 14 + map[i]] = left_finger_joints[i]
            for i in range(6):  # right hand finger
                msg.position[vj_n + 24 + map[i]] = right_finger_joints[i]
        # deal right hand thumb
        msg.position[vj_n + 24 + 1] *= -1
        self.joint_state_puber.publish(msg)
       
    def pub_head_body_pose_msg(self, head_body_pose: HeadBodyPose):
        msg = headBodyPose()
        msg.head_pitch = head_body_pose.head_pitch
        msg.head_yaw = head_body_pose.head_yaw
        msg.body_yaw = head_body_pose.body_yaw
        pitch_ratio = 0.8
        msg.body_pitch = max(3*np.pi/180.0, min(pitch_ratio*head_body_pose.body_pitch, 40*np.pi/180.0))

        msg.body_x = 0.0
        msg.body_y = 0.0
        msg.body_height = max(-0.4, min(head_body_pose.body_height + 0.3, 0.2))
        self.head_body_pose_puber.publish(msg)
