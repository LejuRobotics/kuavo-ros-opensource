"""
Pico VR device integration with ROS.

This module provides tools for:
- ROS node management
- Pose and gesture handling
- Visualization
- Service control
"""

import os
import sys
import signal
import json
import rospy
import rospkg
import tf
import numpy as np
from typing import List, Dict, Optional, Tuple, Any
from tf2_msgs.msg import TFMessage
from kuavo_msgs.msg import (
    twoArmHandPoseCmd, robotBodyMatrices, headBodyPose,
    picoPoseInfoList, robotHeadMotionData, ikSolveParam, 
    footPoseTargetTrajectories
)
from kuavo_msgs.srv import changeArmCtrlMode
from noitom_hi5_hand_udp_python.msg import JoySticks # TODO: 从noitom_hi5_hand_udp_python导入是临时的，等待该消息迁移到 kuavo_msgs
from .pico_utils import KuavoPicoInfoTransformer
from common.logger import SDKLogger


class KuavoPicoNodeManager:
    """Singleton class for managing ROS node initialization."""
    
    _instance = None
    _node_initialized = False

    @classmethod
    def get_instance(cls, node_name: str = 'kuavo_pico_node') -> 'KuavoPicoNodeManager':
        """Get singleton instance."""
        if not cls._instance:
            cls._instance = cls(node_name)
        return cls._instance

    def __init__(self, node_name: str):
        """Initialize the node manager."""
        if not KuavoPicoNodeManager._node_initialized:
            rospy.init_node(node_name)
            KuavoPicoNodeManager._node_initialized = True

    @staticmethod
    def get_ros_is_shutdown() -> bool:
        """Check if ROS is shutdown."""
        return rospy.is_shutdown()


class KuavoPicoBoneFramePublisher:
    """Class for publishing bone frame information."""
    
    def __init__(self, node_name: str = 'kuavo_pico_bone_frame_publisher'):
        """Initialize the bone frame publisher."""
        SDKLogger.info(f"[{node_name}] Initializing KuavoPicoBoneFramePublisher...")
        self.rate = rospy.Rate(100.0)
        
        # Initialize publishers
        self._init_publishers()
        
        # Create TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

    def _init_publishers(self) -> None:
        """Initialize ROS publishers."""
        self.pose_info_list_pub = rospy.Publisher(
            '/leju_pico_bone_poses', 
            picoPoseInfoList, 
            queue_size=10
        )
        self.head_motion_data_pub = rospy.Publisher(
            '/robot_head_motion_data', 
            robotHeadMotionData, 
            queue_size=10
        )
        self.foot_pose_pub = rospy.Publisher(
            '/humanoid_mpc_foot_pose_target_trajectories', 
            footPoseTargetTrajectories, 
            queue_size=10
        )

        # PICO 手柄数据
        self.pico_joy_pub = rospy.Publisher('/pico/joy', JoySticks, queue_size=10)

    def publish_pico_joys(self, joy: JoySticks) -> None:
        """Publish pico joys"""
        try:
            if not rospy.is_shutdown():
                if joy is not None:
                    self.pico_joy_pub.publish(joy)
            else:
                SDKLogger.error("ROS node is shutdown, cannot publish pico joys")
        except Exception as e:
            SDKLogger.error(f"Error publishing pico joys: {e}")

    def publish_pose_info_list(self, pose_info_list: picoPoseInfoList) -> None:
        """Publish pose information list."""
        try:
            if not rospy.is_shutdown():
                self.pose_info_list_pub.publish(pose_info_list)
            else:
                SDKLogger.error("ROS node is shutdown, cannot publish pose info list")
        except Exception as e:
            SDKLogger.error(f"Error publishing pose info list: {e}")

    def publish_foot_pose_trajectory(self, foot_pose_trajectory: footPoseTargetTrajectories) -> None:
        """Publish foot pose trajectory."""
        print("\\\\\\\\\\\\\\\\\\\\\\\\///////////////////////")
        SDKLogger.info(f"publish_foot_pose_trajectory time: {rospy.Time.now().to_sec()}")
        self.foot_pose_pub.publish(foot_pose_trajectory)

    def publish_hands_info_list(self, hands_info_list: picoPoseInfoList) -> None:
        """Publish hands information list."""
        try:
            if not rospy.is_shutdown():
                self.hands_info_list_pub.publish(hands_info_list)
            else:
                SDKLogger.error("ROS node is shutdown, cannot publish hands info list")
        except Exception as e:
            SDKLogger.error(f"Error publishing hands info list: {e}")

    def publish_head_body_pose_msg(self, head_body_pose_msg: headBodyPose) -> None:
        """Publish head body pose."""
        SDKLogger.info("Attempting to publish head body pose...")
        try:
            if not rospy.is_shutdown():
                self.head_body_pose_pub.publish(head_body_pose_msg)
            else:
                SDKLogger.error("ROS node is shutdown, cannot publish head body pose")
        except Exception as e:
            SDKLogger.error(f"Error publishing head motion data: {e}")

    def publish_tf(self, translation: List[float], rotation: List[float], 
                  child: str, parent: str) -> None:
        """Publish transform."""
        self.tf_broadcaster.sendTransform(
            translation, 
            rotation, 
            rospy.Time.now(), 
            child, 
            parent
        )

    def publish_hand_finger_tf(self, tf_msg: TFMessage) -> None:
        """Publish hand finger transform."""
        self.hand_finger_tf_pub.publish(tf_msg)

    def get_listener(self) -> tf.TransformListener:
        """Get transform listener."""
        return self.tf_listener

    def sleep(self) -> None:
        """Sleep for the configured rate."""
        self.rate.sleep()


class KuavoPicoNode:
    """Main Pico node class for handling VR device integration."""
    
    _instance = None
    
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self):
        """Initialize the Pico node."""
        if not hasattr(self, '_initialized'):
            KuavoPicoNodeManager.get_instance('kuavo_pico_node')
            
            # Initialize parameters
            self._init_parameters()
            
            # Initialize model and transformers
            self._init_model_and_transformers()
            
            # Initialize publishers
            self._init_publishers()
            
            # Initialize subscribers
            self._init_subscribers()

            self.robot_matrix_publisher = RobotMatrixPublisher()
            
            self._initialized = True

    def _init_parameters(self) -> None:
        """Initialize node parameters."""
        self.use_custom_ik_param = True
        self.ik_solve_param = ikSolveParam()
        self.set_ik_solver_params()
        
        self.send_srv = True
        self.last_pico_running_state = False
        self.button_y_last = False

    def _init_model_and_transformers(self) -> None:
        """Initialize model and transformers."""
        model_path = self.set_robot_model_params()
        self.bone_frame_publisher = KuavoPicoBoneFramePublisher()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.pico_info_transformer = KuavoPicoInfoTransformer(
            model_path, 
            tf_broadcaster=self.tf_broadcaster,
            bone_frame_publisher=self.bone_frame_publisher
        )

    def _init_publishers(self) -> None:
        """Initialize ROS publishers."""
        self.pub = rospy.Publisher(
            '/mm/two_arm_hand_pose_cmd', 
            twoArmHandPoseCmd, 
            queue_size=10
        )
        self.pub_ik = rospy.Publisher(
            '/ik/two_arm_hand_pose_cmd', 
            twoArmHandPoseCmd, 
            queue_size=10
        )

    def _init_subscribers(self) -> None:
        """Initialize ROS subscribers."""
        rospy.Subscriber(
            "/leju_pico_bone_poses", 
            picoPoseInfoList, 
            self.pico_hands_poses_callback
            # self.pico_hands_poses_callback_ik
        )
        rospy.Subscriber(
            "/robot_body_matrices", 
            robotBodyMatrices, 
            self.robot_body_matrices_callback
        )

    def set_robot_model_params(self) -> str:
        """Set robot model parameters."""
        SDKLogger.info("Setting robot model parameters...")
        kuavo_assests_path = self._get_package_path("kuavo_assets")
        robot_version = os.environ.get('ROBOT_VERSION', '45')
        model_config_file = kuavo_assests_path + f"/config/kuavo_v{robot_version}/kuavo.json"
        
        with open(model_config_file, 'r') as f:
            model_config = json.load(f)
            
        rospy.set_param("/pico/upper_arm_length", model_config["upper_arm_length"])
        rospy.set_param("/pico/lower_arm_length", model_config["lower_arm_length"])
        rospy.set_param("/pico/shoulder_width", model_config["shoulder_width"])
        
        return kuavo_assests_path + f"/models/biped_s{robot_version}"

    def _get_package_path(self, package_name: str) -> Optional[str]:
        """Get the path of a ROS package."""
        try:
            rospack = rospkg.RosPack()
            return rospack.get_path(package_name)
        except rospkg.ResourceNotFound:
            SDKLogger.error(f"Package {package_name} not found")
            return None

    def set_ik_solver_params(self) -> None:
        """Set IK solver parameters."""
        SDKLogger.info("Setting IK solver parameters...")
        self.ik_solve_param.major_optimality_tol = 9e-3
        self.ik_solve_param.major_feasibility_tol = 9e-3
        self.ik_solve_param.minor_feasibility_tol = 9e-3
        self.ik_solve_param.major_iterations_limit = 50
        self.ik_solve_param.oritation_constraint_tol = 9e-3
        self.ik_solve_param.pos_constraint_tol = 9e-3
        self.ik_solve_param.pos_cost_weight = 10.0
        SDKLogger.info("IK solver parameters set successfully")

    def change_arm_ctrl_mode(self, mode: int) -> None:
        """Change arm control mode."""
        try:
            rospy.wait_for_service("/change_arm_ctrl_mode")
            changeHandTrackingMode_srv = rospy.ServiceProxy("/change_arm_ctrl_mode", changeArmCtrlMode)
            changeHandTrackingMode_srv(mode)
        except Exception as e:
            SDKLogger.error(f"Service /change_arm_ctrl_mode call failed: {e}")

    def change_mobile_ctrl_mode(self, mode: int) -> None:
        """Change mobile control mode."""
        try:
            rospy.wait_for_service("/mobile_manipulator_mpc_control")
            changeHandTrackingMode_srv = rospy.ServiceProxy("/mobile_manipulator_mpc_control", changeArmCtrlMode)
            changeHandTrackingMode_srv(mode)
        except Exception as e:
            SDKLogger.error(f"Service /mobile_manipulator_mpc_control call failed: {e}")

    def change_mm_wbc_arm_ctrl_mode(self, mode: int) -> None:
        """Change MM WBC arm control mode."""
        try:
            rospy.wait_for_service("/enable_mm_wbc_arm_trajectory_control")
            changeHandTrackingMode_srv = rospy.ServiceProxy("/enable_mm_wbc_arm_trajectory_control", changeArmCtrlMode)
            changeHandTrackingMode_srv(mode)
        except Exception as e:
            SDKLogger.error(f"Service /enable_mm_wbc_arm_trajectory_control call failed: {e}")

    def robot_body_matrices_callback(self, robot_body_matrices_msg: robotBodyMatrices) -> None:
        """Callback for robot body matrices."""
        pass
        robot_urdf_matrices, current_time = self.robot_matrix_publisher.get_all_matrices_from_message(robot_body_matrices_msg)
        self.pico_info_transformer.publish_local_poses(robot_urdf_matrices, current_time)
    
        self.pico_info_transformer.process_foot_poses_parallel(robot_urdf_matrices)
        # self.pico_info_transformer.test_foot_controller()

    def pico_hands_poses_callback(self, pico_hands_poses_msg: picoPoseInfoList) -> None:
        """Callback for Pico hands poses."""
        self.pico_info_transformer.read_msg_hand(pico_hands_poses_msg)

        left_pose, left_elbow_pos = self.pico_info_transformer.get_hand_pose("Left")
        right_pose, right_elbow_pos = self.pico_info_transformer.get_hand_pose("Right")
        
        eef_pose_msg = twoArmHandPoseCmd()
        eef_pose_msg.hand_poses.left_pose.pos_xyz = left_pose[0]
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = left_pose[1]
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = left_elbow_pos

        eef_pose_msg.hand_poses.right_pose.pos_xyz = right_pose[0]
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = right_pose[1]
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = right_elbow_pos
        eef_pose_msg.ik_param = self.ik_solve_param
        # eef_pose_msg.use_custom_ik_param = self.use_custom_ik_param
        eef_pose_msg.use_custom_ik_param = False
        eef_pose_msg.frame = 3
        
        # Check for VR errors
        if self.pico_info_transformer.check_if_vr_error():
            SDKLogger.error("Detected VR ERROR!!! Please check the Pico device connection and status!")
            return
            
        if self.pico_info_transformer.is_running:
            self.pub.publish(eef_pose_msg)
            if self.pico_info_transformer.control_torso_mode:
                self.pico_info_transformer.pub_head_body_pose_msg(self.pico_info_transformer.head_body_pose)
        else:
            SDKLogger.warn("Not publishing to /mm/two_arm_hand_pose_cmd because is_running is False")
        
        # Handle service mode changes
        if self.send_srv and (self.last_pico_running_state != self.pico_info_transformer.is_running):
            SDKLogger.info(f"Pico running state change to: {self.pico_info_transformer.is_running}")
            mode = 2 if self.pico_info_transformer.is_running else 0
            mobile_mode = 1 if self.pico_info_transformer.is_running else 0
            wbc_mode = 1 if self.pico_info_transformer.is_running else 0
            self.change_arm_ctrl_mode(mode)
            self.change_mobile_ctrl_mode(mobile_mode)
            self.change_mm_wbc_arm_ctrl_mode(wbc_mode)
            self.last_pico_running_state = self.pico_info_transformer.is_running

    def pico_hands_poses_callback_ik(self, pico_hands_poses_msg: picoPoseInfoList) -> None:
        """Callback for Pico hands poses."""
        self.pico_info_transformer.read_msg_hand(pico_hands_poses_msg)

        left_pose, left_elbow_pos = self.pico_info_transformer.get_hand_pose("Left")
        right_pose, right_elbow_pos = self.pico_info_transformer.get_hand_pose("Right")
        
        eef_pose_msg = twoArmHandPoseCmd()
        eef_pose_msg.hand_poses.left_pose.pos_xyz = left_pose[0]
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = left_pose[1]
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = left_elbow_pos

        eef_pose_msg.hand_poses.right_pose.pos_xyz = right_pose[0]
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = right_pose[1]
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = right_elbow_pos
        eef_pose_msg.ik_param = self.ik_solve_param
        eef_pose_msg.use_custom_ik_param = self.use_custom_ik_param
        
        # Check for VR errors
        if self.pico_info_transformer.check_if_vr_error():
            SDKLogger.error("Detected VR ERROR!!! Please check the Pico device connection and status!")
            return
            
        if self.pico_info_transformer.is_running:
            self.pub_ik.publish(eef_pose_msg)
        else:
            SDKLogger.warn("Not publishing to /ik/two_arm_hand_pose_cmd because is_running is False")

class RobotMatrixPublisher:
    """Publisher for robot body matrices data recording."""
    
    def __init__(self, topic_name="/robot_body_matrices", queue_size=100):
        """
        Initialize the publisher.
        
        Args:
            topic_name (str): ROS topic name for publishing
            queue_size (int): Queue size for the publisher
        """
        self.publisher = rospy.Publisher(
            topic_name, 
            robotBodyMatrices, 
            queue_size=queue_size
        )
        
        # 身体部位名称列表 - 与BODY_TRACKER_ROLES对应
        self.body_parts = [
            "Pelvis", "LEFT_HIP", "RIGHT_HIP", "SPINE1", "LEFT_KNEE", 
            "RIGHT_KNEE", "SPINE2", "LEFT_ANKLE", "RIGHT_ANKLE", "SPINE3",
            "LEFT_FOOT", "RIGHT_FOOT", "NECK", "LEFT_COLLAR", "RIGHT_COLLAR",
            "HEAD", "LEFT_SHOULDER", "RIGHT_SHOULDER", "LEFT_ELBOW", 
            "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST", "LEFT_HAND", "RIGHT_HAND"
        ]
        
        self.data_source = "pico_vr_teleoperation"
        SDKLogger.info(f"RobotMatrixPublisher initialized on topic: {topic_name}")
    
    def publish_matrices(self, robot_urdf_matrices, current_time):
        """
        Publish robot body matrices data.
        
        Args:
            robot_urdf_matrices (np.ndarray): Shape [N, 4, 4] transformation matrices
            current_time (rospy.Time): Current ROS time
        """
        try:
            # 创建消息
            msg = robotBodyMatrices()
            
            # 设置header
            msg.header.stamp = current_time
            msg.header.frame_id = "world"
            
            # 设置时间戳
            msg.timestamp = current_time
            
            # 展平矩阵数据
            matrices_flat = []
            for matrix in robot_urdf_matrices:
                matrices_flat.extend(matrix.flatten())
            msg.matrices_data = matrices_flat
            
            # 设置矩阵数量
            msg.num_matrices = len(robot_urdf_matrices)
            
            # 设置身体部位名称
            msg.body_parts = self.body_parts[:msg.num_matrices]
            
            # 设置数据来源
            msg.data_source = self.data_source
            
            # 发布消息
            self.publisher.publish(msg)
            
        except Exception as e:
            SDKLogger.error(f"Error publishing robot matrices: {e}")
    
    def get_matrix_from_message(self, msg, matrix_index):
        """
        Extract a specific matrix from the message.
        
        Args:
            msg (RobotBodyMatrices): The message
            matrix_index (int): Index of the matrix to extract
            
        Returns:
            np.ndarray: 4x4 transformation matrix
        """
        if matrix_index >= msg.num_matrices:
            raise ValueError(f"Matrix index {matrix_index} out of range")
        
        start_idx = matrix_index * 16
        end_idx = start_idx + 16
        matrix_data = msg.matrices_data[start_idx:end_idx]
        timestamp = msg.timestamp
        
        return np.array(matrix_data).reshape(4, 4), timestamp
    
    def get_all_matrices_from_message(self, msg):
        """
        Extract all matrices from the message.
        
        Args:
            msg (RobotBodyMatrices): The message
            
        Returns:
            np.ndarray: Shape [N, 4, 4] transformation matrices
        """
        matrices = []
        for i in range(msg.num_matrices):
            matrix, timestamp = self.get_matrix_from_message(msg, i)
            matrices.append(matrix)
        
        return np.array(matrices), timestamp

def signal_handler(sig: int, frame: Any) -> None:
    """Handle signal for graceful exit."""
    print('Exiting gracefully...')
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    try:
        # Initialize the publisher
        publisher = KuavoPicoNode()
        
        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    