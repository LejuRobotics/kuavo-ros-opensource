#!/opt/miniconda3/envs/joint_cali/bin/python
# -*- coding: utf-8 -*-

# from pinocchio import casadi as cpin
import pinocchio as pin
# import casadi as cs
from pinocchio.robot_wrapper import RobotWrapper

import os
import numpy as np

import rospkg
import numpy as np

def get_package_path(package_name):
    try:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path(package_name)
        return package_path
    except rospkg.ResourceNotFound:
        return None

def quat_to_rot(q):
    w, x, y, z = q
    R = np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                    [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                    [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]], dtype=np.float64)
    return R

def rot_to_quat(R):
    w = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
    x = (R[2, 1] - R[1, 2]) / (4 * w)
    y = (R[0, 2] - R[2, 0]) / (4 * w)
    z = (R[1, 0] - R[0, 1]) / (4 * w)
    return np.array([w, x, y, z])


class HeadKinematics:
    def __init__(self, urdf_path):
        
        origin_robot = pin.RobotWrapper.BuildFromURDF(urdf_path)
        mixed_jointsToLockIDs = [
            "leg_l1_joint",
            "leg_l2_joint",
            "leg_l3_joint",
            "leg_l4_joint",
            "leg_l5_joint",
            "leg_l6_joint",
            "leg_r1_joint",
            "leg_r2_joint",
            "leg_r3_joint",
            "leg_r4_joint",
            "leg_r5_joint",
            "leg_r6_joint",
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
        ]
        
        self.robot = origin_robot.buildReducedRobot(
            list_of_joints_to_lock=mixed_jointsToLockIDs,
            reference_configuration=np.array([0.0] * origin_robot.model.nq),
        )
        print("nq_reduced: ", self.robot.model.nq)
        print("nv_reduced: ", self.robot.model.nv)

    def FK(self, q):
        # image frame in camera_base frame
        # TODO: 需要根据实际的相机安装位置进行修改
        p_ci = np.array([0.0, 0.018, 0.013])
        R_ci = quat_to_rot([0.5, -0.5, 0.5, -0.5])
        # 使用 Pinocchio 的原生方法进行正向运动学计算
        self.data = self.robot.model.createData()
        pin.forwardKinematics(self.robot.model, self.data, q)
        pin.updateFramePlacements(self.robot.model, self.data)

        camera_id = self.robot.model.getFrameId("camera_base", pin.FrameType.BODY)
        camera_pose = self.data.oMf[camera_id]
        p_bc, R_bc = camera_pose.translation, camera_pose.rotation
        p_bi = p_bc + R_bc @ p_ci
        R_bi = R_bc @ R_ci
        return p_bi, R_bi

class ArmKinematics:
    def __init__(self, urdf_path, T_et):
        self.robot = {}
        self.T_et = T_et
        # 加载机器人模型
        self.origin_robot = RobotWrapper.BuildFromURDF(urdf_path)
        self.mixed_jointsToLockIDs = [
            "leg_l1_joint",
            "leg_l2_joint",
            "leg_l3_joint",
            "leg_l4_joint",
            "leg_l5_joint",
            "leg_l6_joint",
            "leg_r1_joint",
            "leg_r2_joint",
            "leg_r3_joint",
            "leg_r4_joint",
            "leg_r5_joint",
            "leg_r6_joint",
            "zhead_1_joint",
            "zhead_2_joint",
        ]
        self.kinematics_l = self.build_arm_fk("l")
        self.kinematics_r = self.build_arm_fk("r")
        self.eef_fk_l = self.build_eef_fk("l")
        self.eef_fk_r = self.build_eef_fk("r")

    def get_T_teef(self, side):
        q0 = np.zeros(self.robot[side].model.nq)
        if side == "l":
            p_bt, R_bt, _ =  self.FK_l(q0)
            p_be, R_be = self.eef_FK_l(q0)
            p_te_b = p_be - p_bt
            p_te = R_bt.T @ p_te_b
            R_te = R_bt.T @ R_be
        else:
            p_bt, R_bt, _ =  self.FK_r(q0)
            p_be, R_be = self.eef_FK_r(q0)
            p_te_b = p_be - p_bt
            p_te = R_bt.T @ p_te_b
            R_te = R_bt.T @ R_be
        return p_te, R_te
        
    def build_arm_fk(self, side):
        if side == "l":
            mixed_jointsToLockIDs_tmp = self.mixed_jointsToLockIDs + \
            ["zarm_r1_joint",
             "zarm_r2_joint",
             "zarm_r3_joint",
             "zarm_r4_joint",
             "zarm_r5_joint",
             "zarm_r6_joint",
             "zarm_r7_joint",]
        else:
            mixed_jointsToLockIDs_tmp = self.mixed_jointsToLockIDs + \
            ["zarm_l1_joint",
             "zarm_l2_joint",
             "zarm_l3_joint",
             "zarm_l4_joint",
             "zarm_l5_joint",
             "zarm_l6_joint",
             "zarm_l7_joint",]
        
        robot = self.origin_robot.buildReducedRobot(
            list_of_joints_to_lock=mixed_jointsToLockIDs_tmp,
            reference_configuration=np.zeros(self.origin_robot.model.nq),
        )
        self.robot[side] = robot
        self.data = robot.model.createData()

        if side == "l":
            hand_id = robot.model.getFrameId("zarm_l7_link")
        else:
            hand_id = robot.model.getFrameId("zarm_r7_link")

        def kinematics(q):
            pin.forwardKinematics(robot.model, self.data, q)
            pin.updateFramePlacements(robot.model, self.data)
            hand_pose = self.data.oMf[hand_id]
            p_be = hand_pose.translation
            R_be = hand_pose.rotation
            pos = p_be + R_be @ self.T_et[:3, 3]
            rot = R_be @ self.T_et[:3, :3]
            J = pin.computeFrameJacobian(robot.model, self.data, q, hand_id)
            return pos, rot, J

        return kinematics

    def FK_l(self, q):
        pos, rot, J = self.kinematics_l(q)
        return pos, rot, J
    
    def FK_r(self, q):
        pos, rot, J = self.kinematics_r(q)
        return pos, rot, J

    def eef_FK_l(self, q):
        pos, rot = self.eef_fk_l(q)
        return pos, rot
    
    def eef_FK_r(self, q):
        pos, rot = self.eef_fk_r(q)
        return pos, rot

    def build_eef_fk(self, side='l'):
        robot = self.robot[side]
        self.data = robot.model.createData()

        if side == "l":
            eef_id = robot.model.getFrameId("zarm_l7_end_effector")
        else:
            eef_id = robot.model.getFrameId("zarm_r7_end_effector")

        def kinematics(q):
            pin.forwardKinematics(robot.model, self.data, q)
            pin.updateFramePlacements(robot.model, self.data)
            eef_pose = self.data.oMf[eef_id]
            p_be = eef_pose.translation
            R_be = eef_pose.rotation
            return p_be, R_be

        return kinematics
        
    def gravityBiasTorque(self, q, side='l'):
        robot = self.robot[side]
        data = robot.model.createData()
        pin.computeGeneralizedGravity(robot.model, data, q)
        return data.g.copy()


if __name__ == "__main__":    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--publish_tf", action="store_true")
    args = parser.parse_args()
    
    n_poses = 3
    n_joints = 14
    q_list = [np.random.uniform(-np.pi, np.pi, n_joints) for _ in range(n_poses)]
    print("q_list: ", q_list)
    
    asset_path = get_package_path("kuavo_assets")
    print(f"asset_path: {asset_path}")
    robot_version = os.environ.get('ROBOT_VERSION', '40')
    urdf_path = os.path.join(asset_path, f"models/biped_s{robot_version}/urdf/biped_s{robot_version}.urdf")
    print(f"urdf_path: {urdf_path}")
    T_et = np.eye(4)
    quat_lc = [0.5, -0.5, 0.5, -0.5]
    # quat_lc = [-0.5, 0.5, -0.5, 0.5]
    T_et[:3, :3] = quat_to_rot(quat_lc)
    T_et[:3, 3] = np.array([0.0, 0.0, 0.0])
    arm_kinematics = ArmKinematics(urdf_path, T_et)
    for q in q_list:
        pos_l, rot_l, J_l = arm_kinematics.FK_l(q[:7])
        pos_r, rot_r, J_r = arm_kinematics.FK_r(q[7:])
        print(f"pos_l: {pos_l}, pos_r: {pos_r}")
    true_bias = np.array([0.02, 0.03, 0.05])
    # test 0.56597157 kinematics
    head_kinematics = HeadKinematics(urdf_path)
    q_head = np.zeros(2)
    pos_i, rot_i = head_kinematics.FK(q_head)
    print(f"pos_i: {pos_i},\nrot_i: {rot_i}")
    
    # 计算重力补偿力矩
    q_arm_l = np.array([-np.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    q_arm_r = np.array([0.0, 0.0, 0.0, -np.pi/2, 0.0, 0.0, 0.0])
    bias_l = arm_kinematics.gravityBiasTorque(q_arm_l, 'l')
    bias_r = arm_kinematics.gravityBiasTorque(q_arm_r, 'r')
    print(f"bias_l: {bias_l}\nbias_r: {bias_r}")
    
    # 
    p_te_l, R_te_l = arm_kinematics.get_T_teef('l')
    p_te_r, R_te_r = arm_kinematics.get_T_teef('r')
    print(f"p_te_l: {p_te_l}, R_te_l: {R_te_l}")
    print(f"p_te_r: {p_te_r}, R_te_r: {R_te_r}")
    
    if not args.publish_tf:
        print("If you want to publish tf, please add '--publish_tf'")
        exit()
    # ros
    import rospy
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import JointState
    import os
    from arm_kinematics import get_package_path, HeadKinematics
    from kuavo_msgs.msg import sensorsData
    import tf2_ros
    import tf.transformations as tf_trans

    tf_broadcaster = tf2_ros.TransformBroadcaster()
    def publish_tf(p, quat, parent_frame_id, child_frame_id):
        # 发布TF转换
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = parent_frame_id
        
        transform.child_frame_id = child_frame_id
        
        # 设置位置
        transform.transform.translation.x = p[0]
        transform.transform.translation.y = p[1]
        transform.transform.translation.z = p[2]
        
        # 设置旋转
        transform.transform.rotation.w = quat[0]
        transform.transform.rotation.x = quat[1]
        transform.transform.rotation.y = quat[2]
        transform.transform.rotation.z = quat[3]
        
        # 广播TF
        tf_broadcaster.sendTransform(transform)

    q_head = None
    q_arm_l = None

    def sensor_data_callback(msg):
        global q_head, q_arm_l
        q_head = np.array(msg.joint_data.joint_q).flatten()[-2:]
        q_arm_l = np.array(msg.joint_data.joint_q).flatten()[12:12+7]

    # quat_lc = [1,0,0,0]
    try:
        rospy.init_node('arm_kinematics_node', anonymous=True)
        rospy.Subscriber("/share_memory/sensor_data_raw", sensorsData, sensor_data_callback)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if q_head is not None:
                pos_i, rot_i = head_kinematics.FK(q_head)
                pos_l, rot_l, J_l = arm_kinematics.FK_l(q_arm_l)
                quat = rot_to_quat(rot_i)
                publish_tf(pos_i, quat, "base_link", "img_link")
                # publish_tf(np.zeros(3), quat_lc, "zarm_l7_link", "cube_true")
                # publish_tf(pos_l, rot_to_quat(rot_l), "base_link", "cube_l_fk")
                # print(f"pos_i: {pos_i},\nrot_i: {quat}")
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
