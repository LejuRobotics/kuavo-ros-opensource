import rospy
import json
import xml.etree.ElementTree as ET
from kuavo_humanoid_sdk.common.logger import SDKLogger
# End effector types
class EndEffectorType:
    QIANGNAO = "qiangnao"
    QIANGNAO_TOUCH = "qiangnao_touch"
    LEJUCLAW = "lejuclaw"
class RosParameter:
    def __init__(self):
        pass
    def robot_version(self)->str:
        if not rospy.has_param('/robot_version'):
            rospy.logerr("robot_version parameter not found")
            return None
        return rospy.get_param('/robot_version')
    
    def arm_dof(self)->int:
        if not rospy.has_param('/armRealDof'):
            rospy.logerr("armRealDof parameter not found")
            return None
        return rospy.get_param('/armRealDof')
    
    def head_dof(self)->int:
        if not rospy.has_param('/headRealDof'):
            rospy.logerr("headRealDof parameter not found")
            return None
        return rospy.get_param('/headRealDof')
    
    def waist_dof(self)->int:
        if not rospy.has_param('/waistRealDof'):
            rospy.logerr("waistRealDof parameter not found")
            return None
        return rospy.get_param('/waistRealDof')

    def leg_dof(self)->int:
        if not rospy.has_param('/legRealDof'):
            rospy.logerr("legRealDof parameter not found")
            return None
        return rospy.get_param('/legRealDof')
    
    def end_effector_type(self)->str:
        if not rospy.has_param('/end_effector_type'):
            return None
        return rospy.get_param('/end_effector_type')
    
    def humanoid_description(self)->str:
        if not rospy.has_param('/humanoid_description'):
            rospy.logerr("humanoid_description parameter not found")
            return None
        return rospy.get_param('/humanoid_description')

    def model_path(self)->str:
        if not rospy.has_param('/modelPath'):
            rospy.logerr("modelPath parameter not found")
            return None
        return rospy.get_param('/modelPath')

    def kuavo_config(self)->str:
        if not rospy.has_param('/kuavo_configuration'):
            rospy.logerr("kuavo_configuration parameter not found")
            return None
        return rospy.get_param('/kuavo_configuration')

    def initial_state(self)->str:
        if not rospy.has_param('/initial_state'):
            rospy.logerr("initial_state parameter not found")
            return None
        return rospy.get_param('/initial_state')
    def init_stand_height(self)->float:
        if not rospy.has_param('/com_height'):
            rospy.logerr("com_height parameter not found")
            # KUAVO-4PRO
            return 0.8328437523948975
        return rospy.get_param('/com_height')

    def robot_type(self)->int:
        """Get robot type: 2/rl/ocs2=legged, 1/wheel-arm=wheel-arm"""
        return rospy.get_param('/robot_type', 2)

    def is_legged_robot(self)->bool:
        """Check if robot is legged (bipedal) type"""
        rt = self.robot_type()
        # 支持数字和字符串类型
        if isinstance(rt, str):
            return rt.lower() in ['2', 'rl', 'ocs2']
        return rt == 2

    def is_wheel_arm_robot(self)->bool:
        """Check if robot is wheel-arm type"""
        rt = self.robot_type()
        # 支持数字和字符串类型
        if isinstance(rt, str):
            return rt.lower() in ['1']
        return rt == 1

kuavo_ros_param = RosParameter()

def joint_names()->dict:

    robot_version_major = (int(kuavo_ros_param.robot_version()) // 10) % 10 

    if robot_version_major  == 1:
        leg_link_names = [
            'leg_l1_link', 'leg_l2_link', 'leg_l3_link', 'leg_l4_link', 'leg_l5_link', 'leg_l6_link',
            'leg_r1_link', 'leg_r2_link', 'leg_r3_link', 'leg_r4_link', 'leg_r5_link', 'leg_r6_link'
        ]
        waist_link_names = [
            'waist_yaw_link'
        ]
        arm_link_names = [
            'zarm_l1_link', 'zarm_l2_link', 'zarm_l3_link', 'zarm_l4_link',
            'zarm_r1_link', 'zarm_r2_link', 'zarm_r3_link', 'zarm_r4_link',
        ]
        head_link_names = [
            'zhead_1_link', 'zhead_2_link'
        ]
    elif robot_version_major == 4 :
        leg_link_names = [
            'leg_l1_link', 'leg_l2_link', 'leg_l3_link', 'leg_l4_link', 'leg_l5_link', 'leg_l6_link',
            'leg_r1_link', 'leg_r2_link', 'leg_r3_link', 'leg_r4_link', 'leg_r5_link', 'leg_r6_link'
        ]
        waist_link_names = []
        arm_link_names = [
            'zarm_l1_link', 'zarm_l2_link', 'zarm_l3_link', 'zarm_l4_link', 'zarm_l5_link', 'zarm_l6_link', 'zarm_l7_link',
            'zarm_r1_link', 'zarm_r2_link', 'zarm_r3_link', 'zarm_r4_link', 'zarm_r5_link', 'zarm_r6_link', 'zarm_r7_link',
        ]
        head_link_names = [
            'zhead_1_link', 'zhead_2_link'
        ]
    elif robot_version_major == 5:
        leg_link_names = [
            'leg_l1_link', 'leg_l2_link', 'leg_l3_link', 'leg_l4_link', 'leg_l5_link', 'leg_l6_link',
            'leg_r1_link', 'leg_r2_link', 'leg_r3_link', 'leg_r4_link', 'leg_r5_link', 'leg_r6_link'
        ]
        waist_link_names = [
            'waist_yaw_link'
        ]
        arm_link_names = [
            'zarm_l1_link', 'zarm_l2_link', 'zarm_l3_link', 'zarm_l4_link', 'zarm_l5_link', 'zarm_l6_link', 'zarm_l7_link',
            'zarm_r1_link', 'zarm_r2_link', 'zarm_r3_link', 'zarm_r4_link', 'zarm_r5_link', 'zarm_r6_link', 'zarm_r7_link',
        ]
        head_link_names = [
            'zhead_1_link', 'zhead_2_link'
        ]
    else:
        leg_link_names = [
            'knee_link', 'leg_link', 'waist_link', 'waist_yaw_link'
        ]
        waist_link_names = []
        arm_link_names = [
            'zarm_l1_link', 'zarm_l2_link', 'zarm_l3_link', 'zarm_l4_link', 'zarm_l5_link', 'zarm_l6_link', 'zarm_l7_link',
            'zarm_r1_link', 'zarm_r2_link', 'zarm_r3_link', 'zarm_r4_link', 'zarm_r5_link', 'zarm_r6_link', 'zarm_r7_link',
        ]
        head_link_names = [
            'zhead_1_link', 'zhead_2_link'
        ]
    robot_desc = kuavo_ros_param.humanoid_description()
    if robot_desc is None:
        return None

    """
        <link name="leg_l1_link">
        <inertial>
        ....
        </inertial>
        <visual>
        ...
        <geometry>
            <mesh filename="package://kuavo_assets/models/biped_s43/meshes/l_leg_roll.STL" />
        </geometry>
        ...
        </visual>
        </link>
    """
    root = ET.fromstring(robot_desc)
    
    def process_link_name(link_name):
        """从URDF中提取与link对应的joint名称"""
        # 查找child link等于目标link_name的joint
        # 遍历所有joint，找到child link匹配的joint
        for joint_elem in root.findall(".//joint"):
            child_elem = joint_elem.find("child")
            if child_elem is not None and child_elem.get("link") == link_name:
                joint_name = joint_elem.get("name")
                if joint_name:
                    return joint_name
        
        # 如果找不到joint，对于4-dof手臂的arm links不警告（可能不存在）
        if robot_version_major == 1 and link_name.startswith('zarm_'):
            return None
        
        # 其他情况记录警告并返回None
        SDKLogger.warn(f"Warning: Joint for link {link_name} not found in URDF")
        return None
    
    # Process waist joint names - extract directly from joint elements
    def process_waist_link_name(link_name):
        # Find the joint that has this link as child
        for joint_elem in root.findall(".//joint"):
            child_elem = joint_elem.find("child")
            if child_elem is not None and child_elem.get("link") == link_name:
                return joint_elem.get("name")
        return None
    
    leg_joint_names = [process_link_name(link_name) for link_name in leg_link_names if process_link_name(link_name) is not None]
    waist_joint_names = [process_waist_link_name(link_name) for link_name in waist_link_names if process_waist_link_name(link_name) is not None]
    arm_joint_names = [process_link_name(link_name) for link_name in arm_link_names if process_link_name(link_name) is not None]
    head_joint_names = [process_link_name(link_name) for link_name in head_link_names if process_link_name(link_name) is not None]

    # For robots with 4-dof arms, we allow arm_link_names to be fewer than expected
    if robot_version_major == 1:
        if len(leg_link_names) != len(leg_joint_names):
            SDKLogger.warn(
                f"leg_joint_names is not equal to leg_link_names, {len(leg_link_names)} != {len(leg_joint_names)}")
            return None
        # Don't check arm_link_names length for robots with 4-dof arms
        if len(head_link_names) != len(head_joint_names):
            SDKLogger.warn(
                f"head_joint_names is not equal to head_link_names, {len(head_link_names)}!= {len(head_joint_names)}")
            return None
        if len(waist_link_names) != len(waist_joint_names):
            SDKLogger.warn(
                f"waist_joint_names is not equal to waist_link_names, {len(waist_link_names)} != {len(waist_joint_names)}")
            # Don't return None, just use empty list if waist joint not found
            waist_joint_names = []
    else:
        if len(leg_link_names) != len(leg_joint_names):
            SDKLogger.warn(
                f"leg_joint_names is not equal to leg_link_names, {len(leg_link_names)} != {len(leg_joint_names)}")
            return None
        if len(arm_link_names) != len(arm_joint_names):
            SDKLogger.warn(
                f"arm_joint_names is not equal to arm_link_names, {len(arm_link_names)}!= {len(arm_joint_names)}")
            return None
        if len(head_link_names) != len(head_joint_names):
            SDKLogger.warn(
                f"head_joint_names is not equal to head_link_names, {len(head_link_names)}!= {len(head_joint_names)}")
            return None
        if len(waist_link_names) != len(waist_joint_names):
            SDKLogger.warn(
                f"waist_joint_names is not equal to waist_link_names, {len(waist_link_names)} != {len(waist_joint_names)}")
            # Don't return None, just use empty list if waist joint not found
            waist_joint_names = []
    
    return leg_joint_names + waist_joint_names + arm_joint_names + head_joint_names

kuavo_ros_info = None

def end_frames_names()->dict:
    kuavo_ros_param = RosParameter()

    # Determine default end effector frame names based on robot version and arm DOF
    robot_version = kuavo_ros_param.robot_version()
    if robot_version is not None:
        robot_version_major = (int(robot_version) // 10) % 10
        arm_dof = kuavo_ros_param.arm_dof()
        if arm_dof is not None:
            # For 4-DOF arms (8 total arm joints), use zarm_l4_link and zarm_r4_link
            if robot_version_major == 1 or arm_dof == 8:
                default = ["torso", "zarm_l4_link", "zarm_r4_link", "zarm_l4_link", "zarm_r4_link"]
            else:
                # For 7-DOF arms (14 total arm joints), use zarm_l7_link and zarm_r7_link
                default = ["torso", "zarm_l7_link", "zarm_r7_link", "zarm_l4_link", "zarm_r4_link"]
        else:
            default = ["torso", "zarm_l7_link", "zarm_r7_link", "zarm_l4_link", "zarm_r4_link"]
    else:
        default = ["torso", "zarm_l7_link", "zarm_r7_link", "zarm_l4_link", "zarm_r4_link"]

    kuavo_json = kuavo_ros_param.kuavo_config()
    if kuavo_json is None:
        return default
    
    try:
        kuavo_config = json.loads(kuavo_json)
        # 为了兼容旧版本：优先使用 end_frames_names，如果没有则使用 end_frames_name_ik
        if kuavo_config.get('end_frames_names') is not None:
            return kuavo_config.get('end_frames_names')
        elif kuavo_config.get('end_frames_name_ik') is not None:
            return kuavo_config.get('end_frames_name_ik')
        else:
            return default
    except Exception as e:
        print(f"Failed to get end_frames_name_ik from kuavo_json: {e}")
        return default

def make_robot_param()->dict:
    global kuavo_ros_info
    if kuavo_ros_info is not None:
        return kuavo_ros_info
    
    kuavo_ros_param = RosParameter()

    kuavo_ros_info = {
        'robot_version': kuavo_ros_param.robot_version(),
        'arm_dof': kuavo_ros_param.arm_dof(),
        'head_dof': kuavo_ros_param.head_dof(),
        'leg_dof': kuavo_ros_param.leg_dof(),
        'waist_dof': kuavo_ros_param.waist_dof(),
        'end_effector_type': kuavo_ros_param.end_effector_type(),
        'joint_names': joint_names(),
        'end_frames_names': end_frames_names(),
        'init_stand_height': kuavo_ros_param.init_stand_height()
    }

    # for key, value in kuavo_ros_info.items():
    #     if value is None and key != 'end_effector_type':
    #         SDKLogger.debug(f"[Error]: Failed to get '{key}' from ROS.")
    #         kuavo_ros_info = None
    #         raise Exception(f"[Error]: Failed to get '{key}' from ROS.")

    return kuavo_ros_info        

if __name__ == "__main__":
    rospy.init_node("kuavo_ros_param_test")
    print(make_robot_param())
