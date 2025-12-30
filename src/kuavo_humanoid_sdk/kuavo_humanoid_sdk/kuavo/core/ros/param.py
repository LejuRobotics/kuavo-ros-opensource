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


def joint_names()->dict:
    leg_link_names = [
        'leg_l1_link', 'leg_l2_link', 'leg_l3_link', 'leg_l4_link', 'leg_l5_link', 'leg_l6_link',
        'leg_r1_link', 'leg_r2_link', 'leg_r3_link', 'leg_r4_link', 'leg_r5_link', 'leg_r6_link'
    ]
    arm_link_names = [
        'zarm_l1_link', 'zarm_l2_link', 'zarm_l3_link', 'zarm_l4_link', 'zarm_l5_link', 'zarm_l6_link', 'zarm_l7_link',
        'zarm_r1_link', 'zarm_r2_link', 'zarm_r3_link', 'zarm_r4_link', 'zarm_r5_link', 'zarm_r6_link', 'zarm_r7_link',
    ]
    head_link_names = [
        'zhead_1_link', 'zhead_2_link'
    ]

    kuavo_ros_param = RosParameter()

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
    process_link_name = lambda link_name: (
        (root.find(f".//link[@name='{link_name}']") is not None and
        root.find(f".//link[@name='{link_name}']/visual") is not None and
        root.find(f".//link[@name='{link_name}']/visual/geometry") is not None and
        root.find(f".//link[@name='{link_name}']/visual/geometry/mesh") is not None and
        root.find(f".//link[@name='{link_name}']/visual/geometry/mesh").get("filename") is not None)
        and (
            # Extract the basename (without path and extension)
            root.find(f".//link[@name='{link_name}']/visual/geometry/mesh")
            .get("filename")
            .split("/")[-1]
            .split(".")[0]
        )
        or (
            SDKLogger.warn(f"Warning: {link_name} is not found or incomplete in robot_desc"),
            None
        )[1]  # Return None after printing the warning
    )
    leg_joint_names = [process_link_name(link_name) for link_name in leg_link_names if process_link_name(link_name) is not None]
    arm_joint_names = [process_link_name(link_name) for link_name in arm_link_names if process_link_name(link_name) is not None]
    head_joint_names = [process_link_name(link_name) for link_name in head_link_names if process_link_name(link_name) is not None]

    if len(leg_link_names) != len(leg_joint_names):
        SDKLogger.warn(f"leg_joint_names is not equal to leg_link_names, {len(leg_link_names)} != {len(leg_joint_names)}")
        return None
    if len(arm_link_names)!= len(arm_joint_names):
        SDKLogger.warn(f"arm_joint_names is not equal to arm_link_names, {len(arm_link_names)}!= {len(arm_joint_names)}")
        return None
    if len(head_link_names)!= len(head_joint_names):
        SDKLogger.warn(f"head_joint_names is not equal to head_link_names, {len(head_link_names)}!= {len(head_joint_names)}")
        return None
    
    return leg_joint_names + arm_joint_names + head_joint_names

kuavo_ros_info = None

def end_frames_names()->dict:
    default = ["torso", "zarm_l7_link", "zarm_r7_link", "zarm_l4_link", "zarm_r4_link"]

    kuavo_ros_param = RosParameter()

    kuavo_json = kuavo_ros_param.kuavo_config()
    if kuavo_json is None:
        return default
    
    try:
        kuavo_config = json.loads(kuavo_json)
        if kuavo_config.get('end_frames_names') is not None:
            return kuavo_config.get('end_frames_names')
        else:
            return default
    except Exception as e:
        print(f"Failed to get end_frames_names from kuavo_json: {e}")
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
        'end_effector_type': kuavo_ros_param.end_effector_type(),
        'joint_names': joint_names(),
        'end_frames_names': end_frames_names(),
    }

    for key, value in kuavo_ros_info.items():
        if value is None and key != 'end_effector_type':
            SDKLogger.debug(f"[Error]: Failed to get '{key}' from ROS.")
            kuavo_ros_info = None
            raise Exception(f"[Error]: Failed to get '{key}' from ROS.")

    return kuavo_ros_info        

if __name__ == "__main__":
    rospy.init_node("kuavo_ros_param_test")
    print(make_robot_param())