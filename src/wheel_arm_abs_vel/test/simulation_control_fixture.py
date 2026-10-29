#!/usr/bin/python3
"""Simulation-only Quest protocol fixture with FK-derived reachable hand targets.

Chest orientation follows the Mapper anatomical convention. A 2-link inverse
retarget creates skeleton directions, while palm quaternions invert the public
controller-to-TCP calibration. Output directory contains command.json controls.
"""
import json,time,math
from pathlib import Path
import rospy,tf
import numpy as np
import xml.etree.ElementTree as ET
import sys
from noitom_hi5_hand_udp_python.msg import PoseInfoList,PoseInfo
from kuavo_msgs.msg import JoySticks
from wheel_arm_abs_vel.msg import TeleopTcpTarget
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
rospy.init_node('gui_controls_fixture')
if rospy.get_param('/real', True):
    raise RuntimeError('Synthetic fixture is simulation-only')
root=Path(sys.argv[1] if len(sys.argv)>1 else '/candidate/output/sim-hold')
model=ET.parse(rospy.get_param('/urdfFile')).getroot()
joints=model.findall('joint')
def synthetic_arms(amplitude,t):
    angles={}
    for side,sign in [('l',1),('r',-1)]:
        for i,a in enumerate([-.3+amplitude*math.sin(t),sign*.25,0,-.8+amplitude*math.sin(t),0,0,0]):
            angles['zarm_%s%d_joint'%(side,i+1)]=a
    transforms={'base_link':np.eye(4)}
    pending=list(joints)
    for _ in range(len(joints)):
        progress=False
        for j in pending[:]:
            parent=j.find('parent').get('link');child=j.find('child').get('link')
            if parent not in transforms:continue
            origin=j.find('origin');axis=j.find('axis')
            xyz=[float(x) for x in origin.get('xyz','0 0 0').split()] if origin is not None else [0]*3
            rpy=[float(x) for x in origin.get('rpy','0 0 0').split()] if origin is not None else [0]*3
            T=tf.transformations.euler_matrix(*rpy);T[:3,3]=xyz
            if j.get('type') in ('revolute','continuous'):
                T=T@tf.transformations.rotation_matrix(angles.get(j.get('name'),0),[float(x) for x in axis.get('xyz','1 0 0').split()])
            transforms[child]=transforms[parent]@T;pending.remove(j);progress=True
        if not progress:break
    positions=[];rotations=[]
    for side,sign in [('l',1),('r',-1)]:
        shoulder=np.array([.11575,sign*.2527,1.20516])
        tcp=transforms['zarm_'+side+'7_end_effector'];hand=tcp[:3,3]
        delta=hand-shoulder;dist=np.linalg.norm(delta);direction=delta/dist
        upper=.28440409631367836;fore=.45
        along=(upper*upper-fore*fore+dist*dist)/(2*dist)
        physical_elbow=transforms['zarm_'+side+'4_link'][:3,3]-shoulder
        perpendicular=physical_elbow-direction*np.dot(physical_elbow,direction)
        perpendicular/=np.linalg.norm(perpendicular)
        elbow=shoulder+along*direction+math.sqrt(max(0,upper*upper-along*along))*perpendicular
        fixed=tf.transformations.euler_matrix(-math.pi/2,0,-math.pi if side=='l' else 0)
        calibration=tf.transformations.euler_matrix(-sign*math.pi/12,0,0)
        source=tcp@np.linalg.inv(calibration)@np.linalg.inv(fixed)
        positions.append((shoulder,elbow,hand));rotations.append(tf.transformations.quaternion_from_matrix(source))
    return positions,rotations

posepub=rospy.Publisher('/leju_quest_bone_poses',PoseInfoList,queue_size=2)
joypub=rospy.Publisher('/quest_joystick_data',JoySticks,queue_size=4)
br=tf.TransformBroadcaster()
latest={}
def target(m):
    latest['target']={k:getattr(m,k) for k in ('session_id','generation','absolute_state','control_state','control_valid','torso_control_enabled','arm_hold','arm_operation_seq','left_valid','right_valid')}
def joint(m):latest['joint']={'name':list(m.name),'q':list(m.position),'dq':list(m.velocity)}
def base(m):latest['base']=[m.linear.x,m.linear.y,m.angular.z]
rospy.Subscriber('/mm/teleop_tcp_target',TeleopTcpTarget,target)
rospy.Subscriber('/joint_states',JointState,joint)
rospy.Subscriber('/cmd_vel',Twist,base)
rospy.Subscriber('/quest3/torso_control_enabled',Bool,lambda m:latest.update(torso=m.data))
rospy.set_param('/quest3/connected',True)
start=time.monotonic();lastsave=0
with (root/'observations.jsonl').open('a',buffering=1) as log:
    while not rospy.is_shutdown():
        now=time.monotonic();t=now-start
        try:c=json.loads((root/'command.json').read_text())
        except Exception:c={}
        joy=JoySticks()
        for k,v in c.get('joy',{}).items():setattr(joy,k,v)
        if c.get('joy_enabled', True): joypub.publish(joy)
        stamp=rospy.Time.now();chest=(0,0,1.2+float(c.get('chest_height_offset',0.)))
        yaw=c.get('torso',0.)*math.sin(t)+float(c.get('torso_offset',0.))
        q=tf.transformations.quaternion_from_euler(0,0,yaw)
        arms,rotations=synthetic_arms(float(c.get('arm',0)),t)
        points=[arms[0][0],arms[0][1],arms[1][0],arms[1][1],arms[0][2],arms[1][2]]
        points += [arms[0][2]]*8+[arms[1][2]]*8+[np.array((0,0,.9)),np.array(chest),np.array((0,0,1.4)),np.array((0,0,1.55))]
        msg=PoseInfoList(timestamp_ms=int(time.time()*1000),is_high_confidence=True,is_hand_tracking=bool(c.get('hand_tracking',False)))
        chest_q=tf.transformations.quaternion_multiply(q,tf.transformations.quaternion_from_euler(math.pi/2,0,math.pi/2))
        for i,(x,y,z) in enumerate(points):
            pose=PoseInfo();pose.position.x=math.cos(yaw)*x-math.sin(yaw)*y;pose.position.y=math.sin(yaw)*x+math.cos(yaw)*y;pose.position.z=z
            rotation=chest_q
            if i in (4,5):rotation=tf.transformations.quaternion_multiply(q,rotations[i-4])
            pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=rotation
            msg.poses.append(pose)
        if msg.is_hand_tracking:
            # Public optical finger encoding: roll relative to the palm.
            gesture=c.get('optical_gesture','neutral')
            for palm_index, first_finger in ((4,10),(5,18)):
                palm=msg.poses[palm_index].orientation
                palm_q=(palm.x,palm.y,palm.z,palm.w)
                for finger in range(4):
                    bent=(gesture=='ok' and finger==0) or (gesture=='stop' and finger in (1,2))
                    qfinger=tf.transformations.quaternion_multiply(palm_q,
                        tf.transformations.quaternion_from_euler(1.2 if bent else 0.,0.,0.))
                    orientation=msg.poses[first_finger+finger].orientation
                    orientation.x,orientation.y,orientation.z,orientation.w=qfinger
        if c.get('body',True):
            posepub.publish(msg)
            br.sendTransform(chest,q,stamp,'Chest','torso')
            head=tf.transformations.quaternion_from_euler(0,c.get('head_pitch',0.),yaw+c.get('head_yaw',0.))
            br.sendTransform((0,0,1.55),head,stamp,'Head','torso')
        if now-lastsave>.05:
            record={'t':now,'command':c,**latest};log.write(json.dumps(record)+'\n');lastsave=now
            tmp=root/'latest.tmp';tmp.write_text(json.dumps(record));tmp.replace(root/'latest.json')
        time.sleep(.01)
