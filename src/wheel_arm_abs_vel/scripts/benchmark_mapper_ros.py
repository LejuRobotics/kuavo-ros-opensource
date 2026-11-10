#!/usr/bin/env python3
"""Isolated TCPROS Mapper benchmark. Never starts hardware/IK or uses port 11311.

--extract creates immutable serialized raw input fixtures from a recorded bag.
--server runs the actual IkRos constructor, callback, publishers and supervisor.
--drive sends bone + joystick + 500Hz feedback over real loopback TCPROS and
receives the serialized TeleopTcpTarget. Numerical equivalence is a separate gate.
All roles require an explicitly isolated localhost ROS master.
"""
import argparse, gc, io, json, os, pickle, sys, threading, time
from pathlib import Path

def stats(v):
    import numpy as np
    x=np.asarray(v,dtype=float)
    return dict(n=len(x),p50_ms=float(np.median(x)),p95_ms=float(np.percentile(x,95)),p99_ms=float(np.percentile(x,99)),max_ms=float(max(x)),over6_pct=float(np.mean(x>=6)*100)) if len(x) else dict(n=0)

def extract(a):
    import rosbag
    bones=[]; samples={}
    names=['/leju_quest_bone_poses','/quest_joystick_data','/sensors_data_raw','/robot_arm_q_v_tau','/humanoid_controller/optimizedState_wbc_mrt_origin','/mobile_manipulator_mpc_observation']
    with rosbag.Bag(a.bag) as bag:
        for topic,msg,stamp in bag.read_messages(topics=names):
            b=io.BytesIO();msg.serialize(b);row=(msg._type,b.getvalue())
            if topic==names[0]:
                if len(bones)<a.frames:bones.append(row)
            else:samples.setdefault(topic,row)
            if len(bones)>=a.frames and all(t in samples for t in names[1:]):break
    Path(a.fixture).write_bytes(pickle.dumps((bones,samples),protocol=4))
    print('EXTRACTED',len(bones),list(samples),flush=True)

def guard():
    uri=os.environ.get('ROS_MASTER_URI','')
    if not uri.startswith('http://127.0.0.1:') or ':11311' in uri:
        raise RuntimeError('Dedicated loopback ROS master required; default/robot graph forbidden')

def server(a):
    guard()
    sys.path.insert(0,str(Path(a.scripts_dir).resolve()))
    if a.optimized:
        from tools.mapper_input_scheduling import configure_mapper_interpreter
        configure_mapper_interpreter()
    import rospy
    rospy.init_node('ik_ros_uni',disable_signals=True)
    params=json.loads(Path(a.params).read_text())
    rospy.set_param('/',params)
    rospy.set_param('/use_sim_time',False)
    rospy.set_param('/robot_type',1)  # Recorded wheel-arm launch; math snapshot omits this global.
    rospy.set_param('~noncontrol_publish_async',True)
    rospy.set_param('~noncontrol_publish_rate_hz',30.0 if a.optimized else 100.0)
    rospy.set_param('~mapping_debug_rate_hz',10.0)
    rospy.set_param('~enable_velocity_ik_takeover',True)
    import ik_ros_velocity as mod
    import roslib.message
    from noitom_hi5_hand_udp_python.msg import PoseInfoList
    from tools.noncontrol_publish_process import CHANNELS
    from wheel_arm_abs_vel.msg import TeleopTcpTarget
    local=threading.local();rows=[];gc_rows=[];decode_rows=[];pub_totals={};serialization_totals={};errors=[]
    gc_start={}
    def gc_hook(phase,info):
        key=info['generation']
        if phase=='start':gc_start[key]=time.monotonic_ns()
        else:
            end=time.monotonic_ns();start=gc_start.pop(key,end)
            gc_rows.append([key,(end-start)*1e-6,start,end,threading.get_native_id()])
    gc.callbacks.append(gc_hook)
    # Observe actual serializers; no dummy publishers, no stripped ROS path.
    types=[c[1] for c in CHANNELS]+['wheel_arm_abs_vel/TeleopTcpTarget','kuavo_msgs/twoArmHandPoseCmd','std_msgs/Float32MultiArray','geometry_msgs/Twist','kuavo_msgs/robotHandPosition','kuavo_msgs/lejuClawCommand','kuavo_msgs/headBodyPose']
    for typ in set(types):
        cls=roslib.message.get_message_class(typ)
        if cls is None:continue
        original=cls.serialize
        def serialize(msg,buff,_original=original,_typ=typ):
            w,c=time.monotonic_ns(),time.thread_time_ns()
            try:return _original(msg,buff)
            finally:
                serialization_totals[_typ]=serialization_totals.get(_typ,0)+1
                item=getattr(local,'row',None)
                if item is not None:
                    item['serialize_ms']+=(time.monotonic_ns()-w)*1e-6
                    item['serialize_cpu_ms']+=(time.thread_time_ns()-c)*1e-6
                    item['serializers'][_typ]=item['serializers'].get(_typ,0)+1
        cls.serialize=serialize
    stages = [(mod.Quest3ArmInfoTransformer, "read_msg"),
              (mod.IkRos, "_tracking_source_policy"),
              (mod.IkRos, "_build_absolute_target_snapshot"),
              (mod.IkRos, "_populate_whole_body_sample"),
              (mod.IkRos, "_publish_quest_tcp_mapping_debug"),
              (mod.IkRos, "_handle_absolute_target_and_gesture")]
    for cls, name in stages:
        original = getattr(cls, name, None)
        if original is None:
            continue  # Optional diagnostics differ between release branches.
        def stage(self,*args,_original=original,_name=name,**kw):
            w,c=time.monotonic_ns(),time.thread_time_ns()
            try:return _original(self,*args,**kw)
            finally:
                item=getattr(local,'row',None)
                if item is not None:
                    item[_name+'_wall_ms']=(time.monotonic_ns()-w)*1e-6
                    item[_name+'_cpu_ms']=(time.thread_time_ns()-c)*1e-6
        setattr(cls,name,stage)
    original_decode=PoseInfoList.deserialize
    def decode(msg,data):
        w,c=time.monotonic_ns(),time.thread_time_ns()
        try:return original_decode(msg,data)
        finally:
            local.decode_end=time.monotonic_ns();local.decode_ms=(local.decode_end-w)*1e-6
            local.decode_cpu=(time.thread_time_ns()-c)*1e-6
            decode_rows.append([local.decode_ms,local.decode_cpu])
    PoseInfoList.deserialize=decode
    publish=rospy.Publisher.publish
    def pub(self,*args,**kw):
        item=getattr(local,'row',None);w=time.monotonic_ns()
        try:return publish(self,*args,**kw)
        finally:
            pub_totals[self.resolved_name]=pub_totals.get(self.resolved_name,0)+1
            if item is not None:
                item['publish_ms']+=(time.monotonic_ns()-w)*1e-6
                item['publications'][self.resolved_name]=item['publications'].get(self.resolved_name,0)+1
                if self.resolved_name=='/mm/teleop_tcp_target':
                    item['to_target_ms']=(time.monotonic_ns()-item['start_ns'])*1e-6
                    msg=args[0];item['control_valid']=bool(msg.control_valid);item['control_state']=int(msg.control_state)
    rospy.Publisher.publish=pub
    callback=mod.IkRos._scheduled_quest_bone_poses_callback
    last_end=[0]
    def timed(self,msg):
        w,c=time.monotonic_ns(),time.thread_time_ns()
        row=dict(start_ns=w,stamp=int(msg.timestamp_ms),serialize_ms=0.,serialize_cpu_ms=0.,publish_ms=0.,publications={},serializers={},
                 decode_ms=getattr(local,'decode_ms',0.),decode_cpu_ms=getattr(local,'decode_cpu',0.),
                 decoded_to_entry_ms=(w-getattr(local,'decode_end',w))*1e-6,
                 entry_gap_ms=(w-last_end[0])*1e-6 if last_end[0] else 0.)
        local.row=row
        try:return callback(self,msg)
        except Exception as e:errors.append(repr(e));raise
        finally:
            end=time.monotonic_ns();row['wall_ms']=(end-w)*1e-6;row['cpu_ms']=(time.thread_time_ns()-c)*1e-6
            row['off_cpu_ms']=row['wall_ms']-row['cpu_ms'];rows.append(row);last_end[0]=end;local.row=None
    mod.IkRos._scheduled_quest_bone_poses_callback=timed
    def run(self):
        # Explicit synthetic ACTIVE lifecycle, on a graph with no actuator.
        # All subsequent source/freshness/permission checks and supervisor work
        # run unchanged. Output records expose any transition to HOLD/inactive.
        self._absolute_state=mod.AbsoluteEnableState.ACTIVE
        self._absolute_gesture_running=True;self._fixed_control_enabled=True
        self._absolute_session_id=1;self._absolute_generation=1
        self._absolute_mode_status=(2,2,False)
        self._absolute_mode_status_time=time.monotonic()
        self.arm_mode_changing=False
        self.quest3_arm_info_transformer.is_runing=True
        Path(a.output+'.ready').write_text(str(os.getpid()))
        deadline=time.monotonic()+a.seconds
        while time.monotonic()<deadline and not rospy.is_shutdown():time.sleep(.05)
        self.stop_event.set();rospy.signal_shutdown('private benchmark complete')
    mod.IkRos.run=run
    mod.IkRos(None,end_effector_type=a.ee,send_srv=False,predict_gesture=False)
    usable=rows[100:]
    result=dict(scope='actual IkRos + ROS construction/serialization + TCPROS feedback; synthetic ACTIVE lifecycle, no hardware or C++ IK',
                gc_enabled=gc.isenabled(),gc_threshold=gc.get_threshold(),gc_frozen=gc.get_freeze_count(),gc_pauses=gc_rows,
                runtime_gc_pauses=[event for event in gc_rows if rows and event[2]>=rows[0]['start_ns'] and event[2]<=rows[-1]['start_ns']+rows[-1]['wall_ms']*1e6],
                errors=errors,frames=len(rows),active_fraction=sum(bool(r.get('control_valid')) for r in usable)/max(1,len(usable)),all_publications=pub_totals,all_serializations=serialization_totals,rows=rows,
                summary={k:stats([r[k] for r in usable if k in r]) for k in ['wall_ms','cpu_ms','off_cpu_ms','serialize_ms','serialize_cpu_ms','publish_ms','to_target_ms','decode_ms','decode_cpu_ms','decoded_to_entry_ms','entry_gap_ms']+[name+suffix for _,name in stages for suffix in ('_wall_ms','_cpu_ms')]})
    Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k not in ('rows','gc_pauses','runtime_gc_pauses')},indent=2),flush=True)
    if errors or result['active_fraction']<.95:raise RuntimeError('Benchmark must exercise ACTIVE control on at least 95% of measured frames')

def receive(a):
    guard()
    import rospy,roslib.message
    from wheel_arm_abs_vel.msg import TeleopTcpTarget
    from tools.noncontrol_publish_process import CHANNELS
    rospy.init_node('mapper_bench_output',disable_signals=True)
    arrivals=[];native=None
    native_path=os.environ.get('MAPPER_BENCH_CPP_SINK')
    if native_path:
        import subprocess
        native=subprocess.Popen([native_path,a.output,str(a.seconds)])
    def recv(m):
        arrivals.append([round(m.header.stamp.to_sec()*1000),time.monotonic_ns(),int(m.control_state),bool(m.control_valid)])
    sinks=[] if native else [rospy.Subscriber('/mm/teleop_tcp_target',TeleopTcpTarget,recv,queue_size=1,tcp_nodelay=True,buff_size=2**20)]
    outputs=[(t,typ) for t,typ,_ in CHANNELS]+[
        ('/control_robot_hand_position','kuavo_msgs/robotHandPosition'),
        ('/leju_claw_command','kuavo_msgs/lejuClawCommand'),
        ('/whole_body_velocity_ik/base_cmd_vel','geometry_msgs/Twist'),
        ('/kuavo_head_body_orientation_data','kuavo_msgs/headBodyPose'),
        ('/kuavo_head_body_orientation','kuavo_msgs/headBodyPose')]
    sinks += [rospy.Subscriber(t,roslib.message.get_message_class(typ),lambda m:None,queue_size=1,tcp_nodelay=True) for t,typ in outputs]
    time.sleep(.3);Path(a.output+'.ready').write_text(str(os.getpid()))
    deadline=time.monotonic()+a.seconds
    while time.monotonic()<deadline and not Path(a.output+'.stop').exists():time.sleep(.02)
    if native:
        native.wait(timeout=5)
        if native.returncode:raise RuntimeError('C++ ROS sink failed')
    else:Path(a.output).write_text(json.dumps(arrivals))
    rospy.signal_shutdown('output complete')

def feedback(a):
    guard()
    import rospy,roslib.message
    _,samples=pickle.loads(Path(a.fixture).read_bytes())
    samples={t:roslib.message.get_message_class(row[0])().deserialize(row[1]) for t,row in samples.items()}
    rospy.init_node('mapper_bench_hardware_feedback',disable_signals=True)
    pubs={t:rospy.Publisher(t,type(m),queue_size=1,tcp_nodelay=True) for t,m in samples.items()}
    tick=0;next_=time.monotonic();deadline=next_+a.seconds
    while time.monotonic()<deadline and not Path(a.output+'.stop').exists():
        for topic,p in pubs.items():
            if topic=='/quest_joystick_data' and tick%4:continue
            m=samples[topic]
            if hasattr(m,'header'):m.header.stamp=rospy.Time.now()
            if topic=='/mobile_manipulator_mpc_observation':m.time=10.
            p.publish(m)
        tick+=1;next_+=.002;time.sleep(max(0,next_-time.monotonic()))
    rospy.signal_shutdown('feedback complete')

def drive(a):
    guard()
    import rospy,roslib.message,subprocess
    bones,_=pickle.loads(Path(a.fixture).read_bytes())
    bones=[roslib.message.get_message_class(row[0])().deserialize(row[1]) for row in bones[:a.frames]]
    span=(bones[-1].timestamp_ms-bones[0].timestamp_ms)*.001
    rospy.init_node('mapper_bench_input',disable_signals=True)
    bone_pub=rospy.Publisher('/leju_quest_bone_poses',type(bones[0]),queue_size=1,tcp_nodelay=True)
    children=[];sent={};sink=a.output+'.sink.json'
    for suffix in ('.ready','.stop'):Path(sink+suffix).unlink(missing_ok=True)
    try:
        # Real hardware feedback and Quest ingress are separate processes. Keep
        # their producer GILs separate here too; Mapper still receives/decodes
        # every subscribed stream in its own interpreter, unchanged.
        for role in ('receive','feedback'):
            children.append(subprocess.Popen([sys.executable,__file__,role,'--fixture',a.fixture,'--output',sink,'--seconds',str(span+8)]))
        deadline=time.monotonic()+10
        while time.monotonic()<deadline and (not Path(sink+'.ready').exists() or bone_pub.get_num_connections()==0):time.sleep(.02)
        if not Path(sink+'.ready').exists() or not bone_pub.get_num_connections():raise RuntimeError('benchmark connections failed')
        time.sleep(.5);start=time.monotonic();first=bones[0].timestamp_ms
        for m in bones:
            wait=start+(m.timestamp_ms-first)*.001-time.monotonic()
            if wait>0:time.sleep(wait)
            sent[int(m.timestamp_ms)]=time.monotonic_ns();bone_pub.publish(m)
        time.sleep(.5);Path(sink+'.stop').touch()
        for child in children:
            child.wait(timeout=10)
            if child.returncode:raise RuntimeError('benchmark child failed')
        arrivals=[[key,(end-sent[key])*1e-6,state,valid] for key,end,state,valid in json.loads(Path(sink).read_text()) if key in sent]
        result=dict(sent=len(bones),received=len(arrivals),source_span_s=span,
                    producer_processes='separate Quest publisher, 500Hz hardware-feedback publisher and output receiver',
                    output_receiver='roscpp AsyncSpinner(2)' if os.environ.get('MAPPER_BENCH_CPP_SINK') else 'rospy',
                    end_to_end=stats([r[1] for r in arrivals[100:]]),arrivals=arrivals)
        Path(a.output).write_text(json.dumps(result,indent=2));print({k:v for k,v in result.items() if k!='arrivals'},flush=True)
    finally:
        Path(sink+'.stop').touch()
        for child in children:
            if child.poll() is None:
                child.terminate()
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:child.kill();child.wait()
        rospy.signal_shutdown('input complete')

def main():
    p=argparse.ArgumentParser();p.add_argument('role',choices=['extract','server','drive','receive','feedback']);p.add_argument('--scripts-dir');p.add_argument('--params');p.add_argument('--bag');p.add_argument('--fixture',required=True);p.add_argument('--output');p.add_argument('--frames',type=int,default=2400);p.add_argument('--seconds',type=float,default=35);p.add_argument('--optimized',action='store_true');p.add_argument('--ee',default='qiangnao');a=p.parse_args()
    globals()[a.role](a)
if __name__=='__main__':main()
