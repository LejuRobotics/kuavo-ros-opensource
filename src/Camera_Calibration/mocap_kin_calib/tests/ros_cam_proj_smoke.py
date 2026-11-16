"""Synthetic ROS pipeline test on a private localhost master; never connects to hardware.
Run after sourcing ROS and this workspace.
"""
import os, sys, time, signal, subprocess, threading, json, xmlrpc.client, tempfile, socket
from pathlib import Path
import numpy as np
test_dir=Path(tempfile.mkdtemp(prefix='cam_proj_ros_smoke_'))
with socket.socket() as sock:
 sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
os.environ.update(ROS_MASTER_URI=f'http://127.0.0.1:{port}', ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1', ROS_LOG_DIR=str(test_dir/'log'), ROS_HOME=str(test_dir/'ros_home'))
log=(test_dir/'roscore.log').open('w')
core=subprocess.Popen(['/opt/ros/noetic/bin/roscore','-p',str(port)],stdout=log,stderr=log,start_new_session=True)
try:
 for _ in range(50):
  try:
   xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/projection_test');break
  except OSError: time.sleep(.1)
 else: raise RuntimeError('isolated roscore did not start')
 import rospy
 from cv_bridge import CvBridge
 from sensor_msgs.msg import Image, CameraInfo
 from kuavo_msgs.msg import sensorsData
 from tf2_msgs.msg import TFMessage
 from geometry_msgs.msg import TransformStamped
 sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
 import cam_proj_validation as p
 rospy.init_node('projection_synthetic_test',disable_signals=True)
 rospy.set_param('/head_camera/camera/color_rotation',180)
 args=p.parse_args(['--robot-version','56','--no-display','--stale-after','0.25','--save-dir',str(test_dir/'snapshots')])
 viewer=p.ProjectionViewer(args)
 outputs=[]
 received=rospy.Subscriber(args.output_topic,Image,lambda m:outputs.append(m),queue_size=10)
 image_pub=rospy.Publisher(args.image_topic,Image,queue_size=1)
 info_pub=rospy.Publisher(args.camera_info_topic,CameraInfo,queue_size=1,latch=True)
 joint_pub=rospy.Publisher(args.joint_topic,sensorsData,queue_size=1)
 tf_pub=rospy.Publisher('/tf_static',TFMessage,queue_size=1,latch=True)
 thread=threading.Thread(target=viewer.run);thread.start()
 time.sleep(.3)
 bridge=CvBridge()
 info=CameraInfo();info.width=640;info.height=480;info.header.frame_id='head_camera_color_optical_frame'
 info.K=[400,0,320,0,400,240,0,0,1];info.D=[0]*5;info.distortion_model='plumb_bob';info_pub.publish(info)
 image=bridge.cv2_to_imgmsg(np.full((480,640,3),60,np.uint8),'bgr8');image.header.frame_id=info.header.frame_id
 joint=sensorsData();q=np.zeros(29);q[13]=q[20]=-1.0;q[28]=.3;joint.joint_data.joint_q=q.tolist()
 def feed(count,stamp_offset=0):
  for _ in range(count):
   now=rospy.Time.now();joint.header.stamp=now;joint_pub.publish(joint)
   image.header.stamp=now+rospy.Duration(stamp_offset);image_pub.publish(image);time.sleep(.035)
 feed(10)
 assert outputs and viewer.last_save is None, 'missing TF should show raw image without valid overlay'
 print('PASS missing optical TF rejects overlay and still publishes image')
 t=TransformStamped();t.header.frame_id='head_camera_link';t.child_frame_id=info.header.frame_id
 t.transform.rotation.x=-.5;t.transform.rotation.y=.5;t.transform.rotation.z=-.5;t.transform.rotation.w=.5
 tf_pub.publish(TFMessage([t]));time.sleep(.1);feed(20)
 assert viewer.last_save is not None, 'valid frame did not produce overlay'
 raw,overlay,meta=viewer.last_save
 assert 'nominal' in meta['extrinsic_source']
 assert meta['pixel_rotation_deg']==0, 'driver rotation parameter must not blindly rotate projection'
 assert meta['mask_pixels']>0 and np.any(overlay[40:]!=raw[40:]), 'mesh not drawn'
 assert outputs[-1].header.frame_id==info.header.frame_id
 viewer.save()
 paths=list(Path(args.save_dir).glob('*/metadata.json'));assert len(paths)==1
 saved=json.loads(paths[0].read_text());assert saved['urdf_sha256']==viewer.model.sha256 and len(saved['joint_q_rad'])==17
 print('PASS live ROS Image/CameraInfo/sensorsData/tf_static -> overlay with original image header + three-file snapshot')
 feed(7,stamp_offset=1.0)
 assert viewer.last_save is None, 'timestamp mismatch not rejected'
 print('PASS image/feedback mismatch rejects overlay')
 args.sync_mode='receipt'
 feed(15,stamp_offset=40.0)
 assert viewer.last_save is not None, 'explicit receipt mode did not handle unsynchronized source clocks'
 receipt_meta=viewer.last_save[2]
 assert receipt_meta['sync_mode']=='receipt' and receipt_meta['capture_time_alignment']=='unverified_receipt'
 assert receipt_meta['header_skew_ms'] < -39000 and abs(receipt_meta['receipt_skew_ms']) < 100
 assert not receipt_meta['header_stamps_within_tolerance'] and receipt_meta['mask_pixels']>0
 print('PASS receipt matching produces overlay and preserves unsynchronized header evidence')
 args.sync_mode='header'
 feed(10)
 assert viewer.last_save is not None
 time.sleep(.4)
 assert viewer.last_save is None and outputs[-1].height==100, 'headless output did not clear stopped stream'
 print('PASS stopped camera clears headless ROS overlay')
 feed(10)
 assert viewer.last_save is not None
 bad=Image();bad.header.frame_id=info.header.frame_id;bad.height=480;bad.width=640;bad.encoding='unsupported_encoding';bad.step=640;bad.data=bytes(640*480)
 bad.header.stamp=rospy.Time.now();image_pub.publish(bad);time.sleep(.13)
 assert viewer.last_save is None and outputs[-1].height==100, 'bad image retained previous overlay'
 print('PASS corrupt image clears headless ROS overlay')
 # Actual compressed-message bridge path, using the same renderer and model.
 import cv2
 args.compressed=True
 args.image_rotation="180"
 compressed=bridge.cv2_to_compressed_imgmsg(np.full((480,640,3),60,np.uint8));compressed.header.frame_id=info.header.frame_id
 compressed.header.stamp=rospy.Time.now();joint.header.stamp=compressed.header.stamp;viewer.on_joints(joint)
 result=viewer.process(compressed,time.monotonic())
 assert viewer.last_save is not None and result.shape==(480,640,3)
 print('PASS compressed image decode and projection')
 args.compressed=False
 args.image_rotation="0"
 feed(2)
 assert viewer.check_inputs(), 'read-only preflight failed with all real-format inputs'
 print('PASS input preflight, including automatic candidate mount + driver TF join')
 rospy.signal_shutdown('synthetic test complete');thread.join(3)
 assert not thread.is_alive()
 print('PASS isolated local ROS test complete; no hardware connection; artifacts:',test_dir)
finally:
 if 'rospy' in globals() and not rospy.is_shutdown(): rospy.signal_shutdown('cleanup')
 os.killpg(core.pid,signal.SIGINT)
 try: core.wait(timeout=8)
 except subprocess.TimeoutExpired: os.killpg(core.pid,signal.SIGTERM);core.wait(timeout=3)
 log.close()
