#!/usr/bin/env python3
"""A blocked discovery/ROS-master call must not block production UDP receive."""
import ast, socket, threading, time, unittest, importlib.util, json
from pathlib import Path
from types import SimpleNamespace as NS
ROOT=Path(__file__).resolve().parents[1]
class ReceiveIsolationTest(unittest.TestCase):
 def test_app_can_discover_before_sending_any_pose_packet(self):
  tree=ast.parse((ROOT/'scripts/monitor_pico.py').read_text())
  cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='PicoQuestReceiver')
  methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('_receive_packets','_publish_discovery')]
  spec=importlib.util.spec_from_file_location('discovery',ROOT/'scripts/pico_input/discovery.py')
  discovery=importlib.util.module_from_spec(spec);spec.loader.exec_module(discovery)
  stop=threading.Event()
  scope=dict(threading=threading,time=time,socket=socket,SourceClockGate=lambda age:NS(reset=lambda:None),
             rospy=NS(is_shutdown=stop.is_set,get_param=lambda *a:'lejuclaw',logwarn_throttle=lambda *a:None),
             announce=discovery.announce)
  code=ast.ClassDef(name='Receiver',bases=[],keywords=[],body=methods,decorator_list=[])
  exec(compile(ast.fix_missing_locations(ast.Module(body=[code],type_ignores=[])),'<production>','exec'),scope)
  # Loopback stands in for the App broadcast listener; no LAN/hardware traffic.
  with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as app, \
       socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as incoming, \
       socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as advertiser:
   app.bind(('127.0.0.1',8443));app.settimeout(2.)
   incoming.bind(('127.0.0.1',0));incoming.settimeout(.02)
   n=scope['Receiver']();n._input_max_age=.25;n._closed=False
   n._discovery_stop=threading.Event();n._discovery_thread=None
   n.advertiser=advertiser;n.discovery_pairs=[('192.168.5.19','127.0.0.1')];n.sock=incoming
   packets=[];n.handle=lambda *args:packets.append(args);n._disconnect=lambda key:None
   worker=threading.Thread(target=n._receive_packets,daemon=True);worker.start()
   try:
    for _ in range(2):
     packet=json.loads(app.recvfrom(65535)[0])
     self.assertEqual(packet,{'data':{'robot_name':'KUAVO','robot_ip':'192.168.5.19','eef_type':'lejuclaw'}})
    self.assertEqual(packets,[],'Discovery must not wait for headset input')
   finally:
    n._discovery_stop.set();stop.set();worker.join(1.)
    if n._discovery_thread:n._discovery_thread.join(1.)
   self.assertFalse(worker.is_alive())
   self.assertFalse(n._discovery_thread.is_alive())

 def test_blocked_discovery_keeps_receiving(self):
  tree=ast.parse((ROOT/'scripts/monitor_pico.py').read_text())
  cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='PicoQuestReceiver')
  methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('_receive_packets','_publish_discovery')]
  scope={};stop=threading.Event();entered=threading.Event();release=threading.Event();delivered=threading.Event();count=[0]
  def param(*args):entered.set();release.wait(2);return 'qiangnao'
  scope.update(threading=threading,time=time,socket=socket,SourceClockGate=lambda age:NS(reset=lambda:None),
   rospy=NS(is_shutdown=stop.is_set,get_param=param,logwarn_throttle=lambda *a:None),announce=lambda *a:None)
  code=ast.ClassDef(name='Receiver',bases=[],keywords=[],body=methods,decorator_list=[])
  exec(compile(ast.fix_missing_locations(ast.Module(body=[code],type_ignores=[])),'<production>','exec'),scope)
  n=scope['Receiver']();n._input_max_age=.25;n._closed=False;n._discovery_stop=threading.Event();n._discovery_thread=None
  n.advertiser=object();n.discovery_pairs=[]
  def recv(size):time.sleep(.002);return b'data',('127.0.0.1',12345)
  n.sock=NS(recvfrom=recv)
  def handle(raw,address,now):
   count[0]+=1
   n.last_fresh['body']=n.last_fresh['joy']=now
   if count[0]>=10:delivered.set()
  n.handle=handle;n._disconnect=lambda key:None
  worker=threading.Thread(target=n._receive_packets,daemon=True);worker.start()
  try:
   self.assertTrue(entered.wait(1),'discovery not exercised')
   self.assertTrue(delivered.wait(.2),'discovery/parameter RPC blocked UDP input')
  finally:
   n._discovery_stop.set();stop.set();release.set();worker.join(1)
   if n._discovery_thread:n._discovery_thread.join(1)
  self.assertFalse(worker.is_alive())
if __name__=='__main__':unittest.main()
