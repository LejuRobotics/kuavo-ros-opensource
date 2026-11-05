import os,sys,socket,subprocess,tempfile,time,signal,xmlrpc.client
with socket.socket() as s:
 s.bind(('127.0.0.1',0)); port=s.getsockname()[1]
env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:%d'%port,ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1')
with tempfile.TemporaryDirectory(prefix='sg100-nohardware-') as tmp:
 env.update(ROS_HOME=tmp,ROS_LOG_DIR=tmp)
 with open(tmp+'/master.log','w') as log:
  master=subprocess.Popen(['roscore','-p',str(port)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  child=None
  try:
   for _ in range(100):
    try:
     if xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/sg100_init_test')[0]==1:break
    except OSError:pass
    time.sleep(.1)
   else:raise RuntimeError('master unavailable')
   # Both ports are explicitly empty; no serial device is opened.
   with open(tmp+'/driver.log','w') as driverlog:
    child=subprocess.Popen([sys.argv[1],'_sg100_left_port:=','_sg100_right_port:='],env=env,stdout=driverlog,stderr=subprocess.STDOUT,start_new_session=True)
    deadline=time.monotonic()+8
    initialized=False
    while time.monotonic()<deadline and child.poll() is None:
     time.sleep(.1)
     text=open(tmp+'/driver.log').read()
     if 'SG100 Hand ROS Node initialized successfully' in text:initialized=True;break
    if child.poll() is None:
     child.send_signal(signal.SIGINT)
     try:child.wait(timeout=8)
     except subprocess.TimeoutExpired:child.kill();child.wait()
   # ROS stdout may flush only at shutdown when redirected to a file.
   text=open(tmp+'/driver.log').read()
   initialized='SG100 Hand ROS Node initialized successfully' in text
   print(text[-6500:])
   print('initialized=',initialized,'exit=',child.returncode)
   sys.exit(0 if initialized and child.returncode==0 else 1)
  finally:
   if child is not None and child.poll() is None:child.kill();child.wait()
   os.killpg(master.pid,signal.SIGINT)
   try:master.wait(timeout=8)
   except subprocess.TimeoutExpired:os.killpg(master.pid,signal.SIGKILL);master.wait()
