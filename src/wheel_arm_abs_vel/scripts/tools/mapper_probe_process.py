"""Bounded observation-only IPC, outside the Mapper interpreter."""
import array, json, os, socket, struct, subprocess, sys, time
from pathlib import Path

class ProbeProcess:
    def __init__(self):
        self.dropped=0; self.errors=0; self.closed=False
        self.socket, child=socket.socketpair(socket.AF_UNIX,socket.SOCK_SEQPACKET)
        self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,262144)
        self.socket.setblocking(False)
        self.process=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),str(child.fileno()),str(os.getpid())],pass_fds=(child.fileno(),),close_fds=True,start_new_session=True)
        child.close()
    def submit(self,frame,serial_drops=0,serial_errors=0):
        data=[1,os.getpid(),frame['tid'],frame['serial'],frame['session'],frame['generation'],frame['sequence'],frame['source_ms'],self.dropped+serial_drops,self.errors+serial_errors,frame['overflow'],len(frame['points'])]
        for row in frame['points']: data.extend(row)
        payload=struct.pack('<%dd'%len(data),*data)
        try:
            if len(payload)>8192: raise ValueError('oversized probe')
            self.socket.send(payload,socket.MSG_DONTWAIT|socket.MSG_NOSIGNAL)
        except BlockingIOError: self.dropped+=1
        except (OSError,ValueError): self.dropped+=1;self.errors+=1
    def close(self):
        if self.closed:return
        self.closed=True;self.socket.close()
        try:self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()

def identities(pid):
    rows=[]
    for p in sorted(Path('/proc',str(pid),'task').glob('*')):
        try:
            tid=int(p.name)
            rows.append(dict(tid=tid,name=(p/'comm').read_text().strip(),target='',bindings=[],stack=[],cpus=sorted(os.sched_getaffinity(tid)),policy=os.sched_getscheduler(tid),priority=os.sched_getparam(tid).sched_priority))
        except (OSError,ValueError):pass
    return dict(version=1,pid=pid,monotonic_ns=time.monotonic_ns(),threads=rows,thread_count=len(rows),truncated=False,wall_ns=0,cpu_ns=0,identity_source='external_proc_current_threads',probe_process=os.getpid(),marker_processes=[],noncontrol_processes=[])

def worker(fd,parent_pid):
    try:
        from .background_fence import apply_background_fence
    except ImportError:
        from background_fence import apply_background_fence
    apply_background_fence()
    import select,rospy
    from std_msgs.msg import Float64MultiArray,String
    os.nice(10)
    sock=socket.socket(fileno=fd)
    rospy.init_node('mapper_probe_worker_'+str(parent_pid),disable_signals=True)
    pub=rospy.Publisher('/whole_body_velocity_ik/diagnostics/mapper_runtime_probe_v1',Float64MultiArray,queue_size=32)
    ident=rospy.Publisher('/whole_body_velocity_ik/diagnostics/mapper_thread_identity_v1',String,queue_size=1,latch=True)
    next_identity=0.
    while not rospy.is_shutdown():
        if time.monotonic()>=next_identity:
            next_identity=time.monotonic()+2.
            ident.publish(String(data=json.dumps(identities(parent_pid))))
        if not select.select([sock],[],[],.05)[0]:continue
        payload=sock.recv(8193)
        if not payload:break
        if len(payload)>8192 or len(payload)%8:continue
        data=array.array('d');data.frombytes(payload)
        if sys.byteorder!='little':data.byteswap()
        if len(data)<12 or len(data)!=12+7*int(data[11]):continue
        pub.publish(Float64MultiArray(data=data.tolist()))
    sock.close()

if __name__=='__main__':worker(int(sys.argv[1]),int(sys.argv[2]))
