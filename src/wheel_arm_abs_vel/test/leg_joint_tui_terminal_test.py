import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
"""Exercise the real Linux terminal driver with SGR mouse input, offline only."""
import os,sys,json,time,pty,select,fcntl,termios,struct,tempfile,signal
from pathlib import Path
if len(sys.argv)>1 and sys.argv[1]=='child':
    import leg_joint_tui_test as fixture
    from textual.widgets import Input,Button
    class Probe(fixture.ui.LegJointTui):
        def on_mount(self):
            super().on_mount();self.set_interval(.1,self.report)
        def report(self):
            data={'focus':getattr(self.focused,'id',None),'commands':len(fixture.commands),'positions':[list(r.position) for r in fixture.commands if r.command==1],
                  'regions':{w.id:[w.region.x+w.region.width//2+1,w.region.y+w.region.height//2+1] for w in self.query('Input, Button')}}
            path=Path(sys.argv[2]);tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data));os.replace(tmp,path)
    Probe().run(mouse=True)
    sys.exit(0)
with tempfile.TemporaryDirectory() as folder:
    status=Path(folder)/'status.json';pid,fd=pty.fork()
    if pid==0:
        os.environ['TERM']='xterm-256color'
        os.execv(sys.executable,[sys.executable,__file__,'child',str(status)])
    fcntl.ioctl(fd,termios.TIOCSWINSZ,struct.pack('HHHH',24,80,0,0))
    output=bytearray()
    def pump():
        if select.select([fd],[],[],.05)[0]:
            try: output.extend(os.read(fd,65536))
            except OSError: pass
    def wait(predicate):
        end=time.monotonic()+5.
        while time.monotonic()<end:
            pump()
            if status.exists():
                data=json.loads(status.read_text())
                if predicate(data):return data
        raise AssertionError('terminal mouse event did not produce expected UI state')
    def click(x,y):
        os.write(fd,('\x1b[<0;%d;%dM\x1b[<0;%d;%dm'%(x,y,x,y)).encode())
    try:
        data=wait(lambda d:len(d['regions'])==6)
        assert b'\x1b[?1006h' in output,'SGR mouse reporting was not enabled'
        click(*data['regions']['in_joint2'])
        wait(lambda d:d['focus']=='in_joint2')
        click(*data['regions']['publish'])
        wait(lambda d:d['commands']==1)
        os.write(fd,b'\x1b')
        data=wait(lambda d:d['commands']==2)
        click(*data['regions']['zero'])
        data=wait(lambda d:d['commands']==3)
        assert data['positions']==[[0.,0.,0.,0.],[0.,0.,0.,0.]]
        print('PASS real Linux PTY: SGR mouse enabled, input focus, Publish click, Zero click')
    finally:
        os.kill(pid,signal.SIGTERM);os.waitpid(pid,0);os.close(fd)
