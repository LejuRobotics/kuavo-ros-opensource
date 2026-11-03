import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
"""Offline mouse/layout regression; ROS transport is replaced before mounting."""
import asyncio
import math
import threading
import rospy
import leg_joint_tui as ui
from wheel_arm_abs_vel.srv import ManualLowerBodyResponse as Response
from textual.widgets import Input, Button
commands=[]
ready=True
state=0
owned=0
lock=threading.Lock()
def fake(req):
    global state,owned,ready
    with lock:
        if req.command==1:
            commands.append(req);state=2;owned+=1;ready=False
        elif req.command==2:
            commands.append(req);state=5;ready=True
        return Response(accepted=True,request_id=owned,state=state,ready=ready,message='fixture',
                        position=[.1,-.1,0,0],measured_position=[.1,-.1,0,0],
                        target_position=[.2,-.2,0,0],lower=[0,-2,-1,-1],upper=[2,0,1,1])
rospy.init_node=lambda *a,**k: None
rospy.wait_for_service=lambda *a,**k: None
rospy.ServiceProxy=lambda *a,**k: fake
rospy.is_shutdown=lambda: False
async def test():
    app=ui.LegJointTui()
    async with app.run_test(size=(80,24)) as pilot:
        await pilot.pause(.5)
        assert [(b.id,str(b.label)) for b in app.query(Button)]==[('publish','发布'),('zero','归零')]
        for name in ui.JOINT_NAMES:
            field=app.query_one('#in_'+name,Input)
            assert field.value=='0.0'
            assert await pilot.click('#in_'+name)
            assert app.focused is field,'mouse did not focus input'
        field=app.query_one('#in_joint1',Input)
        field.value='nan'
        assert await pilot.click('#publish');await pilot.pause(.4)
        assert not commands,'nonfinite goal reached service'
        field.value='70'
        app.query_one('#in_joint2',Input).value='-75'
        app.query_one('#in_joint3',Input).value='5'
        assert await pilot.click('#publish');await pilot.pause(.5)
        assert len(commands)==1
        assert list(commands[0].position)==[math.radians(60),math.radians(-60),0.,0.]
        assert field.value=='60'
        assert await pilot.click('#publish');await pilot.pause(.3)
        assert len(commands)==1,'busy click queued another motion'
        await pilot.press('escape');await pilot.pause(.5)
        assert commands[-1].command==2 and commands[-1].request_id==1
        assert await pilot.click('#zero');await pilot.pause(.5)
        assert commands[-1].command==1 and list(commands[-1].position)==[0.,0.,0.,0.]
        assert all(app.query_one('#in_'+n,Input).value=='0.0' for n in ui.JOINT_NAMES)
        await pilot.press('q')
    assert app._stop.is_set()
    print('PASS 80x24 beta layout; two buttons; mouse input focus/publish/zero; legacy limits; finite validation; busy rejection; cancel/quit')
if __name__=='__main__':
    asyncio.run(test())
