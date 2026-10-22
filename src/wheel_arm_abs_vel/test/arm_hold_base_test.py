#!/usr/bin/env python3
"""#4486: real arm-operation methods must retain the independent wheel channel."""
import ast,threading,unittest
from pathlib import Path
from types import SimpleNamespace as NS,MethodType
import arm_torso_hold_test as arms
import fixed_follow_operation_test as fixed
from wheel_base_command_test import WheelBaseCommandMapper
ROOT=Path(__file__).resolve().parents[1]
class ArmHoldBaseTest(unittest.TestCase):
 def test_hold_resume_preserve_wheels_and_global_stop(self):
  for torso in (False,True):
   with self.subTest(torso=torso):
    f=arms.ArmTorsoHoldTest();f.setUp();n=f.n;n._torso_controls.enabled=torso
    methods=[x for x in ast.walk(ast.parse((ROOT/'scripts/ik_ros_velocity.py').read_text())) if isinstance(x,ast.FunctionDef) and x.name in ('_publish_wheel_base_command','_base_input_ready_locked')]
    def twist():return NS(linear=NS(x=0.,y=0.,z=0.),angular=NS(x=0.,y=0.,z=0.))
    scope={'time':fixed.clock,'Twist':twist,'rospy':NS(loginfo=lambda *a:None)}
    exec(compile(ast.Module(body=methods,type_ignores=[]),'<production>','exec'),scope)
    for method in methods:setattr(n,method.name,MethodType(scope[method.name],n))
    sent=[];n._wheel_base_pub=NS(publish=sent.append);n._wheel_base_mapper=WheelBaseCommandMapper({'alpha':0.});n.stop_event=threading.Event();n._wheel_mpc_stable_for_mm_cmd=lambda:True
    def publish(y,x=False,b=False):
     now=fixed.clock.now;n._joystick_receive_monotonic=now;n._wheel_mpc_obs_receive_monotonic=now;n._absolute_latest_measured_q_time=now
     n._absolute_latest_target['receive_time']=now;n._torso_controls.last_time=now;f.fixture.ack()
     n._publish_wheel_base_command(NS(left_y=y,left_x=0.,right_x=0.,right_y=0.,left_first_button_pressed=x,right_second_button_pressed=b))
    identity=(n._absolute_session_id,n._absolute_generation)
    publish(0.);publish(1.);self.assertEqual(sent[-1].linear.x,.4)
    f.pause()
    for i in range(100):
     fixed.clock.now+=.01;publish(1.,True,True)
     self.assertEqual(sent[-1].linear.x,.4)
     self.assertTrue(n._fixed_arm_hold);self.assertEqual(n._fixed_operation_mode(),0)
    self.assertEqual((n._absolute_session_id,n._absolute_generation),identity)
    self.assertEqual(n._torso_controls.enabled,torso)
    self.assertTrue(n._fixed_arm_operation(2).result)
    publish(1.);self.assertEqual(sent[-1].linear.x,.4)
    n.stop_event.set();publish(1.);self.assertEqual(sent[-1].linear.x,0.)
    n.stop_event.clear();publish(1.);self.assertEqual(sent[-1].linear.x,0.)
    publish(0.);publish(1.);self.assertEqual(sent[-1].linear.x,.4)
if __name__=='__main__':unittest.main()
