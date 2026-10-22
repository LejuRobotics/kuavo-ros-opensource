#!/usr/bin/env python3
"""Run against an already running simulation; never commands a real robot."""
import time
import rospy
from kuavo_msgs.msg import SG100HandCommand
from kuavo_msgs.srv import SG100SwitchGesture, SG100StepGesture, SG100GetGestures
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool


def main():
    rospy.init_node('sg100_service_handover_test', anonymous=True)
    if rospy.get_param('/real', None) is not False:
        raise RuntimeError('This regression test requires /real=false')
    rospy.wait_for_service('/sim_start', timeout=40)
    if not rospy.ServiceProxy('/sim_start', SetBool)(True).success:
        raise RuntimeError('Simulation failed to start')
    rospy.wait_for_service('/sg100/vr_step_gesture', timeout=40)
    switch = rospy.ServiceProxy('/sg100/switch_gesture', SG100SwitchGesture)
    step = rospy.ServiceProxy('/sg100/step_gesture', SG100StepGesture)
    vr_step = rospy.ServiceProxy('/sg100/vr_step_gesture', SG100StepGesture)
    vr = rospy.ServiceProxy('/sg100/use_vr_control', SetBool)
    enable = rospy.ServiceProxy('/sg100/set_enabled', SetBool)
    get = rospy.ServiceProxy('/sg100/get_gestures', SG100GetGestures)
    commands = []
    sub = rospy.Subscriber('/sg100_hand_command', SG100HandCommand, commands.append)
    pub = rospy.Publisher('/sg100/trigger', Float32MultiArray, queue_size=1)

    def fresh():
        deadline = time.monotonic() + 5.
        while pub.get_num_connections() == 0:
            if time.monotonic() > deadline:
                raise RuntimeError('Trigger subscriber unavailable')
            time.sleep(.05)
        for _ in range(8):
            pub.publish(Float32MultiArray(data=[0., 0.]))
            time.sleep(.03)

    def check(condition, label):
        if not condition:
            raise AssertionError(label)
        print('PASS ' + label, flush=True)

    def expect_mask(left, right):
        time.sleep(.12)
        check(bool(commands) and (commands[-1].left_enable_mask,
              commands[-1].right_enable_mask) == (left, right), 'requested hands only')

    try:
        check(switch(1, 2).success, 'seed historical targets on both hands')
        check(enable(True).success, 'enable service output')
        fresh()
        result = vr_step(1, 2)
        check(result.success, 'VR step takes control')
        expect_mask(2047, 2047)
        check(switch(0, 0).success, 'left service reacquires control')
        expect_mask(2047, 0)
        check(step(1, 0).success, 'same service session continues')
        expect_mask(2047, 0)
        fresh()
        result = vr_step(1, 2)
        check(result.success, 'second VR step takes control')
        check(vr(False).success, 'select service without a new target')
        time.sleep(.15)
        count = len(commands)
        time.sleep(.2)
        check(len(commands) == count, 'no historical targets restored')
        total = get(-1, 0).total_count
        result_step = step(1, 1)
        check(result_step.success and result_step.current_id == (result.current_id+1) % total,
              'service step starts from current VR gesture')
        expect_mask(0, 2047)
        fresh()
        check(vr(True).success, 'explicit VR handover')
        check(switch(0, 0).success, 'service after explicit handover')
        expect_mask(2047, 0)
        check(enable(False).success, 'disable output')
        fresh()
        check(vr_step(1, 2).success, 'disabled VR selection accepted')
        time.sleep(.15)
        count = len(commands)
        time.sleep(.2)
        check(len(commands) == count, 'VR handover preserves manual disable')
        print('ALL PASS', flush=True)
    finally:
        enable(False)
        sub.unregister()
        pub.unregister()


if __name__ == '__main__':
    main()
