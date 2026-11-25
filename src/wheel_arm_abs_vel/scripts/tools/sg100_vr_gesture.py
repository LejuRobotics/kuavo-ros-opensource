"""Touch confirmation for SG100; service calls run outside joystick callbacks."""
import math
import queue
import threading
import time


class GestureTouchGate:
    def __init__(self):
        self.reset()

    def reset(self):
        self.direction = 0
        self.started = None
        self.latched = False
        self.last_packet = None

    def update(self, msg, valid, now):
        if not valid or (self.last_packet is not None and now - self.last_packet > .15):
            self.reset()
            self.latched = True
        if not valid:
            return 0
        self.last_packet = now
        x = bool(msg.left_first_button_touched)
        y = bool(msg.left_second_button_touched)
        a = bool(msg.right_first_button_touched)
        b = bool(msg.right_second_button_touched)
        pressed = any((msg.left_first_button_pressed, msg.left_second_button_pressed,
                       msg.right_first_button_pressed, msg.right_second_button_pressed))
        sticks = (msg.left_x, msg.left_y, msg.right_x, msg.right_y)
        neutral = all(math.isfinite(v) and abs(v) <= .1 for v in sticks)
        if not any((x, y, a, b, pressed)):
            self.reset()
            return 0
        if x or pressed or (a and b) or not neutral:
            self.started = None
            self.latched = True
            return 0
        if self.latched:
            return 0
        if not y or a == b:
            self.started = None
            return 0
        direction = 1 if a else -1
        if direction != self.direction or self.started is None:
            self.direction, self.started = direction, now
            return 0
        if now - self.started < .8:
            return 0
        self.latched = True
        return direction


class Sg100VrGestureClient:
    def __init__(self):
        import rospy
        from kuavo_msgs.srv import SG100StepGesture
        self.rospy = rospy
        self.client = rospy.ServiceProxy('/sg100/vr_step_gesture', SG100StepGesture)
        self.gate = GestureTouchGate()
        self.requests = queue.Queue(maxsize=1)
        threading.Thread(target=self._run, daemon=True).start()

    def update(self, msg, valid):
        now = time.monotonic()
        direction = self.gate.update(msg, valid, now)
        if direction:
            try:
                self.requests.put_nowait((direction, now))
            except queue.Full:
                pass

    def _run(self):
        while not self.rospy.is_shutdown():
            try:
                direction, stamp = self.requests.get(timeout=.1)
            except queue.Empty:
                continue
            if time.monotonic() - stamp > .15:
                continue
            try:
                result = self.client(direction=direction, hand_side=2)
                if result.success:
                    self.rospy.loginfo('SG100 VR gesture -> id %d', result.current_id)
                else:
                    self.rospy.logwarn('SG100 VR gesture rejected: %s', result.message)
            except self.rospy.ServiceException as error:
                self.rospy.logwarn_throttle(2., 'SG100 VR gesture service failed: %s', error)
