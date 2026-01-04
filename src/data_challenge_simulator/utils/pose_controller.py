import rospy
import threading
import time
from typing import List

class PoseController:

    def __init__(self, robot, publish_frequency=200.0):
        self.robot = robot
        self.publish_frequency = publish_frequency
        self.publish_rate = rospy.Rate(publish_frequency)

        self.current_target_pose = [0.0, 0.0, 0.0, 0.0]

        self._active_evt = threading.Event() 
        self._shutdown = False
        self._pose_lock = threading.Lock()

        self._th = threading.Thread(target=self._publish_loop, daemon=True)
        self._th.start()

    def _publish_loop(self):
        while not rospy.is_shutdown() and not self._shutdown:

            if not self._active_evt.is_set():
                time.sleep(0.002)
                continue
            try:
                with self._pose_lock:
                    x, y, z, yaw = self.current_target_pose
                self.robot.control_command_pose_world(x, y, z, yaw)
                self.publish_rate.sleep()
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logwarn(f"[PoseController] publish err: {e}")
                time.sleep(0.01)

    def set_target_pose(self, pose: List[float]) -> bool:
        if len(pose) != 4:
            rospy.logwarn(f"Expected 4 values (x,y,z,yaw), got {len(pose)}")
            return False
        with self._pose_lock:
            self.current_target_pose = pose[:]
        return True
        
    def move_to_target(self, pose: List[float], hold_time: float = 1.0) -> bool:
        self.activate()
        self.set_target_pose(pose)
        time.sleep(hold_time)
        self.deactivate()
        return True
    
    def get_current_target_pose(self) -> List[float]:
        with self._pose_lock:
            return self.current_target_pose[:]

    def activate(self):
        self._active_evt.set()

    def deactivate(self):
        self._active_evt.clear()

    def smooth_transition_to(self, target_pose: List[float], duration: float = 1.0, num_steps: int = None):

        if len(target_pose) != 4:
            rospy.logwarn(f"Expected 4 values (x,y,z,yaw), got {len(target_pose)}")
            return False

        start_pose = self.get_current_target_pose()
        if num_steps is None:

            num_steps = max(5, int(duration * self.publish_frequency / 10))

        self.activate() 
        try:
            for i in range(num_steps + 1):
                if rospy.is_shutdown():
                    break
                alpha = float(i) / float(num_steps)
                interp = [start_pose[j] + alpha * (target_pose[j] - start_pose[j]) for j in range(4)]
                self.set_target_pose(interp)
                time.sleep(duration / max(1, num_steps))

            self.set_target_pose(target_pose)
        finally:
            self.deactivate()
        return True

    def goto(self, target_pose: List[float], max_time: float = 2.0):
        """简单到点：线性插值到目标并停止发送"""
        return self.smooth_transition_to(target_pose, duration=max_time)

    def stop(self):
        self._shutdown = True
        self._active_evt.set()
        if self._th.is_alive():
            self._th.join(timeout=2.0)
