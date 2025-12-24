import threading
from queue import Queue
from kuavo_humanoid_sdk.kuavo.core.core import KuavoRobotCore
from kuavo_humanoid_sdk.interfaces.data_types import EndEffectorSide
from kuavo_humanoid_sdk.common.logger import SDKLogger
class LejuClawControl:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self):
        if not hasattr(self, '_initialized'):
            self._initialized = True
            self.queue = Queue()  # Initialize a queue to hold commands
            self._kuavo_core = KuavoRobotCore()
            self.thread = threading.Thread(target=self._process_queue)  # Create a thread to process the queue
            self.thread.daemon = True  # Set the thread as a daemon so it will exit when the main program exits
            self.thread.start()  # Start the thread
            self.last_cmd_position = {EndEffectorSide.LEFT: 0, EndEffectorSide.RIGHT: 0}
            self.last_cmd_torque = {EndEffectorSide.LEFT: 1.0, EndEffectorSide.RIGHT: 1.0}
            self.last_cmd_velocity = {EndEffectorSide.LEFT: 90, EndEffectorSide.RIGHT: 90}
    def control(self, position:list, velocity:list, torque:list, side:EndEffectorSide):
        self.queue.put((EndEffectorSide(side.value), position, velocity, torque))

    def release(self, side:EndEffectorSide):
        if side == EndEffectorSide.BOTH:
            self.queue.put((EndEffectorSide.BOTH, [0.0, 0.0], [90, 90], [1.0, 1.0]))
        else:    
            self.queue.put((EndEffectorSide(side.value), [0], [90], [1.0]))

    def _process_queue(self):
        while True:
            try:
                side, q, v, tau = self.queue.get()
                SDKLogger.debug(f"[LejuClawControl] Received command:  {side} to {q} with {v} and {tau}")
                postions = [self.last_cmd_position[EndEffectorSide.LEFT], self.last_cmd_position[EndEffectorSide.RIGHT]]
                velocities = [self.last_cmd_velocity[EndEffectorSide.LEFT], self.last_cmd_velocity[EndEffectorSide.RIGHT]]
                torques = [self.last_cmd_torque[EndEffectorSide.LEFT], self.last_cmd_torque[EndEffectorSide.RIGHT]]
                if side == EndEffectorSide.LEFT:
                    postions[0] = q[0]
                    velocities[0] = v[0]
                    torques[0] = tau[0]
                elif side == EndEffectorSide.RIGHT:    
                    postions[1] = q[0]
                    velocities[1] = v[0]
                    torques[1] = tau[0]
                else: # both
                    postions = q
                    velocities = v
                    torques = tau

                # call ros service.
                self._kuavo_core.control_leju_claw(postions, velocities, torques)
                # update last cmd
                self.last_cmd_position[EndEffectorSide.LEFT] = postions[0]
                self.last_cmd_position[EndEffectorSide.RIGHT] = postions[1]
                self.last_cmd_velocity[EndEffectorSide.LEFT] = velocities[0]
                self.last_cmd_velocity[EndEffectorSide.RIGHT] = velocities[1]
                self.last_cmd_torque[EndEffectorSide.LEFT] = torques[0]
                self.last_cmd_torque[EndEffectorSide.RIGHT] = torques[1]
                # mark task as done
                self.queue.task_done()
            except KeyboardInterrupt:
                break