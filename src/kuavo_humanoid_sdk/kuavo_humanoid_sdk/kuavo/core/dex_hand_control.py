import copy
import threading
from queue import Queue
from kuavo_humanoid_sdk.interfaces.data_types import EndEffectorSide
from kuavo_humanoid_sdk.kuavo.core.core import KuavoRobotCore
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.kuavo.core.ros.param import make_robot_param, EndEffectorType
class DexHandControl:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, '_initialized'):
            self.queue = Queue()  # Initialize a queue to hold commands
            self._kuavo_core = KuavoRobotCore()
            self.thread = threading.Thread(target=self._process_queue)  # Create a thread to process the queue
            self.thread.daemon = True  # Set the thread as a daemon so it will exit when the main program exits
            self.thread.start()  # Start the thread
            kuavo_info = make_robot_param()
            self._ee_type = kuavo_info['end_effector_type']
            # Initialize last command position, torque, and velocity
            self.last_cmd_position = {EndEffectorSide.LEFT: [0] * 6, EndEffectorSide.RIGHT: [0] * 6}
            self._initialized = True

    def control(self, target_positions: list, side: EndEffectorSide):
        self.queue.put(('position', EndEffectorSide(side.value), target_positions))

    def control_velocity(self, target_velocities: list, side: EndEffectorSide):
        self.queue.put(('velocity', EndEffectorSide(side.value), target_velocities))

    def make_gestures(self, gestures:list)->bool:
        """
            Make a gesture for the dexhand.
            Args:
                gestures: list of gestures to make.
                    [{'gesture_name': 'name', 'side': EndEffectorSide.LEFT},...]
        """
        exec_gs = []
        for gs in gestures:
            side = gs['hand_side']
            gesture_name = gs['gesture_name']
            if side == EndEffectorSide.LEFT:
                exec_gs.append({'gesture_name': gesture_name, 'hand_side': 0})
            elif side == EndEffectorSide.RIGHT:
                exec_gs.append({'gesture_name': gesture_name, 'hand_side': 1})
            elif side == EndEffectorSide.BOTH:
                exec_gs.append({'gesture_name': gesture_name, 'hand_side': 2})

        if len(exec_gs) == 0:
            SDKLogger.error('No gestures to make')
            return False
        
        # Make gestures
        self._kuavo_core.execute_gesture(exec_gs)

    def get_gesture_names(self)->list:
        """
            Get the names of all gestures.
        """
        gs = self._kuavo_core.get_gesture_names()
        if not gs:
            return None
        return gs
    
    
    def _process_dexhand_command(self, command, side, data):
        if command == 'position':
            pos = self.last_cmd_position[EndEffectorSide.LEFT] + self.last_cmd_position[EndEffectorSide.RIGHT]
            if side == EndEffectorSide.BOTH:
                pos = copy.deepcopy(data)
            elif side == EndEffectorSide.LEFT:
                pos[:6] = data
            elif side == EndEffectorSide.RIGHT:
                pos[6:] = data
            else:
                return
            self._kuavo_core.control_robot_dexhand(left_position=pos[:6], right_position=pos[6:])                       
            self.last_cmd_position[EndEffectorSide.LEFT] = pos[:6]
            self.last_cmd_position[EndEffectorSide.RIGHT] = pos[6:]

    def _process_touch_dexhand_command(self, command, side, data):
        if command == 'position':
            ctrl_mode = 0
            if side == EndEffectorSide.BOTH:
                self._kuavo_core.robot_dexhand_command(data, ctrl_mode, 2)
            elif side == EndEffectorSide.LEFT:
                self._kuavo_core.robot_dexhand_command(data, ctrl_mode, 0)
            elif side == EndEffectorSide.RIGHT:
                self._kuavo_core.robot_dexhand_command(data, ctrl_mode, 1)
        elif command == 'velocity':
            ctrl_mode = 0
            if side == EndEffectorSide.BOTH:
                self._kuavo_core.robot_dexhand_command(data, ctrl_mode, 2)
            elif side == EndEffectorSide.LEFT:
                self._kuavo_core.robot_dexhand_command(data, ctrl_mode, 0)
            elif side == EndEffectorSide.RIGHT:
                self._kuavo_core.robot_dexhand_command(data, ctrl_mode, 1)

    def _process_queue(self):
        while True:
            try:
                command, side, data = self.queue.get()  # This will block until an item is available in the queue
                SDKLogger.debug(f'[DexHandControl] Received command: {command}, for side: {side}, with data: {data}')
                if self._ee_type == EndEffectorType.QIANGNAO:
                    self._process_dexhand_command(command, side, data)
                elif self._ee_type == EndEffectorType.QIANGNAO_TOUCH:
                    self._process_touch_dexhand_command(command, side, data)    
                self.queue.task_done()
            except KeyboardInterrupt:
                break